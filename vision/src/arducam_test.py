"""
MERO_AI_ROBOT 메인 실행 파일
─────────────────────────────
실행: python vision/src/arducam_test.py

동작 순서:
  1. YOLO 모델 로드 (best.pt 또는 best.engine)
  2. 캘리브레이션 파일 로드 (있으면 mm 좌표 계산)
  3. Arducam USB 카메라 열기
  4. 매 프레임: 탐지 → 트래킹 → 타겟 선택 → 두 보드에 전송

시리얼 연결 구조:
  Jetson → /dev/ttyUSB0  → ESP32   (UGV02 바퀴 제어, Waveshare JSON)
  Jetson → /dev/ttyACM0  → OpenRB  (Dynamixel 팔·그리퍼, 명령 JSON)

바퀴 제어 방식:
  Python이 mx/my 기반으로 속도 계산 → {"T":1, "L":speed, "R":speed} 직접 전송
  타겟이 ARRIVE_THRESHOLD_MM 이내 → 정지 + OpenRB에 pick 명령
  타겟 없음 → 정지 + OpenRB에 idle 명령
"""

import cv2
import os
import json
import time
import threading
import serial
from enum import Enum
from ultralytics import YOLO

# ──────────────────────────────────────────────
# 모델 로드
# ──────────────────────────────────────────────
# __file__ = src/arducam_test.py
# BASE_DIR  = MERO_AI_ROBOT/ (프로젝트 루트)
BASE_DIR   = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODEL_PATH = os.path.join(BASE_DIR, "model", "best.pt")
# Jetson에서 TensorRT 변환 후에는 아래처럼 변경:
# MODEL_PATH = os.path.join(BASE_DIR, "model", "best.engine")
model      = YOLO(MODEL_PATH)

# ──────────────────────────────────────────────
# 캘리브레이션 로드
# ──────────────────────────────────────────────
# calibration.py 를 먼저 실행하면 model/calibration.json 이 생성됨
# 파일이 있으면 픽셀 좌표를 mm 좌표로 변환할 수 있음
# 없으면 픽셀 좌표만 사용 (기능은 정상 동작)
CALIB_PATH   = os.path.join(BASE_DIR, "model", "calibration.json")
MM_PER_PIXEL = None   # 1픽셀 = 몇 mm인지 비율
FRAME_W      = None   # 캘리브레이션 당시 프레임 가로 크기
FRAME_H      = None   # 캘리브레이션 당시 프레임 세로 크기

if os.path.exists(CALIB_PATH):
    with open(CALIB_PATH) as f:
        calib = json.load(f)
    MM_PER_PIXEL = calib["mm_per_pixel"]
    FRAME_W      = calib["frame_width"]
    FRAME_H      = calib["frame_height"]
    print(f"✅ 캘리브레이션 로드: {MM_PER_PIXEL:.4f} mm/pixel")
else:
    print("⚠️ calibration.json 없음 — 픽셀 좌표만 사용합니다.")
    print("   (calibration.py 를 먼저 실행하면 mm 좌표도 사용 가능)")

def pixel_to_mm(cx, cy):
    """
    픽셀 좌표(cx, cy)를 이미지 중심 기준 mm 좌표(mx, my)로 변환.

    반환 좌표 기준:
      - (0, 0) = 카메라 정중앙 (테이블 중심)
      - mx 양수 = 오른쪽, 음수 = 왼쪽
      - my 양수 = 아래쪽, 음수 = 위쪽

    캘리브레이션 파일 없으면 (None, None) 반환.
    """
    if MM_PER_PIXEL is None:
        return None, None
    w  = FRAME_W or 640
    h  = FRAME_H or 480
    mx = round((cx - w / 2) * MM_PER_PIXEL, 1)
    my = round((cy - h / 2) * MM_PER_PIXEL, 1)
    return mx, my

# ──────────────────────────────────────────────
# 타겟 선택 (우선순위 로직)
# ──────────────────────────────────────────────
def select_target(objects: list) -> dict | None:
    """
    탐지된 물체 중 로봇이 집을 대상 1개를 선택해 반환.
    아무것도 탐지되지 않으면 None 반환.

    현재 기준: 신뢰도(conf)가 가장 높은 물체
    변경 방법:
      - 화면 중앙에 가장 가까운 것: key=lambda o: abs(o["cx"] - FRAME_W/2)
      - 특정 클래스 우선: if o["cls"] == "d8" 등 필터링 후 선택
    """
    if not objects:
        return None
    return max(objects, key=lambda o: o["conf"])

# ──────────────────────────────────────────────
# 시리얼 초기화
# Jetson → ESP32  (/dev/ttyUSB0): 바퀴 제어 (Waveshare JSON)
# Jetson → OpenRB (/dev/ttyACM0): 팔·그리퍼 제어 (명령 JSON)
#
# Jetson에서 실행 전 권한 열기 (USB 연결할 때마다):
#   sudo chmod 666 /dev/ttyUSB0
#   sudo chmod 666 /dev/ttyACM0
# ──────────────────────────────────────────────
ESP32_PORT  = "/dev/ttyUSB0"   # ESP32 (UGV02 바퀴) — 라이다 있으면 ttyACM1일 수 있음
OPENRB_PORT = "/dev/ttyACM0"   # OpenRB (Dynamixel 팔·그리퍼)
BAUD_RATE   = 115200

# ──────────────────────────────────────────────
# 바퀴 제어 파라미터
# ──────────────────────────────────────────────
ARRIVE_THRESHOLD_MM = 30.0   # 이 거리 이내면 도착으로 판단 → 정지
MOVE_SPEED          = 0.3    # 기본 이동 속도 (Waveshare: -0.5 ~ 0.5)
SLOW_SPEED          = 0.15   # 근접 100mm 이내 감속 속도
MAX_MX              = 200.0  # 카메라 좌우 반폭 추정값(mm) — 캘리브레이션 후 조정
MAX_MY              = 150.0  # 카메라 상하 반폭 추정값(mm) — 캘리브레이션 후 조정

# ──────────────────────────────────────────────
# 로봇 상태 머신
# ──────────────────────────────────────────────
class RobotState(Enum):
    SEARCHING = "탐색중"   # 타겟 탐지 + 이동
    WAITING   = "팔동작대기"  # pick 명령 전송 후 OpenRB 시퀀스 완료 대기

PICK_WAIT_SECS = 8.0   # OpenRB pick→drop→return 예상 소요 시간 (실물 테스트 후 조정)

robot_state  = RobotState.SEARCHING
pick_sent_at = 0.0

def _open_serial(port):
    """시리얼 포트 열기. 실패해도 에러 없이 None 반환."""
    try:
        s = serial.Serial(port, BAUD_RATE, timeout=1)
        print(f"✅ 시리얼 연결 성공: {port}")
        return s
    except Exception:
        print(f"⚠️ 시리얼 연결 실패: {port} — 해당 보드 없이 실행합니다.")
        return None

ser_esp32  = _open_serial(ESP32_PORT)
ser_openrb = _open_serial(OPENRB_PORT)

# ──────────────────────────────────────────────
# ESP32 수신 스레드 (배터리 전압 모니터링)
# ESP32는 주기적으로 {"T":1001, ..., "v":1137} 전송
# v = 배터리 전압 × 100 (1137 = 11.37V)
# ──────────────────────────────────────────────
battery_v = None  # 현재 배터리 전압 (V), 수신 전까지 None

def _read_esp32_loop():
    global battery_v
    while True:
        if ser_esp32 is None or not ser_esp32.is_open:
            time.sleep(0.5)
            continue
        try:
            if ser_esp32.in_waiting > 0:
                raw  = ser_esp32.readline()
                data = json.loads(raw.decode("utf-8").strip())
                if data.get("T") == 1001:
                    v_raw = data.get("v") or data.get("V")  # 소문자·대문자 모두 대응
                    if v_raw is not None:
                        battery_v = v_raw / 100.0
        except Exception:
            pass
        time.sleep(0.01)

threading.Thread(target=_read_esp32_loop, daemon=True).start()

# ──────────────────────────────────────────────
# OpenRB 수신 스레드 (팔 시퀀스 완료 신호)
# OpenRB가 RETURN→IDLE 시 {"status":"done"} 전송
# → Python WAITING 상태 즉시 해제 (타이머 대기 불필요)
# ──────────────────────────────────────────────
openrb_done = False  # OpenRB 완료 신호 수신 여부

def _read_openrb_loop():
    global openrb_done
    while True:
        if ser_openrb is None or not ser_openrb.is_open:
            time.sleep(0.5)
            continue
        try:
            if ser_openrb.in_waiting > 0:
                raw  = ser_openrb.readline()
                data = json.loads(raw.decode("utf-8").strip())
                if data.get("status") == "done":
                    openrb_done = True
                    print("[OpenRB] 팔 시퀀스 완료 신호 수신")
        except Exception:
            pass
        time.sleep(0.01)

threading.Thread(target=_read_openrb_loop, daemon=True).start()

def control_wheels(target: dict | None):
    """
    target의 mx/my 기반으로 바퀴 속도 계산 → ESP32에 Waveshare 포맷 전송.
    형식: {"T":1, "L": 좌속도, "R": 우속도}

    차동 조향: mx/my 비율로 L/R 속도 차이를 계산해 전진하면서 동시에 방향 조정.
      - turn = mx / MAX_MX  (-1=좌, +1=우)
      - fwd  = my / MAX_MY  (-1=후진, +1=전진)
      - L = speed * (fwd + turn),  R = speed * (fwd - turn)
      - 타겟 없거나 mx=None → 정지
      - dist < ARRIVE_THRESHOLD_MM → 정지
    """
    if ser_esp32 is None or not ser_esp32.is_open:
        return

    L, R = 0.0, 0.0

    if target is not None and target.get("mx") is not None:
        mx   = target["mx"]
        my   = target["my"]
        dist = (mx ** 2 + my ** 2) ** 0.5

        if dist >= ARRIVE_THRESHOLD_MM:
            speed = SLOW_SPEED if dist < 100.0 else MOVE_SPEED

            turn = max(-1.0, min(1.0, mx / MAX_MX))  # 좌우 방향 비율
            fwd  = max(-1.0, min(1.0, my / MAX_MY))  # 전후 방향 비율

            L = speed * (fwd + turn)
            R = speed * (fwd - turn)

            # Waveshare 속도 범위 제한
            L = max(-0.5, min(0.5, L))
            R = max(-0.5, min(0.5, R))

    cmd = {"T": 1, "L": round(L, 2), "R": round(R, 2)}
    ser_esp32.write((json.dumps(cmd) + "\n").encode())

def send_to_openrb(target: dict | None, at_target: bool = False):
    """
    팔·그리퍼 명령을 OpenRB로 전송.

    at_target=True (타겟 도달) → pick 명령
    at_target=False 또는 타겟 없음 → idle 명령

    형식:
      {"cmd": "pick", "cls": "d8", "mx": 12.3, "my": -5.1}
      {"cmd": "idle"}
    """
    if ser_openrb is None or not ser_openrb.is_open:
        return
    if target is not None and at_target:
        payload = json.dumps({
            "cmd": "pick",
            "cls": target["cls"],
            "mx":  target.get("mx", 0),
            "my":  target.get("my", 0),
        }) + "\n"
    else:
        payload = json.dumps({"cmd": "idle"}) + "\n"
    ser_openrb.write(payload.encode())

# ──────────────────────────────────────────────
# 카메라 초기화
# ──────────────────────────────────────────────
# 노트북 내장캠 = 0, Arducam USB = 1 (연결 환경에 따라 다를 수 있음)
CAMERA_INDEX = 1
cap = cv2.VideoCapture(CAMERA_INDEX)

if not cap.isOpened():
    # 1번 카메라 없으면 0번(내장캠)으로 자동 전환
    print(f"⚠️ {CAMERA_INDEX}번 카메라를 열 수 없습니다. 0번으로 재시도합니다.")
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("❌ 사용 가능한 카메라를 찾을 수 없습니다.")
        exit()

WINDOW_NAME = "MERO_AI_ROBOT_TEST"
cv2.destroyAllWindows()
cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE)

print("🚀 실시간 트래킹 시작! 종료하려면 'q'를 누르거나 Ctrl+C를 누르세요.")

# ──────────────────────────────────────────────
# 메인 트래킹 루프
# ──────────────────────────────────────────────
try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("⚠️ 프레임을 읽을 수 없습니다. 카메라 연결을 확인해주세요.")
            break

        # YOLOv8 트래킹
        # persist=True  : 프레임이 바뀌어도 같은 물체에 같은 ID 유지 (트래킹 핵심)
        # conf=0.5      : 신뢰도 50% 미만 탐지 무시
        # verbose=False : 매 프레임 콘솔 로그 억제
        results = model.track(frame, persist=True, conf=0.5, verbose=False)
        boxes   = results[0].boxes
        detected = []  # 이번 프레임에서 탐지된 물체 목록

        if boxes is not None and len(boxes) > 0:
            # ids: 트래킹 ID 배열. 트래커가 아직 ID를 배정 못한 경우(첫 몇 프레임) None일 수 있음
            ids = boxes.id

            for i, box in enumerate(boxes):
                cls_id   = int(box.cls[0])
                cls_name = model.names[cls_id]   # 클래스 이름 (d6, d8, apple 등)
                conf     = float(box.conf[0])     # 탐지 신뢰도 (0.0 ~ 1.0)

                # 바운딩박스 좌표 → 중심 좌표 계산
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                cx = (x1 + x2) / 2   # 중심 X (픽셀)
                cy = (y1 + y2) / 2   # 중심 Y (픽셀)

                # 픽셀 → mm 변환 (캘리브레이션 파일 없으면 None 반환)
                mx, my = pixel_to_mm(cx, cy)

                # 트래킹 ID (-1 이면 아직 ID 미배정)
                track_id = int(ids[i]) if ids is not None else -1

                # 로봇 제어에 필요한 정보를 딕셔너리로 정리
                obj = {
                    "id":   track_id,
                    "cls":  cls_name,
                    "cx":   round(cx, 1),  # 픽셀 좌표
                    "cy":   round(cy, 1),
                    "conf": round(conf, 2),
                }
                if mx is not None:
                    obj["mx"] = mx   # mm 좌표 (이미지 중심 기준)
                    obj["my"] = my

                detected.append(obj)

                # 콘솔 출력
                coord_str = f"({cx:.1f}px, {cy:.1f}px)"
                if mx is not None:
                    coord_str += f" = ({mx:.1f}mm, {my:.1f}mm)"
                print(f"[트래킹] ID={track_id} | {cls_name} (conf={conf:.2f}) | {coord_str}")

        # 이번 프레임에서 집을 물체 1개 선택
        target = select_target(detected)

        # 타겟 도달 여부 판단 (캘리브레이션 있을 때만 가능)
        at_target = False
        if target and target.get("mx") is not None:
            dist = (target["mx"] ** 2 + target["my"] ** 2) ** 0.5
            at_target = dist < ARRIVE_THRESHOLD_MM

        # ── 상태 머신 ────────────────────────────
        global robot_state, pick_sent_at, openrb_done

        if robot_state == RobotState.WAITING:
            elapsed = time.time() - pick_sent_at
            # 완료 신호 수신 시 즉시 전환, 없으면 타이머로 fallback
            if openrb_done:
                robot_state  = RobotState.SEARCHING
                openrb_done  = False
                print(f"[상태] WAITING → SEARCHING (완료 신호 수신, {elapsed:.1f}s)")
            elif elapsed >= PICK_WAIT_SECS:
                robot_state = RobotState.SEARCHING
                print(f"[상태] WAITING → SEARCHING (타이머 fallback, {elapsed:.1f}s)")

        if robot_state == RobotState.SEARCHING:
            if target and target.get("mx") is not None:
                status = "도달" if at_target else f"이동중 ({dist:.0f}mm)"
                print(f"[타겟]   ID={target['id']} | {target['cls']} | {status}")

            control_wheels(target)   # 바퀴: 타겟 추적

            if at_target:
                send_to_openrb(target, at_target=True)   # pick 명령 1회 전송
                robot_state  = RobotState.WAITING
                pick_sent_at = time.time()
                openrb_done  = False   # 이전 완료 신호 초기화
                print(f"[상태] SEARCHING → WAITING (pick 전송: {target['cls']})")
            else:
                send_to_openrb(None)   # idle

        else:  # WAITING
            elapsed = time.time() - pick_sent_at
            print(f"[상태] 팔 동작 대기중... ({elapsed:.1f}s / {PICK_WAIT_SECS}s)")
            control_wheels(None)   # 바퀴 정지
            send_to_openrb(None)   # idle

        # ── 시각화 ──────────────────────────────
        # YOLO 기본 오버레이 (바운딩박스 + 클래스명 + ID + 신뢰도)
        annotated_frame = results[0].plot()

        # 타겟으로 선택된 물체에 노란 테두리 + "TARGET" 텍스트 추가
        if target and boxes is not None:
            ids = boxes.id
            for i, box in enumerate(boxes):
                tid = int(ids[i]) if ids is not None else -1
                if tid == target["id"]:
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    cv2.rectangle(annotated_frame,
                                  (int(x1) - 4, int(y1) - 4),
                                  (int(x2) + 4, int(y2) + 4),
                                  (0, 255, 255), 3)   # 노란 테두리
                    cv2.putText(annotated_frame, "TARGET",
                                (int(x1), int(y1) - 12),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        # ── 상태 오버레이 ────────────────────────
        h, w = annotated_frame.shape[:2]

        # 상태에 따라 색상·텍스트 결정
        if robot_state == RobotState.SEARCHING:
            state_color = (0, 255, 0)      # 초록
            state_text  = f"STATE: SEARCHING"
        else:
            elapsed     = time.time() - pick_sent_at
            remain      = max(0.0, PICK_WAIT_SECS - elapsed)
            state_color = (0, 165, 255)    # 주황
            state_text  = f"STATE: WAITING  {elapsed:.1f}s / {PICK_WAIT_SECS:.0f}s"

        # 반투명 검정 배경 (텍스트 가독성)
        overlay = annotated_frame.copy()
        cv2.rectangle(overlay, (0, h - 80), (w, h), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.5, annotated_frame, 0.5, 0, annotated_frame)

        # 상태 텍스트
        cv2.putText(annotated_frame, state_text,
                    (10, h - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, state_color, 2)

        # 타겟 정보 (있을 때만)
        if target:
            if target.get("mx") is not None:
                dist_now = (target["mx"]**2 + target["my"]**2)**0.5
                tgt_text = f"TARGET: {target['cls']}  dist={dist_now:.0f}mm  conf={target['conf']:.2f}"
            else:
                tgt_text = f"TARGET: {target['cls']}  conf={target['conf']:.2f}"
            cv2.putText(annotated_frame, tgt_text,
                        (10, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        # 배터리 전압 (우측 상단)
        if battery_v is not None:
            if battery_v >= 11.5:
                batt_color = (0, 255, 0)    # 초록 (양호)
            elif battery_v >= 10.0:
                batt_color = (0, 165, 255)  # 주황 (주의)
            else:
                batt_color = (0, 0, 255)    # 빨강 (위험 — 충전 필요)
            batt_text = f"BAT: {battery_v:.2f}V"
        else:
            batt_color = (128, 128, 128)    # 회색 (수신 대기)
            batt_text  = "BAT: --"
        cv2.putText(annotated_frame, batt_text,
                    (w - 150, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, batt_color, 2)

        cv2.imshow(WINDOW_NAME, annotated_frame)

        # 'q' 누르면 루프 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # 'q' 종료든 Ctrl+C든 예외 발생이든 반드시 자원 해제
    cap.release()
    if ser_esp32  and ser_esp32.is_open:  ser_esp32.close()
    if ser_openrb and ser_openrb.is_open: ser_openrb.close()
    cv2.destroyAllWindows()
    print("카메라 및 시리얼 자원이 안전하게 해제되었습니다.")
