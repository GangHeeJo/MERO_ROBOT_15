"""
MERO_AI_ROBOT 메인 실행 파일
─────────────────────────────
실행: python vision/src/main.py [--cls d8]

상태 머신:
  SEARCHING      — 타겟 탐지 + 이동
  GRIPPING       — grip 명령 전송 후 gripped 신호 대기 (바퀴 정지)
  GO_TO_STORAGE  — 바퀴로 보관함까지 고정 경로 이동
  DROPPING       — drop 명령 전송 후 done 신호 대기 (바퀴 정지)

시리얼:
  /dev/ttyACM0 → ESP32  (UGV02 바퀴)   {"T":1, "L":speed, "R":speed}
  /dev/ttyACM1 → OpenRB (팔·그리퍼)    {"cmd":"grip"/"drop"/"idle"}

OpenRB 응답:
  {"status":"gripped"} — grip 시퀀스 완료 (물체 집음)
  {"status":"done"}    — drop+return 완료
"""

import argparse
import cv2
import os
import json
import time
import threading
import serial
from enum import Enum
from ultralytics import YOLO

# ── 인수 파싱 ───────────────────────────────────────────
# 경기 당일 오전 공지된 타겟 클래스를 지정
# 예: python main.py --cls d8
parser = argparse.ArgumentParser()
parser.add_argument('--cls', default=None,
                    help='타겟 클래스 (예: d8, d12, apple). 미지정 시 모든 클래스 대상')
args       = parser.parse_args()
TARGET_CLS = args.cls   # None이면 필터 없이 전체 탐지

# ── 모델 로드 ────────────────────────────────────────────
BASE_DIR   = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODEL_PATH = os.path.join(BASE_DIR, "model", "best.pt")
# Jetson TensorRT 변환 후:
# MODEL_PATH = os.path.join(BASE_DIR, "model", "best.engine")
model = YOLO(MODEL_PATH)

# ── 캘리브레이션 로드 ────────────────────────────────────
CALIB_PATH   = os.path.join(BASE_DIR, "model", "calibration.json")
MM_PER_PIXEL = None
FRAME_W      = None
FRAME_H      = None

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
    """픽셀 좌표 → 이미지 중심 기준 mm 좌표."""
    if MM_PER_PIXEL is None:
        return None, None
    w  = FRAME_W or 640
    h  = FRAME_H or 480
    mx = round((cx - w / 2) * MM_PER_PIXEL, 1)
    my = round((cy - h / 2) * MM_PER_PIXEL, 1)
    return mx, my


def select_target(objects: list) -> dict | None:
    """
    탐지된 물체 중 타겟 1개 선택.
    --cls 지정 시 해당 클래스만 필터링 후 최고 신뢰도 반환.
    """
    if not objects:
        return None
    if TARGET_CLS:
        objects = [o for o in objects if o['cls'] == TARGET_CLS]
    if not objects:
        return None
    return max(objects, key=lambda o: o['conf'])


# ── 시리얼 포트 ──────────────────────────────────────────
ESP32_PORT  = "/dev/ttyACM0"   # CH343 드라이버 → ACM (ttyUSB 아님)
OPENRB_PORT = "/dev/ttyACM1"
BAUD_RATE   = 115200

# ── 바퀴 제어 파라미터 ───────────────────────────────────
ARRIVE_THRESHOLD_MM = 30.0   # 이 거리 이내면 도착 판단
MOVE_SPEED          = 0.3    # 기본 이동 속도
SLOW_SPEED          = 0.15   # 근접 100mm 이내 감속
MAX_MX              = 200.0  # 카메라 좌우 반폭 추정(mm)
MAX_MY              = 150.0  # 카메라 상하 반폭 추정(mm)

# ── 보관함 이동 고정 경로 파라미터 ──────────────────────
# 경기장: 4m×4m / 출발=우하단 / 보관함=좌하단
# 순서: ① 좌회전 → ② 직진
# TODO: 실측 후 조정
STORAGE_TURN_SECS   = 2.0    # 회전 시간 (초)
STORAGE_DRIVE_SECS  = 3.0    # 직진 시간 (초)
STORAGE_TURN_SPEED  = 0.25   # 회전 속도 (L=-값, R=+값 → 좌회전)
STORAGE_DRIVE_SPEED = 0.3    # 직진 속도

# ── 상태 머신 ────────────────────────────────────────────
class RobotState(Enum):
    SEARCHING     = "탐색중"
    GRIPPING      = "집는중"
    GO_TO_STORAGE = "보관함이동"
    DROPPING      = "내려놓는중"

robot_state         = RobotState.SEARCHING
grip_sent_at        = 0.0
storage_phase       = 0     # 0=회전, 1=직진
storage_phase_start = 0.0

# ── 시리얼 연결 ──────────────────────────────────────────
def _open_serial(port):
    try:
        s = serial.Serial(port, BAUD_RATE, timeout=1)
        print(f"✅ 시리얼 연결 성공: {port}")
        return s
    except Exception:
        print(f"⚠️ 시리얼 연결 실패: {port} — 해당 보드 없이 실행합니다.")
        return None

ser_esp32  = _open_serial(ESP32_PORT)
ser_openrb = _open_serial(OPENRB_PORT)

# ── ESP32 수신 스레드 (배터리 모니터링) ─────────────────
battery_v = None

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
                    v_raw = data.get("v") or data.get("V")
                    if v_raw is not None:
                        battery_v = v_raw / 100.0
        except Exception:
            pass
        time.sleep(0.01)

threading.Thread(target=_read_esp32_loop, daemon=True).start()

# ── OpenRB 수신 스레드 (팔 완료 신호) ───────────────────
openrb_gripped = False   # {"status":"gripped"} 수신 시 True
openrb_done    = False   # {"status":"done"}    수신 시 True

def _read_openrb_loop():
    global openrb_gripped, openrb_done
    while True:
        if ser_openrb is None or not ser_openrb.is_open:
            time.sleep(0.5)
            continue
        try:
            if ser_openrb.in_waiting > 0:
                raw  = ser_openrb.readline()
                data = json.loads(raw.decode("utf-8", errors="ignore").strip())
                if data.get("status") == "gripped":
                    openrb_gripped = True
                    print("\n[OpenRB] 집기 완료 신호 수신")
                elif data.get("status") == "done":
                    openrb_done = True
                    print("\n[OpenRB] 내려놓기 완료 신호 수신")
        except Exception:
            pass
        time.sleep(0.01)

threading.Thread(target=_read_openrb_loop, daemon=True).start()


# ── 바퀴 제어 ────────────────────────────────────────────
def control_wheels(target: dict | None, override_l: float | None = None, override_r: float | None = None):
    """
    override 지정 시 해당 속도 직접 전송 (보관함 이동 고정 경로용).
    미지정 시 target mx/my 기반 차동 조향.
    """
    if ser_esp32 is None or not ser_esp32.is_open:
        return

    if override_l is not None:
        L, R = override_l, override_r
    elif target is not None and target.get("mx") is not None:
        mx   = target["mx"]
        my   = target["my"]
        dist = (mx ** 2 + my ** 2) ** 0.5

        if dist >= ARRIVE_THRESHOLD_MM:
            speed = SLOW_SPEED if dist < 100.0 else MOVE_SPEED
            turn  = max(-1.0, min(1.0, mx / MAX_MX))
            fwd   = max(-1.0, min(1.0, my / MAX_MY))
            L = max(-0.5, min(0.5, speed * (fwd + turn)))
            R = max(-0.5, min(0.5, speed * (fwd - turn)))
        else:
            L, R = 0.0, 0.0
    else:
        L, R = 0.0, 0.0

    cmd = {"T": 1, "L": round(L, 2), "R": round(R, 2)}
    ser_esp32.write((json.dumps(cmd) + "\n").encode())


# ── OpenRB 명령 전송 ─────────────────────────────────────
def send_grip(target: dict):
    """물체 집기 명령 → OpenRB IDLE→GRIPPING 시퀀스 시작."""
    if ser_openrb is None or not ser_openrb.is_open:
        return
    payload = json.dumps({
        "cmd": "grip",
        "cls": target["cls"],
        "mx":  target.get("mx", 0),
        "my":  target.get("my", 0),
    }) + "\n"
    ser_openrb.write(payload.encode())

def send_drop():
    """보관함에 내려놓기 명령 → OpenRB HOLDING→DROPPING 시퀀스 시작."""
    if ser_openrb is None or not ser_openrb.is_open:
        return
    ser_openrb.write((json.dumps({"cmd": "drop"}) + "\n").encode())

def send_idle():
    if ser_openrb is None or not ser_openrb.is_open:
        return
    ser_openrb.write((json.dumps({"cmd": "idle"}) + "\n").encode())


# ── 카메라 초기화 ────────────────────────────────────────
CAMERA_INDEX = 1
cap = cv2.VideoCapture(CAMERA_INDEX)
if not cap.isOpened():
    print(f"⚠️ {CAMERA_INDEX}번 카메라를 열 수 없습니다. 0번으로 재시도합니다.")
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("❌ 사용 가능한 카메라를 찾을 수 없습니다.")
        exit()

HEADLESS    = os.environ.get("DISPLAY") is None
WINDOW_NAME = "MERO_AI_ROBOT"
if not HEADLESS:
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE)

print(f"🚀 실시간 트래킹 시작! 타겟 클래스: {TARGET_CLS or '전체'}")
if HEADLESS:
    print("ℹ️ 헤드리스 모드 — 터미널 로그만 출력합니다.")

fps_counter = 0
fps_display = 0.0
fps_timer   = time.time()

# ── 메인 루프 ────────────────────────────────────────────
try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("⚠️ 프레임을 읽을 수 없습니다. 카메라 연결을 확인해주세요.")
            break

        results  = model.track(frame, persist=True, conf=0.5, verbose=False)
        boxes    = results[0].boxes
        detected = []

        if boxes is not None and len(boxes) > 0:
            ids = boxes.id
            for i, box in enumerate(boxes):
                cls_id   = int(box.cls[0])
                cls_name = model.names[cls_id]
                conf     = float(box.conf[0])
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                cx = (x1 + x2) / 2
                cy = (y1 + y2) / 2
                mx, my   = pixel_to_mm(cx, cy)
                track_id = int(ids[i]) if ids is not None else -1

                obj = {
                    "id":   track_id,
                    "cls":  cls_name,
                    "cx":   round(cx, 1),
                    "cy":   round(cy, 1),
                    "conf": round(conf, 2),
                }
                if mx is not None:
                    obj["mx"] = mx
                    obj["my"] = my
                detected.append(obj)

                coord_str = f"({cx:.1f}px, {cy:.1f}px)"
                if mx is not None:
                    coord_str += f" = ({mx:.1f}mm, {my:.1f}mm)"
                print(f"[탐지] ID={track_id} | {cls_name} (conf={conf:.2f}) | {coord_str}")

        target = select_target(detected)

        at_target = False
        dist      = None
        if target and target.get("mx") is not None:
            dist      = (target["mx"] ** 2 + target["my"] ** 2) ** 0.5
            at_target = dist < ARRIVE_THRESHOLD_MM

        # ── 상태 머신 ──────────────────────────────────
        if robot_state == RobotState.SEARCHING:
            if target:
                if dist is not None:
                    status_str = f"도달 ({dist:.0f}mm)" if at_target else f"이동중 ({dist:.0f}mm)"
                else:
                    status_str = "픽셀좌표만 (캘리브 필요)"
                print(f"[타겟] {target['cls']} | {status_str}")

            control_wheels(target)

            if at_target:
                send_grip(target)
                robot_state  = RobotState.GRIPPING
                grip_sent_at = time.time()
                print(f"[상태] SEARCHING → GRIPPING (grip 전송: {target['cls']})")
            else:
                send_idle()

        elif robot_state == RobotState.GRIPPING:
            control_wheels(None)   # 바퀴 정지
            elapsed = time.time() - grip_sent_at
            if openrb_gripped:
                openrb_gripped      = False
                storage_phase       = 0
                storage_phase_start = time.time()
                robot_state         = RobotState.GO_TO_STORAGE
                print(f"[상태] GRIPPING → GO_TO_STORAGE ({elapsed:.1f}s)")
            else:
                print(f"[상태] 집는중... ({elapsed:.1f}s)", end="\r")

        elif robot_state == RobotState.GO_TO_STORAGE:
            now     = time.time()
            elapsed = now - storage_phase_start

            if storage_phase == 0:
                # 페이즈 0: 보관함 방향으로 좌회전
                control_wheels(None, override_l=-STORAGE_TURN_SPEED, override_r=STORAGE_TURN_SPEED)
                print(f"[상태] 보관함 방향 회전중... ({elapsed:.1f}s / {STORAGE_TURN_SECS}s)", end="\r")
                if elapsed >= STORAGE_TURN_SECS:
                    storage_phase       = 1
                    storage_phase_start = now
                    print(f"\n[상태] GO_TO_STORAGE: 회전 완료 → 직진 시작")
            else:
                # 페이즈 1: 보관함까지 직진
                control_wheels(None, override_l=STORAGE_DRIVE_SPEED, override_r=STORAGE_DRIVE_SPEED)
                print(f"[상태] 보관함으로 직진중... ({elapsed:.1f}s / {STORAGE_DRIVE_SECS}s)", end="\r")
                if elapsed >= STORAGE_DRIVE_SECS:
                    control_wheels(None)   # 정지
                    send_drop()
                    robot_state = RobotState.DROPPING
                    print(f"\n[상태] GO_TO_STORAGE → DROPPING (drop 전송)")

        elif robot_state == RobotState.DROPPING:
            control_wheels(None)   # 바퀴 정지
            elapsed = time.time() - storage_phase_start
            if openrb_done:
                openrb_done = False
                robot_state = RobotState.SEARCHING
                print(f"[상태] DROPPING → SEARCHING ({elapsed:.1f}s)")
            else:
                print(f"[상태] 내려놓는중...", end="\r")

        # ── 시각화 ──────────────────────────────────────
        annotated_frame = results[0].plot()

        # 타겟 선택 물체에 노란 테두리
        if target and boxes is not None:
            ids = boxes.id
            for i, box in enumerate(boxes):
                tid = int(ids[i]) if ids is not None else -1
                if tid == target["id"]:
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    cv2.rectangle(annotated_frame,
                                  (int(x1) - 4, int(y1) - 4),
                                  (int(x2) + 4, int(y2) + 4),
                                  (0, 255, 255), 3)
                    cv2.putText(annotated_frame, "TARGET",
                                (int(x1), int(y1) - 12),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        h, w = annotated_frame.shape[:2]

        state_colors = {
            RobotState.SEARCHING:     (0, 255, 0),
            RobotState.GRIPPING:      (0, 165, 255),
            RobotState.GO_TO_STORAGE: (255, 165, 0),
            RobotState.DROPPING:      (0, 165, 255),
        }
        state_color = state_colors.get(robot_state, (128, 128, 128))

        overlay = annotated_frame.copy()
        cv2.rectangle(overlay, (0, h - 80), (w, h), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.5, annotated_frame, 0.5, 0, annotated_frame)

        cv2.putText(annotated_frame, f"STATE: {robot_state.value}",
                    (10, h - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, state_color, 2)

        if target:
            if target.get("mx") is not None:
                tgt_text = f"TARGET: {target['cls']}  dist={dist:.0f}mm  conf={target['conf']:.2f}"
            else:
                tgt_text = f"TARGET: {target['cls']}  conf={target['conf']:.2f}"
            cv2.putText(annotated_frame, tgt_text,
                        (10, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        if battery_v is not None:
            batt_color = (0, 255, 0) if battery_v >= 11.5 else (0, 165, 255) if battery_v >= 10.0 else (0, 0, 255)
            batt_text  = f"BAT: {battery_v:.2f}V"
        else:
            batt_color = (128, 128, 128)
            batt_text  = "BAT: --"
        cv2.putText(annotated_frame, batt_text,
                    (w - 150, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, batt_color, 2)

        fps_counter += 1
        elapsed_fps = time.time() - fps_timer
        if elapsed_fps >= 1.0:
            fps_display = fps_counter / elapsed_fps
            fps_counter = 0
            fps_timer   = time.time()
            print(f"[FPS] {fps_display:.1f}")

        cv2.putText(annotated_frame, f"FPS: {fps_display:.1f}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

        if not HEADLESS:
            cv2.imshow(WINDOW_NAME, annotated_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

finally:
    cap.release()
    if ser_esp32  and ser_esp32.is_open:  ser_esp32.close()
    if ser_openrb and ser_openrb.is_open: ser_openrb.close()
    cv2.destroyAllWindows()
    print("카메라 및 시리얼 자원이 안전하게 해제되었습니다.")
