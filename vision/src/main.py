"""
MERO_AI_ROBOT 메인 실행 파일
─────────────────────────────
실행: python vision/src/main.py [--cls d8 apple]

캘리브레이션 유무에 따라 자동 전환:
  - calibration.json 있음 → mm 기반 거리 판단 (정확)
  - calibration.json 없음 → bbox 면적 기반 판단 (캘리브 전 테스트용)

상태 머신:
  SEARCHING      — 타겟 탐지 + 이동
  GRIPPING       — grip 명령 전송 후 gripped 신호 대기 (바퀴 정지)
  GO_TO_STORAGE  — 바퀴로 보관함까지 고정 경로 이동
  DROPPING       — drop 명령 전송 후 done 신호 대기 (바퀴 정지)

시리얼:
  /dev/ttyACM0 → ESP32  (UGV02 바퀴)   {"T":1, "L":speed, "R":speed}
  /dev/ttyACM1 → OpenRB (팔·그리퍼)    {"cmd":"grip"/"drop"/"idle"}

OpenRB 응답:
  {"status":"gripped"} — grip 시퀀스 완료
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
parser = argparse.ArgumentParser()
parser.add_argument('--cls', nargs='+', default=None,
                    help='타겟 클래스 목록 (예: --cls d8 apple). 미지정 시 모든 클래스 대상')
args       = parser.parse_args()
TARGET_CLS = set(args.cls) if args.cls else None

# ── 모델 로드 ────────────────────────────────────────────
BASE_DIR   = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODEL_PATH = os.path.join(BASE_DIR, "model", "best.pt")
# Jetson TensorRT 변환 후: MODEL_PATH = os.path.join(BASE_DIR, "model", "best.engine")
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
    print(f"[캘리브] mm 기반 모드: {MM_PER_PIXEL:.4f} mm/pixel")
else:
    print("[캘리브] calibration.json 없음 → bbox 면적 기반 모드로 실행")


def pixel_to_mm(cx, cy):
    """픽셀 좌표 → 이미지 중심 기준 mm 좌표. 캘리브 없으면 (None, None)."""
    if MM_PER_PIXEL is None:
        return None, None
    w  = FRAME_W or 640
    h  = FRAME_H or 480
    return round((cx - w / 2) * MM_PER_PIXEL, 1), round((cy - h / 2) * MM_PER_PIXEL, 1)


def select_target(objects: list) -> dict | None:
    """--cls 필터 후 bbox area 최대(가장 가까운) 1개 반환."""
    if not objects:
        return None
    if TARGET_CLS:
        objects = [o for o in objects if o['cls'] in TARGET_CLS]
    if not objects:
        return None
    return max(objects, key=lambda o: o['area'])


# ── 시리얼 포트 ──────────────────────────────────────────
ESP32_PORT  = "/dev/ttyACM0"   # CH343 드라이버 → ACM
OPENRB_PORT = "/dev/ttyACM1"
BAUD_RATE   = 115200

# ── 바퀴 제어 파라미터 ───────────────────────────────────
MOVE_SPEED          = 0.3
SLOW_SPEED          = 0.15

# mm 모드 (calibration 있을 때)
ARRIVE_THRESHOLD_MM = 30.0
SLOW_THRESHOLD_MM   = 100.0
MAX_MX              = 200.0
MAX_MY              = 150.0

# 픽셀 모드 (calibration 없을 때) — bbox 면적 기반
AREA_THRESHOLD      = 40000   # 이 면적 이상이면 "도달"로 판단 (w×h px²)
AREA_SLOW_THRESHOLD = 28000   # 이 면적 이상이면 감속 시작
CENTER_MARGIN_PX    = 80      # 픽셀 모드: 화면 중심에서 이 픽셀 이내여야 도달 인정
ALIGN_THRESHOLD     = 0.4     # 이 이상 turn값이면 전진 없이 제자리 회전 우선
TURN_ONLY_SPEED     = 0.2     # 제자리 회전 속도

# 오인식 방지
CONFIRM_FRAMES      = 5       # 연속 N프레임 도달 조건 만족해야 grip 전송

# ── 보관함 이동 고정 경로 파라미터 ──────────────────────
# 경기장: 4m×4m / 출발=우하단 / 보관함=좌하단
# 순서: ① 좌회전(STORAGE_TURN_SECS초) → ② 직진(STORAGE_DRIVE_SECS초)
# TODO: 실측 후 조정
STORAGE_TURN_SECS   = 2.0
STORAGE_DRIVE_SECS  = 3.0
STORAGE_TURN_SPEED  = 0.25
STORAGE_DRIVE_SPEED = 0.3

# ── 상태 머신 ────────────────────────────────────────────
class RobotState(Enum):
    SEARCHING     = "탐색중"
    GRIPPING      = "집는중"
    GO_TO_STORAGE = "보관함이동"
    DROPPING      = "내려놓는중"

robot_state         = RobotState.SEARCHING
grip_sent_at        = 0.0
storage_phase       = 0
storage_phase_start = 0.0
confirm_count       = 0

# ── 시리얼 연결 ──────────────────────────────────────────
def _open_serial(port):
    try:
        s = serial.Serial(port, BAUD_RATE, timeout=1)
        print(f"[시리얼] 연결 성공: {port}")
        return s
    except Exception:
        print(f"[시리얼] 연결 실패: {port} — 해당 보드 없이 실행")
        return None

ser_esp32  = _open_serial(ESP32_PORT)
ser_openrb = _open_serial(OPENRB_PORT)

# ── ESP32 수신 스레드 (배터리 모니터링) ─────────────────
battery_v = None

def _read_esp32_loop():
    global battery_v
    while True:
        if ser_esp32 is None or not ser_esp32.is_open:
            time.sleep(0.5); continue
        try:
            if ser_esp32.in_waiting > 0:
                data = json.loads(ser_esp32.readline().decode("utf-8").strip())
                if data.get("T") == 1001:
                    v_raw = data.get("v") or data.get("V")
                    if v_raw is not None:
                        battery_v = v_raw / 100.0
        except Exception:
            pass
        time.sleep(0.01)

threading.Thread(target=_read_esp32_loop, daemon=True).start()

# ── OpenRB 수신 스레드 (팔 완료 신호) ───────────────────
openrb_gripped = False
openrb_done    = False

def _read_openrb_loop():
    global openrb_gripped, openrb_done
    while True:
        if ser_openrb is None or not ser_openrb.is_open:
            time.sleep(0.5); continue
        try:
            if ser_openrb.in_waiting > 0:
                data = json.loads(ser_openrb.readline().decode("utf-8", errors="ignore").strip())
                if data.get("status") == "gripped":
                    openrb_gripped = True
                    print("\n[OpenRB] 집기 완료")
                elif data.get("status") == "done":
                    openrb_done = True
                    print("\n[OpenRB] 내려놓기 완료")
        except Exception:
            pass
        time.sleep(0.01)

threading.Thread(target=_read_openrb_loop, daemon=True).start()


# ── 바퀴 제어 ────────────────────────────────────────────
def control_wheels(target: dict | None, override_l: float | None = None, override_r: float | None = None):
    """
    override 지정 시 직접 속도 전송 (고정 경로 이동용).
    target 있으면 mm 또는 픽셀 기반 차동 조향.
    target=None이면 정지.
    """
    if ser_esp32 is None or not ser_esp32.is_open:
        return

    if override_l is not None:
        L, R = override_l, override_r

    elif target is None:
        L, R = 0.0, 0.0

    elif target.get("mx") is not None:
        # ── mm 모드 (calibration 있을 때) ──
        mx, my = target["mx"], target["my"]
        dist   = (mx ** 2 + my ** 2) ** 0.5
        if dist >= ARRIVE_THRESHOLD_MM:
            speed = SLOW_SPEED if dist < SLOW_THRESHOLD_MM else MOVE_SPEED
            turn  = max(-1.0, min(1.0, mx / MAX_MX))
            fwd   = max(-1.0, min(1.0, my / MAX_MY))
            L = max(-0.5, min(0.5, speed * (fwd + turn)))
            R = max(-0.5, min(0.5, speed * (fwd - turn)))
        else:
            L, R = 0.0, 0.0

    else:
        # ── 픽셀 모드 (calibration 없을 때) ──
        frame_w = FRAME_W or 640
        turn    = max(-1.0, min(1.0, (target["cx"] - frame_w / 2) / (frame_w / 2)))
        area    = target.get("area", 0)
        if area < AREA_THRESHOLD:
            if abs(turn) > ALIGN_THRESHOLD:
                # 물체가 많이 치우쳐 있으면 제자리 회전 먼저
                L = max(-0.5, min(0.5,  TURN_ONLY_SPEED * turn))
                R = max(-0.5, min(0.5, -TURN_ONLY_SPEED * turn))
            else:
                # 중앙에 가까우면 전진하면서 조향
                speed = SLOW_SPEED if area > AREA_SLOW_THRESHOLD else MOVE_SPEED
                L = max(-0.5, min(0.5, speed * (1.0 + turn)))
                R = max(-0.5, min(0.5, speed * (1.0 - turn)))
        else:
            L, R = 0.0, 0.0

    ser_esp32.write((json.dumps({"T": 1, "L": round(L, 2), "R": round(R, 2)}) + "\n").encode())


def _is_at_target(target: dict) -> bool:
    """도달 여부 판단. mm 모드 → 거리, 픽셀 모드 → area + 중심 정렬."""
    if target.get("mx") is not None:
        dist = (target["mx"] ** 2 + target["my"] ** 2) ** 0.5
        return dist < ARRIVE_THRESHOLD_MM
    frame_w  = FRAME_W or 640
    centered = abs(target["cx"] - frame_w / 2) <= CENTER_MARGIN_PX
    return centered and target.get("area", 0) >= AREA_THRESHOLD


# ── OpenRB 명령 전송 ─────────────────────────────────────
def send_grip(target: dict):
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
    print(f"[카메라] {CAMERA_INDEX}번 실패 → 0번 재시도")
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("[카메라] 사용 가능한 카메라 없음")
        exit()

HEADLESS    = os.environ.get("DISPLAY") is None
WINDOW_NAME = "MERO_AI_ROBOT"
if not HEADLESS:
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE)

mode_str = "mm 모드" if MM_PER_PIXEL else "픽셀 모드 (캘리브 없음)"
cls_str  = ' + '.join(sorted(TARGET_CLS)) if TARGET_CLS else '전체'
print(f"[시작] 타겟: {cls_str} | {mode_str}")
if HEADLESS:
    print("[시작] 헤드리스 모드")

fps_counter = 0
fps_display = 0.0
fps_timer   = time.time()

# ── 메인 루프 ────────────────────────────────────────────
try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("[오류] 프레임 읽기 실패")
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
                area     = (x2 - x1) * (y2 - y1)

                obj = {
                    "id":   track_id,
                    "cls":  cls_name,
                    "cx":   round(cx, 1),
                    "cy":   round(cy, 1),
                    "conf": round(conf, 2),
                    "area": round(area),
                }
                if mx is not None:
                    obj["mx"] = mx
                    obj["my"] = my
                detected.append(obj)

                if mx is not None:
                    coord_str = f"({mx:.1f}mm, {my:.1f}mm) area={area:.0f}"
                else:
                    coord_str = f"({cx:.1f}px, {cy:.1f}px) area={area:.0f}"
                print(f"[탐지] ID={track_id} | {cls_name} conf={conf:.2f} | {coord_str}")

        target    = select_target(detected)
        at_target = _is_at_target(target) if target else False

        # ── 상태 머신 ──────────────────────────────────
        if robot_state == RobotState.SEARCHING:
            if target:
                if target.get("mx") is not None:
                    dist = (target["mx"] ** 2 + target["my"] ** 2) ** 0.5
                    info = f"dist={dist:.0f}mm"
                else:
                    info = f"area={target['area']}"
                status = "도달" if at_target else f"이동중 ({info})"
                print(f"[타겟] {target['cls']} | {status}")

            control_wheels(target)

            if at_target:
                confirm_count += 1
                print(f"[타겟] 도달 확인 {confirm_count}/{CONFIRM_FRAMES}", end="\r")
                if confirm_count >= CONFIRM_FRAMES:
                    confirm_count = 0
                    send_grip(target)
                    robot_state  = RobotState.GRIPPING
                    grip_sent_at = time.time()
                    print(f"\n[상태] SEARCHING → GRIPPING (grip: {target['cls']})")
            else:
                confirm_count = 0
                send_idle()

        elif robot_state == RobotState.GRIPPING:
            control_wheels(None)
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
                # 좌회전
                control_wheels(None, override_l=-STORAGE_TURN_SPEED, override_r=STORAGE_TURN_SPEED)
                print(f"[상태] 회전중... ({elapsed:.1f}s / {STORAGE_TURN_SECS}s)", end="\r")
                if elapsed >= STORAGE_TURN_SECS:
                    storage_phase       = 1
                    storage_phase_start = now
                    print(f"\n[상태] 회전 완료 → 직진 시작")
            else:
                # 직진
                control_wheels(None, override_l=STORAGE_DRIVE_SPEED, override_r=STORAGE_DRIVE_SPEED)
                print(f"[상태] 직진중... ({elapsed:.1f}s / {STORAGE_DRIVE_SECS}s)", end="\r")
                if elapsed >= STORAGE_DRIVE_SECS:
                    control_wheels(None)
                    send_drop()
                    robot_state = RobotState.DROPPING
                    print(f"\n[상태] GO_TO_STORAGE → DROPPING (drop 전송)")

        elif robot_state == RobotState.DROPPING:
            control_wheels(None)
            elapsed = time.time() - storage_phase_start
            if openrb_done:
                openrb_done   = False
                confirm_count = 0
                robot_state   = RobotState.SEARCHING
                print(f"[상태] DROPPING → SEARCHING ({elapsed:.1f}s)")
            else:
                print(f"[상태] 내려놓는중...", end="\r")

        # ── 시각화 ──────────────────────────────────────
        annotated_frame = results[0].plot()

        # 타겟 노란 테두리
        if target and boxes is not None:
            ids = boxes.id
            for i, box in enumerate(boxes):
                if (int(ids[i]) if ids is not None else -1) == target["id"]:
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    cv2.rectangle(annotated_frame,
                                  (int(x1) - 4, int(y1) - 4),
                                  (int(x2) + 4, int(y2) + 4),
                                  (0, 255, 255), 3)
                    cv2.putText(annotated_frame, "TARGET",
                                (int(x1), int(y1) - 12),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        h, w = annotated_frame.shape[:2]

        # 하단 상태 바
        state_colors = {
            RobotState.SEARCHING:     (0, 255, 0),
            RobotState.GRIPPING:      (0, 165, 255),
            RobotState.GO_TO_STORAGE: (255, 165, 0),
            RobotState.DROPPING:      (0, 165, 255),
        }
        overlay = annotated_frame.copy()
        cv2.rectangle(overlay, (0, h - 80), (w, h), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.5, annotated_frame, 0.5, 0, annotated_frame)

        cv2.putText(annotated_frame, f"STATE: {robot_state.value}",
                    (10, h - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    state_colors.get(robot_state, (255, 255, 255)), 2)

        if target:
            if target.get("mx") is not None:
                d = (target["mx"] ** 2 + target["my"] ** 2) ** 0.5
                tgt_text = f"TARGET: {target['cls']}  dist={d:.0f}mm  conf={target['conf']:.2f}"
            else:
                tgt_text = f"TARGET: {target['cls']}  area={target['area']}  conf={target['conf']:.2f}"
            cv2.putText(annotated_frame, tgt_text,
                        (10, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        # 배터리
        if battery_v is not None:
            bc = (0, 255, 0) if battery_v >= 11.5 else (0, 165, 255) if battery_v >= 10.0 else (0, 0, 255)
            cv2.putText(annotated_frame, f"BAT: {battery_v:.2f}V",
                        (w - 150, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, bc, 2)

        # FPS
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
    print("자원 해제 완료.")
