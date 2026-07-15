"""
MERO_AI_ROBOT 메인 실행 파일
─────────────────────────────
실행: python vision/src/main.py [--cls d8 apple]

캘리브레이션 유무에 따라 자동 전환:
  - calibration.json 있음 → mm 기반 거리 판단 (정확)
  - calibration.json 없음 → bbox 면적 기반 판단 (캘리브 전 테스트용)

상태 머신:
  SEARCHING      — 타겟 탐지 + 이동, 도달 시 grip 전송
  GRIPPING       — grip 전송 후 gripped 신호 대기 (집기+팔올림+투하+팔내림 완료)
                   → gripped 수신 시 SEARCHING 복귀 (반복 수집)
                   → 모든 타겟 수집 완료 시 GO_TO_STORAGE
  GO_TO_STORAGE  — 바퀴로 보관함까지 고정 경로 이동 → dump 전송
  DROPPING       — dump 전송 후 dumped 신호 대기 (컨테이너 열어 쏟기 완료)

시리얼:
  /dev/ttyACM0 → ESP32  (UGV02 바퀴)   {"T":1, "L":speed, "R":speed}
  /dev/ttyACM1 → OpenRB (팔·그리퍼)    {"cmd":"grip"/"dump"/"idle"}

OpenRB 응답:
  {"status":"gripped"}     — 집기+컨테이너 투하 완료 → SEARCHING 복귀
  {"status":"grip_failed"} — 집기 실패 → SEARCHING 복귀
  {"status":"dumped"}      — 컨테이너 열기 완료 → SEARCHING 복귀
"""

import argparse
import cv2
import glob
import os
import json
import time
import threading
import serial
from enum import Enum
from http.server import BaseHTTPRequestHandler, HTTPServer
from ultralytics import YOLO

# ── 인수 파싱 ───────────────────────────────────────────
parser = argparse.ArgumentParser()
parser.add_argument('--cls', nargs='+', default=None,
                    help='타겟 클래스 목록 (예: --cls d8 apple). 미지정 시 모든 클래스 대상')
parser.add_argument('--timer', action='store_true',
                    help='3분 경기 타이머 표시')
parser.add_argument('--test', action='store_true',
                    help='테스트 모드: 집으면 1초 직진 후 바로 drop')
args       = parser.parse_args()
TARGET_CLS    = set(args.cls) if args.cls else None
TEST_MODE     = args.test
SHAPE_CLASSES = {'d6', 'd8', 'd12', 'd20'}
FRUIT_CLASSES = {'apple', 'banana', 'orange', 'pineapple'}

def max_count(cls: str) -> int:
    return 4 if cls in SHAPE_CLASSES else 3

# ── 모델 로드 ────────────────────────────────────────────
BASE_DIR        = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODEL_PATH      = os.path.join(BASE_DIR, "model", "best.pt")
FLAG_MODEL_PATH = os.path.join(BASE_DIR, "model", "flag.pt")
# Jetson TensorRT 변환 후: MODEL_PATH = os.path.join(BASE_DIR, "model", "best.engine")
model      = YOLO(MODEL_PATH)
flag_model = YOLO(FLAG_MODEL_PATH)

# ── 카메라 인덱스 ────────────────────────────────────────
CAMERA_INDEX_OBJ  = 0   # 물체 카메라 (1사분면 USB-A, USB 2.0)  ← 실제 확인 필요
CAMERA_INDEX_FLAG = 2   # 태극기 카메라 (4사분면 USB-A, USB 3.0) ← 실제 확인 필요

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


CONF_THRESHOLD_SHAPE = 0.25  # shape 클래스 confidence 임계값
CONF_THRESHOLD_FRUIT = 0.6   # 과일 클래스 — 오픽업 패널티 40점이라 높게 설정

def select_target(objects: list) -> dict | None:
    """--cls 필터 + 목표 개수 미달 + 클래스별 confidence 임계값 통과한 것 중 area 최대 반환."""
    if not objects:
        return None
    filtered = []
    for o in objects:
        if TARGET_CLS and o['cls'] not in TARGET_CLS:
            continue
        if pickup_counts.get(o['cls'], 0) >= max_count(o['cls']):
            continue
        threshold = CONF_THRESHOLD_FRUIT if o['cls'] in FRUIT_CLASSES else CONF_THRESHOLD_SHAPE
        if o['conf'] >= threshold:
            filtered.append(o)
    if not filtered:
        return None
    return max(filtered, key=lambda o: o['area'])


# ── 시리얼 포트 자동 감지 ────────────────────────────────
def _find_port(keywords, fallback):
    """USB ID 키워드로 포트 자동 탐지 (Linux /dev/serial/by-id/ 기반)."""
    for path in glob.glob("/dev/serial/by-id/*"):
        name = path.lower()
        if any(k.lower() in name for k in keywords):
            detected = os.path.realpath(path)
            print(f"[포트] {keywords[0]} → {detected}")
            return detected
    print(f"[포트] {keywords[0]} 자동 탐지 실패 → 기본값 {fallback}")
    return fallback

ESP32_PORT  = _find_port(["1a86", "ch343", "ch34"], "/dev/ttyACM0")   # CH343 드라이버
OPENRB_PORT = _find_port(["openrb", "robotis", "2ecc"], "/dev/ttyACM1")
BAUD_RATE   = 115200

# ── 바퀴 제어 파라미터 ───────────────────────────────────
MOVE_SPEED          = 0.2
SLOW_SPEED          = 0.1

# mm 모드 (calibration 있을 때)
ARRIVE_THRESHOLD_MM = 30.0
SLOW_THRESHOLD_MM   = 100.0
MAX_MX              = 200.0
MAX_MY              = 150.0

# 픽셀 모드 (calibration 없을 때) — bbox 면적 기반
AREA_THRESHOLD      = 28000   # 이 면적 이상이면 "도달"로 판단 (w×h px²)
AREA_SLOW_THRESHOLD = 20000   # 이 면적 이상이면 감속 시작
AREA_ROTATE_THRESHOLD = 15000 # 이 이하일 때만 제자리 회전 정렬
CENTER_MARGIN_PX    = 120     # 픽셀 모드: 가로 중심에서 이 픽셀 이내여야 도달 인정
CENTER_MARGIN_Y_PX  = 100     # 픽셀 모드: 세로 중심에서 이 픽셀 이내여야 도달 인정
CENTER_OFFSET_Y_PX  = 20      # 세로 중심 오프셋 (양수=아래)
ALIGN_THRESHOLD     = 0.25    # 이 이상 turn값이면 전진 없이 제자리 회전 우선
TURN_ONLY_SPEED     = 0.2     # 제자리 회전 속도

# 오인식 방지
CONFIRM_FRAMES      = 3       # 연속 N프레임 도달 조건 만족해야 grip 전송

# 탐색 회전
SEARCH_ROTATE_SPEED = 0.2     # 타겟 없을 때 제자리 회전 속도

# 타임아웃
GRIP_TIMEOUT_SECS    = 15.0   # grip 전송 후 gripped 신호 최대 대기
DROP_TIMEOUT_SECS    = 15.0   # drop 전송 후 done 신호 최대 대기
STORAGE_TIMEOUT_SECS = 15.0   # GO_TO_STORAGE 전체 최대 시간

# 경기 타이머
MATCH_DURATION_SECS = 180.0
match_start_time    = time.time() if args.timer else None

# ── 태극기 네비게이션 파라미터 ──────────────────────────
FLAG_CONF_THRESHOLD      = 0.5
FLAG_CENTER_MARGIN_PX    = 100    # 가로 정렬 허용 범위 (px)
FLAG_AREA_THRESHOLD      = 60000  # 도달 판단 면적 (px²)  ⚠️ 임의값 — 실측 필요
FLAG_AREA_SLOW_THRESHOLD = 30000  # 감속 시작 면적 (px²)  ⚠️ 임의값 — 실측 필요
FLAG_SEARCH_SPEED        = 0.15   # 탐색 회전 속도
FLAG_APPROACH_SPEED      = 0.2    # 후진 접근 속도
FLAG_APPROACH_SLOW       = 0.1    # 감속 후진 속도

# ── 상태 머신 ────────────────────────────────────────────
class RobotState(Enum):
    SEARCHING     = "SEARCHING"
    GRIPPING      = "GRIPPING"
    GO_TO_STORAGE = "GO_TO_STORAGE"
    DROPPING      = "DROPPING"

robot_state         = RobotState.SEARCHING
grip_sent_at        = 0.0
drop_sent_at        = 0.0
_frame_fail_count   = 0
storage_phase       = 0   # 0=탐색회전, 1=후진접근
storage_phase_start = 0.0
storage_enter_time  = 0.0
confirm_count       = 0
last_target_id      = -1
gripped_cls         = None
pickup_counts       = {}   # {cls: 바스켓에 넣은 개수}

# IMU
imu_yaw       = None
_last_imu_req = 0.0

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
    global battery_v, imu_yaw, _last_imu_req
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
                elif data.get("T") == 126 and "y" in data:
                    imu_yaw = float(data["y"])
        except Exception:
            pass
        time.sleep(0.01)

threading.Thread(target=_read_esp32_loop, daemon=True).start()

# ── OpenRB 수신 스레드 (팔 완료 신호) ───────────────────
openrb_gripped     = False
openrb_dumped      = False
openrb_grip_failed = False

def _read_openrb_loop():
    global openrb_gripped, openrb_dumped, openrb_grip_failed
    while True:
        if ser_openrb is None or not ser_openrb.is_open:
            time.sleep(0.5); continue
        try:
            if ser_openrb.in_waiting > 0:
                data = json.loads(ser_openrb.readline().decode("utf-8", errors="ignore").strip())
                if data.get("status") == "gripped":
                    openrb_gripped = True
                    print("\n[OpenRB] 집기+투하 완료")
                elif data.get("status") == "dumped":
                    openrb_dumped = True
                    print("\n[OpenRB] 컨테이너 쏟기 완료")
                elif data.get("status") == "grip_failed":
                    openrb_grip_failed = True
                    print("\n[OpenRB] 집기 실패 (전류 미달)")
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
        frame_w  = FRAME_W or 640
        turn     = max(-1.0, min(1.0, (target["cx"] - frame_w / 2) / (frame_w / 2)))
        area     = target.get("area", 0)

        if abs(turn) > ALIGN_THRESHOLD and area < AREA_ROTATE_THRESHOLD:
            # 멀리 있을 때만 제자리 회전 정렬
            L = max(-0.5, min(0.5,  TURN_ONLY_SPEED * turn))
            R = max(-0.5, min(0.5, -TURN_ONLY_SPEED * turn))
        else:
            # 중앙에 가까우면 전진하면서 조향
            speed = SLOW_SPEED if area > AREA_SLOW_THRESHOLD else MOVE_SPEED
            L = max(-0.5, min(0.5, speed * (1.0 + turn)))
            R = max(-0.5, min(0.5, speed * (1.0 - turn)))

    ser_esp32.write((json.dumps({"T": 1, "L": round(L, 2), "R": round(R, 2)}) + "\n").encode())


def _is_at_target(target: dict) -> bool:
    """도달 여부 판단. mm 모드 → 거리, 픽셀 모드 → area + 중심 정렬."""
    if target.get("mx") is not None:
        dist = (target["mx"] ** 2 + target["my"] ** 2) ** 0.5
        return dist < ARRIVE_THRESHOLD_MM
    frame_w  = FRAME_W or 640
    frame_h  = FRAME_H or 480
    cx_ok = abs(target["cx"] - frame_w / 2) <= CENTER_MARGIN_PX
    cy_ok = abs(target["cy"] - (frame_h / 2 + CENTER_OFFSET_Y_PX)) <= CENTER_MARGIN_Y_PX
    return cx_ok and cy_ok and target.get("area", 0) >= AREA_THRESHOLD


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

def send_dump():
    if ser_openrb is None or not ser_openrb.is_open:
        return
    ser_openrb.write((json.dumps({"cmd": "dump"}) + "\n").encode())

_last_idle_t = 0.0
def send_idle():
    global _last_idle_t
    if ser_openrb is None or not ser_openrb.is_open:
        return
    now = time.time()
    if now - _last_idle_t >= 1.0:
        ser_openrb.write((json.dumps({"cmd": "idle"}) + "\n").encode())
        _last_idle_t = now


# ── 카메라 초기화 ────────────────────────────────────────
def _init_camera(index, name):
    cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        print(f"[카메라] {name} ({index}번) 열기 실패")
        return None
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('Y', 'U', 'Y', '2'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1200)
    cap.set(cv2.CAP_PROP_FPS, 50)
    ret, f = cap.read()
    if ret:
        h, w = f.shape[:2]
        print(f"[카메라] {name} ({index}번) 준비: {w}×{h}")
    return cap

cap  = _init_camera(CAMERA_INDEX_OBJ,  "물체캠")   # 1사분면, 아래 대각
cap2 = _init_camera(CAMERA_INDEX_FLAG, "태극기캠") # 4사분면, 뒤쪽

if cap is None:
    print("[오류] 물체 카메라 없음 — 종료"); exit()

# 실제 카메라 해상도로 FRAME_W/H 보정 (calibration.json 없을 때)
if FRAME_W is None:
    _ret, _f = cap.read()
    if _ret:
        FRAME_H, FRAME_W = _f.shape[:2]

FRAME_W2 = FRAME_H2 = None
if cap2 is not None:
    _ret2, _f2 = cap2.read()
    if _ret2:
        FRAME_H2, FRAME_W2 = _f2.shape[:2]

HEADLESS    = True  # X11 imshow 비활성화 (SSH+WiFi 병목 방지)
WINDOW_NAME = "MERO_AI_ROBOT"
if not HEADLESS:
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE)

mode_str = "mm 모드" if MM_PER_PIXEL else "픽셀 모드 (캘리브 없음)"
cls_str  = ' + '.join(sorted(TARGET_CLS)) if TARGET_CLS else '전체'
print(f"[시작] 타겟: {cls_str} | {mode_str}")
if HEADLESS:
    print("[시작] 헤드리스 모드")

# ── MJPEG 스트리밍 서버 (브라우저에서 http://jetson_ip:8080 접속) ──
_stream_frame = None
_stream_lock  = threading.Lock()

class _MJPEGHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        self.send_response(200)
        self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
        self.end_headers()
        try:
            while True:
                with _stream_lock:
                    f = _stream_frame
                if f is None:
                    time.sleep(0.03)
                    continue
                _, jpg = cv2.imencode('.jpg', f, [cv2.IMWRITE_JPEG_QUALITY, 70])
                self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpg.tobytes() + b'\r\n')
        except Exception:
            pass
    def log_message(self, *_):
        pass

threading.Thread(
    target=lambda: HTTPServer(('0.0.0.0', 8080), _MJPEGHandler).serve_forever(),
    daemon=True
).start()
print("[스트림] http://172.20.10.5:8080 에서 카메라 확인 가능")

# ── 카메라 캡처 스레드 (cap.read 블로킹을 메인 루프에서 분리) ──

fps_counter = 0
fps_display = 0.0
fps_timer   = time.time()
_last_print_t = 0.0  # 탐지/타겟 로그 출력 주기 제어

# ── 메인 루프 ────────────────────────────────────────────
try:
    while True:
        # GO_TO_STORAGE 중에는 cap2+flag_model 사용 (GO_TO_STORAGE 블록 내부에서 처리)
        # 그 외 상태는 cap1+model로 물체 탐지
        if robot_state == RobotState.GO_TO_STORAGE:
            cap.read()  # 버퍼 비우기만
            results  = None
            boxes    = None
            detected = []
            target   = None
            at_target = False
            annotated_frame = frame if 'frame' in dir() else None
        else:
            ret, frame = cap.read()
            if not ret:
                _frame_fail_count += 1
                if _frame_fail_count >= 10:
                    print("[오류] 프레임 읽기 연속 10회 실패 — 종료")
                    break
                continue
            _frame_fail_count = 0

            results  = model.track(frame, persist=True, conf=0.25, verbose=False, device="cuda", tracker="bytetrack.yaml")
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
                if time.time() - _last_print_t >= 0.5:
                    print(f"[탐지] ID={track_id} | {cls_name} conf={conf:.2f} | {coord_str}")

        target    = select_target(detected)
        at_target = _is_at_target(target) if target else False

        # ── IMU 주기 요청 ────────────────────────────────
        _now_loop = time.time()
        if _now_loop - _last_imu_req >= 0.15:
            if ser_esp32 and ser_esp32.is_open:
                ser_esp32.write((json.dumps({"T": 126}) + "\n").encode())
            _last_imu_req = _now_loop

        # ── 상태 머신 ──────────────────────────────────
        if robot_state == RobotState.SEARCHING:
            if target:
                # 클래스가 바뀔 때만 confirm_count 리셋 (ID 변경은 무시)
                if target["cls"] != getattr(select_target, "_last_cls", None):
                    confirm_count  = 0
                    select_target._last_cls = target["cls"]

                if target.get("mx") is not None:
                    dist = (target["mx"] ** 2 + target["my"] ** 2) ** 0.5
                    info = f"dist={dist:.0f}mm"
                else:
                    info = f"area={target['area']}"
                frame_w = FRAME_W or 640
                frame_h = FRAME_H or 480
                cx_off  = abs(target["cx"] - frame_w / 2)
                cy_off  = abs(target["cy"] - (frame_h / 2 + CENTER_OFFSET_Y_PX))
                status = f"도달" if at_target else f"이동중 ({info}) cx={cx_off:.0f} cy={cy_off:.0f}"
                if time.time() - _last_print_t >= 0.5:
                    print(f"[타겟] {target['cls']} | {status}")
                    _last_print_t = time.time()

            if at_target:
                control_wheels(None)  # 도달 시 정지 후 confirm
            elif target:
                control_wheels(target)
            else:
                control_wheels(None, override_l=-SEARCH_ROTATE_SPEED, override_r=SEARCH_ROTATE_SPEED)

            all_done = TARGET_CLS and all(pickup_counts.get(c, 0) >= max_count(c) for c in TARGET_CLS)
            if at_target:
                confirm_count += 1
                print(f"[타겟] 도달 확인 {confirm_count}/{CONFIRM_FRAMES}", end="\r")
                if confirm_count >= CONFIRM_FRAMES:
                    confirm_count  = 0
                    last_target_id = -1
                    gripped_cls    = target["cls"]
                    openrb_gripped     = False
                    openrb_dumped      = False
                    openrb_grip_failed = False
                    send_grip(target)
                    robot_state  = RobotState.GRIPPING
                    grip_sent_at = time.time()
                    print(f"\n[상태] SEARCHING → GRIPPING (grip: {target['cls']})")
            elif all_done:
                control_wheels(None)
                storage_phase       = 0
                storage_phase_start = time.time()
                storage_enter_time  = time.time()
                robot_state         = RobotState.GO_TO_STORAGE
                print(f"\n[상태] SEARCHING → GO_TO_STORAGE (전체 수집 완료)")
            else:
                confirm_count = 0
                send_idle()

        elif robot_state == RobotState.GRIPPING:
            control_wheels(None)
            elapsed = time.time() - grip_sent_at
            if openrb_gripped:
                openrb_gripped      = False
                openrb_dumped       = False
                openrb_grip_failed  = False
                if gripped_cls:
                    pickup_counts[gripped_cls] = pickup_counts.get(gripped_cls, 0) + 1
                    print(f"[스코어] {gripped_cls}: {pickup_counts[gripped_cls]}/{max_count(gripped_cls)}")
                    gripped_cls = None
                confirm_count  = 0
                last_target_id = -1
                robot_state    = RobotState.SEARCHING
                print(f"[상태] GRIPPING → SEARCHING ({elapsed:.1f}s)")
            elif openrb_grip_failed:
                openrb_grip_failed = False
                openrb_gripped     = False
                openrb_dumped      = False
                confirm_count      = 0
                last_target_id     = -1
                robot_state        = RobotState.SEARCHING
                print(f"\n[상태] GRIPPING → SEARCHING (집기 실패)")
            elif elapsed > GRIP_TIMEOUT_SECS:
                print(f"\n[경고] grip 타임아웃 → SEARCHING 복귀")
                confirm_count  = 0
                last_target_id = -1
                robot_state    = RobotState.SEARCHING
            else:
                print(f"[상태] 집어서 컨테이너 투하중... ({elapsed:.1f}s)", end="\r")

        elif robot_state == RobotState.GO_TO_STORAGE:
            now           = time.time()
            total_elapsed = now - storage_enter_time

            # 타임아웃
            if total_elapsed > STORAGE_TIMEOUT_SECS:
                control_wheels(None)
                print(f"\n[경고] GO_TO_STORAGE 타임아웃 → SEARCHING 복귀")
                robot_state = RobotState.SEARCHING

            # 태극기 카메라로 플래그 감지
            elif cap2 is None:
                print("[경고] 태극기 카메라 없음 — SEARCHING 복귀")
                robot_state = RobotState.SEARCHING

            else:
                ret2, frame2 = cap2.read()
                flag_detected = None
                if ret2:
                    flag_res  = flag_model(frame2, conf=FLAG_CONF_THRESHOLD, verbose=False, device="cuda")
                    flag_boxes = flag_res[0].boxes
                    if flag_boxes is not None and len(flag_boxes) > 0:
                        best = max(flag_boxes, key=lambda b: float(b.conf[0]))
                        x1, y1, x2, y2 = best.xyxy[0].tolist()
                        flag_detected = {
                            "cx":   (x1 + x2) / 2,
                            "area": (x2 - x1) * (y2 - y1),
                        }

                fw2 = FRAME_W2 or 640

                if storage_phase == 0:
                    # 태극기 탐색 — 제자리 회전
                    if flag_detected:
                        storage_phase       = 1
                        storage_phase_start = now
                        print(f"\n[상태] 태극기 발견 → 후진 접근 시작")
                    else:
                        control_wheels(None, override_l=FLAG_SEARCH_SPEED, override_r=-FLAG_SEARCH_SPEED)
                        print(f"[상태] 태극기 탐색 회전중... ({total_elapsed:.1f}s)", end="\r")

                elif storage_phase == 1:
                    # 태극기 후진 접근
                    if not flag_detected:
                        # 태극기 놓침 → 탐색으로 복귀
                        storage_phase       = 0
                        storage_phase_start = now
                        print(f"\n[상태] 태극기 놓침 → 탐색 복귀")
                    elif flag_detected["area"] >= FLAG_AREA_THRESHOLD:
                        # 도달
                        control_wheels(None)
                        openrb_dumped = False
                        send_dump()
                        drop_sent_at  = time.time()
                        robot_state   = RobotState.DROPPING
                        print(f"\n[상태] 태극기 도달 → dump 전송")
                    else:
                        # 후진하면서 정렬
                        # 후방 카메라: flag.cx 기준 조향 (좌우 반전 없음 — 후진이므로 동일 부호)
                        turn  = (flag_detected["cx"] - fw2 / 2) / (fw2 / 2)
                        speed = FLAG_APPROACH_SLOW if flag_detected["area"] > FLAG_AREA_SLOW_THRESHOLD else FLAG_APPROACH_SPEED
                        L = -(speed + turn * 0.3)
                        R = -(speed - turn * 0.3)
                        control_wheels(None, override_l=L, override_r=R)
                        print(f"[상태] 후진 접근중... area={flag_detected['area']:.0f}", end="\r")

        elif robot_state == RobotState.DROPPING:
            control_wheels(None)
            elapsed = time.time() - drop_sent_at
            if openrb_dumped:
                openrb_dumped  = False
                confirm_count  = 0
                last_target_id = -1
                pickup_counts.clear()
                robot_state = RobotState.SEARCHING
                print(f"[상태] DROPPING → SEARCHING ({elapsed:.1f}s)")
            elif elapsed > DROP_TIMEOUT_SECS:
                print(f"\n[경고] dump 타임아웃 → SEARCHING 복귀")
                confirm_count  = 0
                last_target_id = -1
                robot_state    = RobotState.SEARCHING
            else:
                print(f"[상태] 컨테이너 쏟는중... ({elapsed:.1f}s)", end="\r")

        # ── 시각화 ──────────────────────────────────────
        if results is not None:
            annotated_frame = results[0].plot()
        elif frame is not None:
            annotated_frame = frame.copy()
        else:
            continue

        # 중앙 정렬 가이드라인 (OK 박스)
        _fw = FRAME_W or 640
        _fh = FRAME_H or 480
        _cx = _fw // 2
        _cy = _fh // 2 + CENTER_OFFSET_Y_PX
        _box_color = (0, 255, 0) if at_target else (0, 200, 255)
        cv2.rectangle(annotated_frame,
                      (_cx - CENTER_MARGIN_PX, _cy - CENTER_MARGIN_Y_PX),
                      (_cx + CENTER_MARGIN_PX, _cy + CENTER_MARGIN_Y_PX),
                      _box_color, 1)
        cv2.line(annotated_frame, (_cx, _cy - 8), (_cx, _cy + 8), _box_color, 1)
        cv2.line(annotated_frame, (_cx - 8, _cy), (_cx + 8, _cy), _box_color, 1)

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

        # 스코어 표시
        if TARGET_CLS:
            score_parts = [f"{c}:{pickup_counts.get(c,0)}/{max_count(c)}" for c in sorted(TARGET_CLS)]
            all_done = all(pickup_counts.get(c, 0) >= max_count(c) for c in TARGET_CLS)
            score_text = "  ".join(score_parts)
            score_color = (0, 255, 255) if all_done else (255, 255, 255)
            if all_done:
                score_text += "  DONE!"
            cv2.putText(annotated_frame, score_text,
                        (w // 2 - 80, h - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.65, score_color, 2)

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

        # 경기 타이머
        if match_start_time is not None:
            remaining = max(0.0, MATCH_DURATION_SECS - (time.time() - match_start_time))
            mins = int(remaining // 60)
            secs = int(remaining % 60)
            tc = (0, 0, 255) if remaining < 30 else (0, 165, 255) if remaining < 60 else (0, 255, 255)
            cv2.putText(annotated_frame, f"{mins}:{secs:02d}",
                        (w // 2 - 25, 35), cv2.FONT_HERSHEY_SIMPLEX, 1.0, tc, 2)

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

        with _stream_lock:
            _stream_frame = annotated_frame.copy()

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
