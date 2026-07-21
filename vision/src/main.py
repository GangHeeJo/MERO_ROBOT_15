"""
MERO_AI_ROBOT 메인 실행 파일
─────────────────────────────
실행: python vision/src/main.py [--cls d8 apple]

캘리브레이션 유무에 따라 자동 전환:
  - calibration.json 있음 → mm 기반 거리 판단 (정확)
  - calibration.json 없음 → bbox 면적 기반 판단 (캘리브 전 테스트용)

카메라 1대(전면) + 통합 모델(best.pt: 도형+과일+flag) 구조.
select_target()이 flag 클래스는 항상 제외하므로 SEARCHING/GRIPPING 중엔 절대
flag를 집으러 가지 않고, GO_TO_STORAGE에서만 같은 탐지 결과 중 flag를 걸러 씀.

상태 머신:
  SEARCHING      — flag 제외 물체 탐지, 보이면 거리 상관없이 바로 정밀 정렬
                   (전진/후진→회전) 진입 → 정렬 끝나면 직진 접근 후 grip 전송
  GRIPPING       — grip 전송 후 gripped 신호 대기 (집기+팔올림+투하+팔내림 완료)
                   → gripped 수신 시 SEARCHING 복귀 (반복 수집)
                   → 모든 타겟 수집 완료 시 GO_TO_STORAGE
  GO_TO_STORAGE  — flag 탐지해서 전진 접근 → dump 전송
  DROPPING       — dump 전송 후 dumped 신호 대기 (컨테이너 열어 쏟기 완료)

시리얼:
  /dev/ttyACM0 → ESP32  (UGV02 바퀴)   {"T":1, "L":speed, "R":speed}
  /dev/ttyACM1 → OpenRB (팔·그리퍼)    {"cmd":"grip"/"dump"/"idle"/"gripper_open"/"gripper_close"}

OpenRB 응답:
  {"status":"gripped"}        — 집기+컨테이너 투하+그리퍼 재닫힘 완료 → SEARCHING 복귀
  {"status":"grip_failed"}    — 집기 실패 → SEARCHING 복귀
  {"status":"dumped"}         — 컨테이너 열기 완료 → SEARCHING 복귀
  {"status":"gripper_opened"} — 접근 전 그리퍼 미리 열기 완료
  {"status":"gripper_closed"} — 접근 취소 후 그리퍼 대기 상태로 닫힘 완료

그리퍼 안전 정책:
  IDLE 기본값은 "닫힘" (엉뚱한 물체가 벌어진 집게로 들어와 잡히는 것 방지).
  타겟 발견 시(정밀 정렬 진입) gripper_open 전송 → 실제 접근 시작.
  정밀 정렬 중 타겟을 놓치면 gripper_close 전송 후 재탐색.
  grip 성공적으로 전송되면 이후 재닫힘은 OpenRB(robot.ino LIFTING 단계)가 자체 처리.
"""

import argparse
import cv2
import glob
import os
import json
import socket
import time
import threading
import serial
from enum import Enum
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from ultralytics import YOLO

# ── 인수 파싱 ───────────────────────────────────────────
parser = argparse.ArgumentParser()
parser.add_argument('--cls', nargs='+', default=None,
                    help='타겟 클래스 목록 (예: --cls d8 apple). 미지정 시 카메라/모델 로드 후 콘솔에서 2개 직접 입력받음')
parser.add_argument('--timer', action='store_true',
                    help='3분 경기 타이머 표시')
parser.add_argument('--test', action='store_true',
                    help='테스트 모드: 집으면 1초 직진 후 바로 drop')
parser.add_argument('--align-only', action='store_true',
                    help='정렬 순서 비교용(참고): 기본 순서(전진/후진→회전)와 반대로 회전→전진/후진 먼저 시도')
parser.add_argument('--no-wheels', action='store_true',
                    help='바퀴 명령을 ESP32로 보내지 않음 (탐지/그리퍼만 테스트할 때)')
args       = parser.parse_args()
TARGET_CLS    = set(args.cls) if args.cls else None
TEST_MODE     = args.test
ALIGN_ONLY    = args.align_only
NO_WHEELS     = args.no_wheels
SHAPE_CLASSES = {'d6', 'd8', 'd12', 'd20'}
FRUIT_CLASSES = {'apple', 'banana', 'orange', 'pineapple'}

# ── 모델 로드 (통합 모델 하나 — 도형+과일+flag 전부 포함) ──
BASE_DIR   = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODEL_PATH = os.path.join(BASE_DIR, "model", "best.pt")
# Jetson TensorRT 변환 후: MODEL_PATH = os.path.join(BASE_DIR, "model", "best.engine")
model = YOLO(MODEL_PATH)
FLAG_CLASS_AVAILABLE = 'flag' in model.names.values()
if not FLAG_CLASS_AVAILABLE:
    print("[모델] best.pt에 flag 클래스 없음 — GO_TO_STORAGE 태극기 감지 비활성화")

# ── 카메라 인덱스 자동 감지 (카메라 1대만 사용) ──────────
def _find_camera_index(keywords, fallback):
    """장치 이름 키워드로 카메라 인덱스 자동 탐지 (/sys/class/video4linux 기반).
    카메라 하나가 capture/metadata 등 여러 /dev/videoN 노드를 가질 수 있어
    매칭된 것 중 가장 작은 번호(=capture 노드)를 사용한다."""
    matches = []
    for path in glob.glob("/sys/class/video4linux/video*/name"):
        try:
            with open(path) as f:
                name = f.read().strip().lower()
        except OSError:
            continue
        if any(k.lower() in name for k in keywords):
            idx = int(path.split("/")[-2].replace("video", ""))
            matches.append(idx)
    if matches:
        idx = min(matches)
        print(f"[카메라] {keywords[0]} → /dev/video{idx}")
        return idx
    print(f"[카메라] {keywords[0]} 자동 탐지 실패 → 기본값 {fallback}")
    return fallback

CAMERA_INDEX_OBJ = _find_camera_index(["arducam"], 2)  # 물체+태극기 겸용 카메라 (전면)

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


CONF_THRESHOLD_SHAPE   = 0.25  # shape 클래스 confidence 임계값
CONF_THRESHOLD_FRUIT   = 0.6   # 과일 클래스 — 오픽업 패널티 40점이라 높게 설정
CLUSTER_RADIUS_PX      = 300   # 이 픽셀 반경 안에 있는 다른 물체 개수로 밀집도 계산
OVERLAP_MERGE_RADIUS_PX = 40   # 이 픽셀 이내로 중심이 겹치면 "같은 물체"로 보고 과일 쪽 우선

def select_target(objects: list) -> dict | None:
    """--cls 필터 + 클래스별 confidence 임계값 통과한 것 중,
    주변에 다른 물체가 많이 몰려있는(밀집도 높은) 것 우선 선택 → 여러 개 연속으로 집기 쉬운 쪽으로 이동.
    밀집도가 같으면 area(가까운 정도)가 큰 쪽 우선.
    과일 큐브는 모양이 d6과 같아서 같은 물체에 shape+fruit 박스가 겹쳐 잡힐 수 있음 —
    이 경우 표면 이미지가 진짜 정체성이므로 과일 쪽을 우선(겹치는 shape 후보는 제거)."""
    if not objects:
        return None
    filtered = []
    for o in objects:
        if o['cls'] == 'flag':
            continue  # flag는 GO_TO_STORAGE 전용 — SEARCHING/GRIPPING 중엔 집을 대상으로 절대 선택 안 함
        if TARGET_CLS and o['cls'] not in TARGET_CLS:
            continue
        threshold = CONF_THRESHOLD_FRUIT if o['cls'] in FRUIT_CLASSES else CONF_THRESHOLD_SHAPE
        if o['conf'] >= threshold:
            filtered.append(o)
    if not filtered:
        return None

    fruit_candidates = [o for o in filtered if o['cls'] in FRUIT_CLASSES]
    shape_candidates = [o for o in filtered if o['cls'] not in FRUIT_CLASSES]
    for s in shape_candidates[:]:
        for f in fruit_candidates:
            dist = ((s['cx'] - f['cx']) ** 2 + (s['cy'] - f['cy']) ** 2) ** 0.5
            if dist <= OVERLAP_MERGE_RADIUS_PX:
                shape_candidates.remove(s)
                break
    filtered = fruit_candidates + shape_candidates

    def cluster_score(o):
        return sum(
            1 for other in filtered
            if other is not o and ((other['cx'] - o['cx']) ** 2 + (other['cy'] - o['cy']) ** 2) ** 0.5 <= CLUSTER_RADIUS_PX
        )

    return max(filtered, key=lambda o: (cluster_score(o), o['area']))


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
AREA_GRIP_THRESHOLD = 30000   # 이 면적 이상이면 정지 후 직진 접근 → grip
AREA_SLOW_THRESHOLD = 20000   # 이 면적 이상이면 감속 시작
AREA_ROTATE_THRESHOLD = 15000 # 이 이하일 때만 제자리 회전 정렬
MIN_DETECTED_FOR_EXPLORE = 2      # 탐색 이동 조건: 클래스 무관 총 탐지 개수가 이 이상이어야 시도
                                   # (이 미만, 즉 0~1개면 회전 탐색 프로세스로 방향을 잡는다)
MAX_ROTATE_SECS           = 5.0   # 제자리 회전 탐색 최대 지속 시간 — 넘으면 강제로 현재 방향 직진 (실측 필요)
SEARCH_FORWARD_BURST_SECS = 1.0   # 최대 회전 시간 초과 시 현재 방향으로 직진하는 시간 (실측 필요)
CENTER_MARGIN_PX    = 42      # 픽셀 모드: 가로 중심에서 이 픽셀 이내 (시각화 가이드용, 면적 2배)
CENTER_MARGIN_Y_PX  = 35      # 픽셀 모드: 세로 중심에서 이 픽셀 이내 (시각화 가이드용, 면적 2배)
CENTER_OFFSET_Y_PX  = 220     # 세로 중심 오프셋 (양수=아래)
CENTER_OFFSET_X_PX  = 0       # 가로 중심 오프셋 (양수=오른쪽)
ALIGN_THRESHOLD     = 0.25    # 이 이상 turn값이면 전진 없이 제자리 회전 우선
TURN_ONLY_SPEED     = 0.2     # 제자리 회전 속도
FINAL_APPROACH_SECS  = 1.7        # area 임계 도달 후 정지→직진하는 시간
FINAL_APPROACH_SPEED = MOVE_SPEED # 직진 접근 속도
FORWARD_TRIM = 0.025  # 직진 시 우측으로 쏠리는 것 보정 (양수=오른쪽 바퀴를 더 빠르게)

# 오인식 방지
CONFIRM_FRAMES      = 3       # 연속 N프레임 도달 조건 만족해야 grip 전송

# 탐색 회전
SEARCH_ROTATE_SPEED = 0.2     # 타겟 없을 때 제자리 회전 속도

# 타임아웃
GRIP_TIMEOUT_SECS    = 15.0   # grip 전송 후 gripped 신호 최대 대기
DROP_TIMEOUT_SECS    = 15.0   # drop 전송 후 done 신호 최대 대기
STORAGE_TIMEOUT_SECS = 60.0   # GO_TO_STORAGE 전체 최대 시간 (태극기 탐색 포함)

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
align_phase          = 0      # --align-only 전용: 0=회전으로 좌우(cx) 정렬, 1=전진/후진으로 상하(cy) 정렬
align_final_forward       = False  # --align-only 전용: cy 정렬 완료 후 1초 직진 중
align_final_forward_start = 0.0
align_final_forward_cls   = None
fb_phase          = 0      # 실제 grip 정밀 정렬: 0=전진/후진으로 상하(cy) 정렬, 1=회전으로 좌우(cx) 정렬
fb_final_forward       = False  # cx 정렬 완료 후 직진 중
fb_final_forward_start = 0.0
fb_final_forward_cls   = None
precise_align = False  # True면 area 임계 도달 후 정밀 정렬(전진/후진→회전) 진행 중
gripper_prepped = False  # True면 이번 접근을 위해 그리퍼를 미리 열어둔 상태 (grip 전송 또는 취소 시 False로 복귀)
search_rotate_start        = None   # 제자리 회전 탐색이 연속으로 시작된 시각 (None=회전 중 아님)
search_forward_burst       = False  # True면 회전 최대 시간 초과로 현재 방향 강제 직진 중
search_forward_burst_start = 0.0

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
                elif data.get("status") == "gripper_opened":
                    print("\n[OpenRB] 그리퍼 미리 열기 완료")
                elif data.get("status") == "gripper_closed":
                    print("\n[OpenRB] 그리퍼 대기 상태로 닫힘")
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
    if NO_WHEELS or ser_esp32 is None or not ser_esp32.is_open:
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
        turn     = max(-1.0, min(1.0, (target["cx"] - (frame_w / 2 + CENTER_OFFSET_X_PX)) / (frame_w / 2)))
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
    """도달(=정밀 정렬 진입) 여부 판단. mm 모드 → 거리, 픽셀 모드 → area 임계 도달.
    중심 정렬(cx/cy)은 여기서 안 보고, 도달 이후 정밀 정렬 단계(전진/후진→회전)에서 맞춘다."""
    if target.get("mx") is not None:
        dist = (target["mx"] ** 2 + target["my"] ** 2) ** 0.5
        return dist < ARRIVE_THRESHOLD_MM
    return target.get("area", 0) >= AREA_GRIP_THRESHOLD


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

def send_start():
    """경기 시작 — 전원 켤 때 시작 크기 규정으로 올려둔 팔을 내림."""
    if ser_openrb is None or not ser_openrb.is_open:
        return
    ser_openrb.write((json.dumps({"cmd": "start"}) + "\n").encode())

def send_gripper_open():
    """물체 쪽으로 접근하기 직전 — 그리퍼를 열어서 물체가 들어올 공간을 만든다.
    (IDLE 기본값이 닫힘이라, 이걸 안 하면 grip 시점에 손가락이 이미 닫혀있어 못 집음)"""
    if ser_openrb is None or not ser_openrb.is_open:
        return
    ser_openrb.write((json.dumps({"cmd": "gripper_open"}) + "\n").encode())

def send_gripper_close():
    """접근을 포기하고 재탐색으로 돌아갈 때 — 열어뒀던 그리퍼를 대기 상태로 되돌린다."""
    if ser_openrb is None or not ser_openrb.is_open:
        return
    ser_openrb.write((json.dumps({"cmd": "gripper_close"}) + "\n").encode())

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
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1200)
    cap.set(cv2.CAP_PROP_FPS, 50)
    ret, f = cap.read()
    if ret:
        h, w = f.shape[:2]
        print(f"[카메라] {name} ({index}번) 준비: {w}×{h}")
    return cap

cap = _init_camera(CAMERA_INDEX_OBJ, "물체캠")  # 물체+태극기 겸용 (전면)

if cap is None:
    print("[오류] 물체 카메라 없음 — 종료"); exit()

# 실제 카메라 해상도로 FRAME_W/H 보정 (calibration.json 없을 때)
if FRAME_W is None:
    _ret, _f = cap.read()
    if _ret:
        FRAME_H, FRAME_W = _f.shape[:2]

HEADLESS    = True  # X11 imshow 비활성화 (SSH+WiFi 병목 방지)
WINDOW_NAME = "MERO_AI_ROBOT"
if not HEADLESS:
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE)

mode_str = "mm 모드" if MM_PER_PIXEL else "픽셀 모드 (캘리브 없음)"

# --cls 미지정 시: 카메라/모델 다 뜬 상태에서 경기 당일 타겟 클래스 2개를 직접 입력받음.
# Enter 누르는 순간이 곧 "경기 시작" 신호 — 이 직후 send_start()로 팔이 내려감.
if TARGET_CLS is None:
    valid_classes = SHAPE_CLASSES | FRUIT_CLASSES
    while True:
        raw = input(f"[시작] 타겟 클래스 2개 입력 후 Enter (예: d8 apple) — 도형:{sorted(SHAPE_CLASSES)} 과일:{sorted(FRUIT_CLASSES)}: ").strip()
        cls_list = raw.split()
        if len(cls_list) == 2 and all(c in valid_classes for c in cls_list):
            TARGET_CLS = set(cls_list)
            break
        print("[오류] 정확히 2개, 유효한 클래스 이름만 입력하세요.")

cls_str = ' + '.join(sorted(TARGET_CLS))
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
    target=lambda: ThreadingHTTPServer(('0.0.0.0', 8080), _MJPEGHandler).serve_forever(),
    daemon=True
).start()

def _local_ip():
    """현재 연결된 네트워크 기준 실제 IP 확인 (핫스팟이 바뀌어도 자동으로 맞는 IP 표시)."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    except Exception:
        return "127.0.0.1"
    finally:
        s.close()

print(f"[스트림] http://{_local_ip()}:8080 에서 카메라 확인 가능 (또는 http://{socket.gethostname()}.local:8080)")

# ── 카메라 캡처 스레드 (cap.read 블로킹을 메인 루프에서 분리) ──

fps_counter = 0
fps_display = 0.0
fps_timer   = time.time()
_last_print_t = 0.0
frame = None  # 최초 루프 진입 전 초기화

send_start()  # 시작 크기 규정으로 올려둔 팔을 내림 (전진 시작과 함께)

# ── 메인 루프 ────────────────────────────────────────────
try:
    while True:
        # 카메라 1대로 모든 상태(SEARCHING/GRIPPING/GO_TO_STORAGE)에서 동일하게 탐지
        # GO_TO_STORAGE는 아래에서 detected 중 cls=='flag'만 걸러서 씀
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
                    _last_print_t = time.time()

        target    = select_target(detected)
        at_target = _is_at_target(target) if target else False

        # ── IMU 주기 요청 (GO_TO_STORAGE 제외) ──────────
        _now_loop = time.time()
        if robot_state != RobotState.GO_TO_STORAGE and _now_loop - _last_imu_req >= 0.15:
            if ser_esp32 and ser_esp32.is_open:
                ser_esp32.write((json.dumps({"T": 126}) + "\n").encode())
            _last_imu_req = _now_loop

        # ── 상태 머신 ──────────────────────────────────
        if robot_state == RobotState.SEARCHING:
            if align_final_forward:
                # cy 정렬 완료 후 1초 직진 → grip 전송 → GRIPPING (완료되면 다시 SEARCHING으로 반복)
                control_wheels(None, override_l=FINAL_APPROACH_SPEED - FORWARD_TRIM / 2, override_r=FINAL_APPROACH_SPEED + FORWARD_TRIM / 2)
                elapsed_af = time.time() - align_final_forward_start
                print(f"[테스트] 직진중... ({elapsed_af:.1f}s)", end="\r")
                if elapsed_af >= FINAL_APPROACH_SECS:
                    control_wheels(None)
                    align_final_forward = False
                    last_target_id      = -1
                    gripped_cls         = align_final_forward_cls
                    openrb_gripped      = False
                    openrb_dumped       = False
                    openrb_grip_failed  = False
                    send_grip({"cls": align_final_forward_cls})
                    print(f"\n[테스트] grip 전송 ({align_final_forward_cls})")
                    robot_state  = RobotState.GRIPPING
                    grip_sent_at = time.time()
                    print(f"[상태] SEARCHING → GRIPPING (grip: {align_final_forward_cls})")

            elif target and ALIGN_ONLY:
                # 방향 검증 전용: 1단계 제자리 회전(좌우 cx) → 2단계 직진/후진(상하 cy). 회전과 전진을 분리.
                frame_w = FRAME_W or 640
                frame_h = FRAME_H or 480
                cx_ref  = frame_w / 2 + CENTER_OFFSET_X_PX
                cy_ref  = frame_h / 2 + CENTER_OFFSET_Y_PX
                cx_aligned = abs(target["cx"] - cx_ref) <= CENTER_MARGIN_PX
                cy_aligned = abs(target["cy"] - cy_ref) <= CENTER_MARGIN_Y_PX

                if align_phase == 0:
                    if cx_aligned:
                        control_wheels(None)
                        print(f"\n[테스트] 좌우 정렬 완료 (cx={target['cx']:.0f}) → 1초 대기 후 전후 정렬 시작")
                        time.sleep(1.0)
                        align_phase = 1
                    else:
                        turn = max(-1.0, min(1.0, (target["cx"] - cx_ref) / (frame_w / 2)))
                        control_wheels(None, override_l=TURN_ONLY_SPEED * turn, override_r=-TURN_ONLY_SPEED * turn)
                        print(f"[테스트] 회전 정렬중... cx={target['cx']:.0f}", end="\r")

                else:
                    if not cx_aligned:
                        # 전후 이동 중 좌우가 틀어지면 회전 단계로 복귀
                        align_phase = 0
                    elif cy_aligned:
                        control_wheels(None)
                        align_phase                = 0
                        align_final_forward        = True
                        align_final_forward_start  = time.time()
                        align_final_forward_cls    = target["cls"]
                        print(f"\n[테스트] 상하 정렬 완료 (cy={target['cy']:.0f}) → 1초 직진")
                    else:
                        # cy_ref보다 위(작음)=목표가 더 멀리 있음 → 전진, 아래(큼)=너무 가까움 → 후진
                        # (카메라 장착 각도 기준 가정 — 방향 반대면 부호만 뒤집으면 됨)
                        fwd = SLOW_SPEED if target["cy"] < cy_ref else -SLOW_SPEED
                        control_wheels(None, override_l=fwd, override_r=fwd)
                        direction = "전진" if fwd > 0 else "후진"
                        print(f"[테스트] {direction} 정렬중... cy={target['cy']:.0f}", end="\r")

            elif fb_final_forward:
                # cx 정렬 완료 후 1초 직진 → grip 전송 → GRIPPING (완료되면 다시 SEARCHING으로 반복)
                control_wheels(None, override_l=FINAL_APPROACH_SPEED - FORWARD_TRIM / 2, override_r=FINAL_APPROACH_SPEED + FORWARD_TRIM / 2)
                elapsed_fb = time.time() - fb_final_forward_start
                print(f"[상태] 직진 접근중... ({elapsed_fb:.1f}s)", end="\r")
                if elapsed_fb >= FINAL_APPROACH_SECS:
                    control_wheels(None)
                    fb_final_forward   = False
                    last_target_id     = -1
                    gripped_cls        = fb_final_forward_cls
                    openrb_gripped     = False
                    openrb_dumped      = False
                    openrb_grip_failed = False
                    send_grip({"cls": fb_final_forward_cls})
                    gripper_prepped = False  # 이제부터는 OpenRB가 집기~재닫힘까지 직접 관리
                    print(f"\n[상태] grip 전송 ({fb_final_forward_cls})")
                    if TEST_MODE:
                        print("[테스트] 1회성 테스트 — grip 전송 후 종료")
                        break
                    robot_state  = RobotState.GRIPPING
                    grip_sent_at = time.time()
                    print(f"[상태] SEARCHING → GRIPPING (grip: {fb_final_forward_cls})")

            elif precise_align:
                # 실제 grip 정밀 정렬: 1단계 전진/후진(상하 cy) → 2단계 제자리 회전(좌우 cx)
                if not target:
                    # 정밀 정렬 중 타겟을 놓침 — 재탐색으로 복귀
                    precise_align = False
                    fb_phase      = 0
                    if gripper_prepped:
                        send_gripper_close()
                        gripper_prepped = False
                    print("\n[상태] 정밀 정렬 중 타겟 놓침 → 재탐색 (그리퍼 닫음)")
                else:
                    frame_w = FRAME_W or 640
                    frame_h = FRAME_H or 480
                    cx_ref  = frame_w / 2 + CENTER_OFFSET_X_PX
                    cy_ref  = frame_h / 2 + CENTER_OFFSET_Y_PX
                    cx_aligned = abs(target["cx"] - cx_ref) <= CENTER_MARGIN_PX
                    cy_aligned = abs(target["cy"] - cy_ref) <= CENTER_MARGIN_Y_PX

                    if fb_phase == 0:
                        if cy_aligned:
                            control_wheels(None)
                            fb_phase = 1
                            print(f"\n[상태] 상하 정렬 완료 (cy={target['cy']:.0f}) → 좌우 정렬")
                        else:
                            # cy_ref보다 위(작음)=목표가 더 멀리 있음 → 전진, 아래(큼)=너무 가까움 → 후진
                            fwd = SLOW_SPEED if target["cy"] < cy_ref else -SLOW_SPEED
                            control_wheels(None, override_l=fwd, override_r=fwd)
                            direction = "전진" if fwd > 0 else "후진"
                            print(f"[상태] {direction} 정렬중... cy={target['cy']:.0f}", end="\r")

                    else:
                        if not cy_aligned:
                            # 회전 중 상하가 틀어지면 전후 단계로 복귀
                            fb_phase = 0
                        elif cx_aligned:
                            control_wheels(None)
                            precise_align          = False
                            fb_phase                = 0
                            fb_final_forward        = True
                            fb_final_forward_start  = time.time()
                            fb_final_forward_cls    = target["cls"]
                            print(f"\n[상태] 좌우 정렬 완료 (cx={target['cx']:.0f}) → 직진 접근 시작")
                        else:
                            turn = max(-1.0, min(1.0, (target["cx"] - cx_ref) / (frame_w / 2)))
                            control_wheels(None, override_l=TURN_ONLY_SPEED * turn, override_r=-TURN_ONLY_SPEED * turn)
                            print(f"[상태] 회전 정렬중... cx={target['cx']:.0f}", end="\r")

            elif target:
                search_rotate_start  = None
                search_forward_burst = False
                if time.time() - _last_print_t >= 0.5:
                    print(f"[타겟] {target['cls']} | area={target['area']}")
                    _last_print_t = time.time()

                # 타겟이 보이면 area/중앙정렬 상관없이 바로 정밀 정렬(전진/후진→회전) 진입 —
                # 예전 --align-fwd-first와 동일한 방식 (거리 무관하게 즉시 시작)
                control_wheels(None)
                precise_align = True
                fb_phase      = 0
                send_gripper_open()
                gripper_prepped = True
                print(f"\n[상태] 타겟 발견 (area={target['area']}) → 그리퍼 열기 + 정밀 정렬 시작")

            else:
                align_phase   = 0
                fb_phase      = 0
                precise_align = False

                # 타겟 미검출 — 클래스/신뢰도 상관없이(flag는 제외) 탐지된 물체가 2개 이상이면
                # 그중 가장 먼(area 최소) 물체가 카메라 중심에 오도록 이동하며 탐색한다.
                # 1개 이하면(=비교 대상 없음) 회전 탐색 프로세스로 방향을 잡는다.
                explorable = [o for o in detected if o['cls'] != 'flag']
                farthest = min(explorable, key=lambda o: o["area"]) if explorable else None
                can_explore = len(explorable) >= MIN_DETECTED_FOR_EXPLORE and farthest is not None

                if can_explore:
                    control_wheels(farthest)
                    search_rotate_start  = None
                    search_forward_burst = False
                    if time.time() - _last_print_t >= 0.5:
                        print(f"[탐색] 물체 {len(explorable)}개 감지, 가장 먼 물체({farthest['cls']}, area={farthest['area']}) 방향으로 이동")
                        _last_print_t = time.time()

                elif search_forward_burst:
                    # 회전 최대 시간 초과 — 멈춘 방향으로 잠깐 직진 후 회전 재개
                    control_wheels(None, override_l=FINAL_APPROACH_SPEED, override_r=FINAL_APPROACH_SPEED)
                    print(f"[탐색] 회전 시간 초과 → 현재 방향 직진중...", end="\r")
                    if time.time() - search_forward_burst_start >= SEARCH_FORWARD_BURST_SECS:
                        search_forward_burst = False
                        search_rotate_start   = time.time()

                else:
                    if search_rotate_start is None:
                        search_rotate_start = time.time()

                    if time.time() - search_rotate_start >= MAX_ROTATE_SECS:
                        search_forward_burst       = True
                        search_forward_burst_start = time.time()
                        print(f"\n[탐색] 제자리 회전 {MAX_ROTATE_SECS:.0f}초 초과 → 현재 방향으로 직진 전환")
                    else:
                        control_wheels(None, override_l=-SEARCH_ROTATE_SPEED, override_r=SEARCH_ROTATE_SPEED)

                send_idle()

        elif robot_state == RobotState.GRIPPING:
            control_wheels(None)
            elapsed = time.time() - grip_sent_at
            if openrb_gripped:
                openrb_gripped      = False
                openrb_dumped       = False
                openrb_grip_failed  = False
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

            # 같은 카메라/모델의 detected에서 flag 클래스만 걸러서 사용
            elif not FLAG_CLASS_AVAILABLE:
                print("[경고] flag 클래스 없음 — SEARCHING 복귀")
                robot_state = RobotState.SEARCHING

            else:
                flag_candidates = [o for o in detected if o['cls'] == 'flag' and o['conf'] >= FLAG_CONF_THRESHOLD]
                flag_detected = max(flag_candidates, key=lambda o: o['conf']) if flag_candidates else None

                fw2 = FRAME_W or 640

                if storage_phase == 0:
                    # 태극기 탐색 — 제자리 회전
                    if flag_detected:
                        storage_phase       = 1
                        storage_phase_start = now
                        print(f"\n[상태] 태극기 발견 → 전진 접근 시작")
                    else:
                        control_wheels(None, override_l=FLAG_SEARCH_SPEED, override_r=-FLAG_SEARCH_SPEED)
                        print(f"[상태] 태극기 탐색 회전중... ({total_elapsed:.1f}s)", end="\r")

                elif storage_phase == 1:
                    # 태극기 전진 접근 (전면 카메라라 정방향으로 다가감)
                    if not flag_detected:
                        # 태극기 놓침 → 탐색으로 복귀
                        control_wheels(None)
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
                        # 전진하면서 정렬
                        turn  = (flag_detected["cx"] - fw2 / 2) / (fw2 / 2)
                        speed = FLAG_APPROACH_SLOW if flag_detected["area"] > FLAG_AREA_SLOW_THRESHOLD else FLAG_APPROACH_SPEED
                        L = speed + turn * 0.3
                        R = speed - turn * 0.3
                        control_wheels(None, override_l=L, override_r=R)
                        print(f"[상태] 전진 접근중... area={flag_detected['area']:.0f}", end="\r")

        elif robot_state == RobotState.DROPPING:
            control_wheels(None)
            elapsed = time.time() - drop_sent_at
            if openrb_dumped:
                openrb_dumped  = False
                confirm_count  = 0
                last_target_id = -1
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
        _cx = _fw // 2 + CENTER_OFFSET_X_PX
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
