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

경기 시작 시 "남은 경기 시간"을 MM:SS로 입력받음(처음 시작이면 그냥 Enter = 3:00) —
match_start_time을 그만큼 과거로 당겨서 실제 경기 시계와 동기화. 카메라 hang 등으로
경기 도중 main.py를 재시작해야 할 때, 그 시점의 실제 남은 시간을 입력하면 됨
(안 하면 PICK_PHASE_SECS 카운트다운이 재시작 시점부터 새로 시작돼서 실제 경기 종료
전에 GO_TO_STORAGE 전환을 영영 못 하는 문제가 있었음). 이 입력 직후 PICK_PHASE_SECS
(150s=2분30초) 동안 SEARCHING/GRIPPING/
POST_GRIP_SCAN을 반복하다가, 그 시간이 지나면 셋 중 어느 상태에 있든(grip
응답 대기 중이든 집기후 스캔 중이든) 매 프레임 즉시 GO_TO_STORAGE로 전환됨
(남은 30초 동안 회전하며 flag 탐색 → 1프레임이라도 감지되면 정렬/접근 없이 즉시 정지, 경기 종료 취급).
전환 시점에
cam_backward(카메라 후방)와 arm_up(팔 규정 크기 위치 복귀)을 동시에 전송함. 이 체크는
상태머신 분기 진입 전에 한 번만 수행 — GRIPPING/POST_GRIP_SCAN에서도
안 걸리면 최악의 경우(grip 타임아웃 15초+스캔 4초) 30초 중 19초를
까먹을 수 있어서 반드시 세 상태 모두에서 체크해야 함.

상태 머신:
  SEARCHING      — flag 제외 물체 탐지, 보이면 거리 상관없이 바로 정밀 정렬
                   (전후진+회전 동시 보정) 진입 → 정렬 끝나면 직진 접근 후 grip 전송
  GRIPPING       — grip 전송 후 gripped 신호 대기 (집기+팔올림+투하+팔내림 완료)
                   → gripped 수신 시 POST_GRIP_SCAN
  POST_GRIP_SCAN — 집기 직후 POST_GRIP_BACKUP_SECS(1초) 후진 후 제자리 스캔,
                   타겟 있으면/시간 다 차면 SEARCHING
  GO_TO_STORAGE  — 제자리 회전하며 flag 탐색 → 1프레임이라도 감지되면 정렬/접근 없이
                   즉시 정지 (dump 명령 없음, 여기서 경기 종료 취급)

시리얼:
  /dev/ttyACM0 → ESP32  (UGV02 바퀴)   {"T":1, "L":speed, "R":speed}
  /dev/ttyACM1 → OpenRB (팔·그리퍼)    {"cmd":"grip"/"idle"/"gripper_open"/"gripper_close"}

OpenRB 응답:
  {"status":"gripped"}        — 집기+컨테이너 투하+그리퍼 재닫힘 완료 → SEARCHING 복귀
  {"status":"grip_failed"}    — 집기 실패 → SEARCHING 복귀
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
import fcntl
import glob
import os
import json
import queue
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
parser.add_argument('--record', action='store_true',
                    help='테스트 중 프레임을 저해상도 JPEG로 샘플링해 저장 (vision/records/<시각>/) — 백그라운드 스레드+낮은 fps라 부하 거의 없음')
parser.add_argument('--record-raw', action='store_true',
                    help='--record와 같이 씀: 박스/오버레이 없는 원본 프레임을 저장 (기본은 탐지 박스 그려진 화면 저장)')
parser.add_argument('--storage-only', action='store_true',
                    help='SEARCHING/GRIPPING 건너뛰고 시작하자마자 바로 GO_TO_STORAGE로 진입 (태극기 정렬+접근+dump만 단독 테스트)')
args       = parser.parse_args()
TARGET_CLS    = set(args.cls) if args.cls else None
TEST_MODE     = args.test
ALIGN_ONLY    = args.align_only
STORAGE_ONLY  = args.storage_only
NO_WHEELS     = args.no_wheels
RECORD        = args.record
RECORD_RAW    = args.record_raw
SHAPE_CLASSES = {'d6', 'd8', 'd12', 'd20'}
FRUIT_CLASSES = {'apple', 'banana', 'orange', 'pineapple'}

# ── 모델 로드 (통합 모델 하나 — 도형+과일+flag 전부 포함) ──
BASE_DIR   = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODEL_PATH = os.path.join(BASE_DIR, "model", "best.engine")  # 2026-07-22 TensorRT 변환 완료, 전환
# 변환 전(PyTorch) 원본: MODEL_PATH = os.path.join(BASE_DIR, "model", "best.pt")
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
    """--cls 필터 + 클래스별 confidence 임계값 통과한 것 중 가장 오른쪽(cx 큰 것) 하나를 고른다.
    예전엔 area(화면 중앙 가까운/큰 것) 기준이었는데, 같은 물체 2개가 붙어있는 경우처럼
    area만으로 고르기 애매한 상황 대응을 위해 오른쪽 우선으로 변경. (정밀 정렬 시작 후에는
    이 함수를 다시 안 부르고 last_target_id로 같은 물체를 계속 추적하니, 이 함수는
    "처음에 뭘 고를지"만 담당한다.)
    과일 큐브는 모양이 d6과 같아서 같은 물체에 shape+fruit 박스가 겹쳐 잡힐 수 있음 —
    이 경우 표면 이미지가 진짜 정체성이므로 과일 쪽을 우선(겹치는 shape 후보는 제거)."""
    if not objects:
        return None
    filtered = []
    for o in objects:
        if o['cls'] == 'flag':
            continue  # flag는 GO_TO_STORAGE 전용 — SEARCHING/GRIPPING 중엔 집을 대상으로 절대 선택 안 함
        if o['id'] == -1:
            continue  # 트래커가 아직 id를 못 붙인 물체 — last_target_id의 "락 없음" 값(-1)과 겹쳐서
                       # 나중에 정밀 정렬 중 엉뚱한 (역시 id=-1인) 물체와 혼동될 수 있어 애초에 제외
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

    # 후보가 여러 개면(같은 물체끼리 붙어있는 경우 등 area만으로 고르기 애매한 상황)
    # 오른쪽에 있는 것부터(cx 큰 순) 우선 선택 — 후보 1개면 그거 그대로 반환
    return max(filtered, key=lambda o: o['cx'])


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

# ── 카메라 프레임 실패 허용치 ────────────────────────────
# select() timeout 등 USB 대역폭 타이밍 노이즈로 가끔 프레임 읽기가 실패하는데,
# 랜덤하게 튀는 수준이라 너무 낮으면 멀쩡한데도 종료될 수 있어 여유를 둠.
FRAME_FAIL_LIMIT = 100  # 연속 이 횟수만큼 실패해야 진짜 종료

# ── 바퀴 제어 파라미터 ───────────────────────────────────
MOVE_SPEED          = 0.35
SLOW_SPEED          = 0.25

# mm 모드 (calibration 있을 때)
ARRIVE_THRESHOLD_MM = 30.0
SLOW_THRESHOLD_MM   = 100.0
MAX_MX              = 200.0
MAX_MY              = 150.0

# 픽셀 모드 (calibration 없을 때) — bbox 면적 기반
AREA_GRIP_THRESHOLD = 30000   # 이 면적 이상이면 정지 후 직진 접근 → grip
AREA_SLOW_THRESHOLD = 20000   # 이 면적 이상이면 감속 시작
AREA_ROTATE_THRESHOLD = 15000 # 이 이하일 때만 제자리 회전 정렬
MIN_DETECTED_FOR_EXPLORE = 3      # 탐색 이동 조건: 클래스 무관 총 탐지 개수가 이 이상이어야 시도
                                   # (이 미만, 즉 0~1개면 제자리 회전만 계속함 — 전진 안 함,
                                   #  안 보이는 방향으로 무작정 전진하면 벽에 부딪힐 수 있어서)
                                   # SEARCH_ROTATE_SPEED 기준 3초≈180도 (실측)
POST_GRIP_SCAN_SECS       = 4.0   # 집기 완료 직후 제자리 360도 스캔 시간 (SEARCH_ROTATE_SPEED 기준, 실측 필요)
POST_GRIP_BACKUP_SECS     = 1.0   # 회전 스캔 시작 전 먼저 뒤로 후진하는 시간
POST_GRIP_BACKUP_SPEED    = 0.2   # 후진 속도
CENTER_MARGIN_PX    = 42      # 픽셀 모드: 가로 중심에서 이 픽셀 이내 (시각화 가이드용, 면적 2배)
CENTER_MARGIN_Y_PX  = 35      # 픽셀 모드: 세로 중심에서 이 픽셀 이내 (시각화 가이드용, 면적 2배)
CENTER_OFFSET_Y_PX  = 170     # 세로 중심 오프셋 (양수=아래)
CENTER_OFFSET_X_PX  = 0       # 가로 중심 오프셋 (양수=오른쪽)
ALIGN_THRESHOLD     = 0.25    # 이 이상 turn값이면 전진 없이 제자리 회전 우선
TURN_ONLY_SPEED     = 0.1     # 제자리 회전 속도
FINAL_APPROACH_SECS  = 1.7        # area 임계 도달 후 정지→직진하는 시간
FINAL_APPROACH_SPEED = 0.25       # 직진 접근 속도
FORWARD_TRIM = 0.025  # 직진 시 우측으로 쏠리는 것 보정 (양수=오른쪽 바퀴를 더 빠르게)

# 오인식 방지
CONFIRM_FRAMES      = 3       # 연속 N프레임 도달 조건 만족해야 grip 전송

# 정밀 정렬 (precise_align) 전용 속도 — SLOW_SPEED/TURN_ONLY_SPEED는 탐색 밀집이동에서도
# 같이 쓰여서 건드리면 그쪽도 같이 바뀌므로, 정밀 정렬만 따로 뗀 전용 상수를 쓴다.
PRECISE_ALIGN_FB_SPEED   = 0.25  # 1단계 전후(cy) 정렬 속도
PRECISE_ALIGN_TURN_SPEED = 0.25  # 2단계 좌우(cx) 회전 정렬 속도
TARGET_MISS_GRACE_FRAMES = 10    # 정밀 정렬 중 순간적으로 타겟을 놓쳐도 이 프레임 수까지는 포기 안 하고 정지 대기 (모션블러 등 프레임 단위 오탐 대응)
GRIPPER_OPEN_LEAD_SECS   = 0.3    # gripper_open 명령 후 직진 시작까지 짧게 두는 텀 (그리퍼가 실제로 열리는 물리 시간, gripper.ino wait_ms=300과 맞춤)

# 탐색 회전
SEARCH_ROTATE_SPEED = 0.1     # 타겟 없을 때 제자리 회전 속도

# 타임아웃
GRIP_TIMEOUT_SECS    = 15.0   # grip 전송 후 gripped 신호 최대 대기
STORAGE_TIMEOUT_SECS = 60.0   # GO_TO_STORAGE 전체 최대 시간 (태극기 탐색 포함)

# 경기 타이머 / 픽업↔보관 전환
MATCH_DURATION_SECS = 180.0
PICK_PHASE_SECS      = 150.0  # 이 시간(2분30초) 지나면 SEARCHING/GRIPPING 중이든 상관없이 GO_TO_STORAGE로 전환
SHOW_TIMER            = args.timer  # 화면에 카운트다운 표시 여부 (전환 로직 자체는 --timer 없어도 항상 동작)

# ── 태극기 네비게이션 파라미터 ──────────────────────────
# 정렬/접근 없이 태극기 1프레임 감지 즉시 정지 — 실측 안 된 area 임계값에 기대는
# 대신 단순하고 확실한 쪽 선택 (2026-07-24).
FLAG_CONF_THRESHOLD  = 0.5
FLAG_SEARCH_SPEED    = 0.07   # 탐색 회전 속도

# ── 상태 머신 ────────────────────────────────────────────
class RobotState(Enum):
    SEARCHING      = "SEARCHING"
    GRIPPING       = "GRIPPING"
    POST_GRIP_SCAN = "POST_GRIP_SCAN"
    GO_TO_STORAGE  = "GO_TO_STORAGE"  # 제자리 회전하며 태극기 탐색 → 1프레임이라도 감지되면 즉시 정지 (경기 종료 취급)

robot_state          = RobotState.SEARCHING
grip_sent_at         = 0.0
flag_arrived             = False  # 태극기 감지되어 정지 완료 — 이후 아무 것도 안 함
storage_enter_time   = 0.0
confirm_count        = 0
last_target_id       = -1
post_grip_scan_start = 0.0   # POST_GRIP_SCAN 진입 시각
gripped_cls         = None
align_phase          = 0      # --align-only 전용: 0=회전으로 좌우(cx) 정렬, 1=전진/후진으로 상하(cy) 정렬
align_final_forward       = False  # --align-only 전용: cy 정렬 완료 후 1초 직진 중
align_final_forward_start = 0.0
align_final_forward_cls   = None
fb_final_forward       = False  # cx/cy 정렬 완료 후 직진 중
fb_final_forward_start = 0.0
fb_final_forward_cls   = None
gripper_open_wait       = False  # cx/cy 정렬 완료 → gripper_open 명령 보낸 직후 GRIPPER_OPEN_LEAD_SECS만큼 짧게 정지 대기 (응답 확인 아님, 고정 시간만)
gripper_open_wait_start = 0.0
gripper_open_wait_cls   = None
precise_align = False  # True면 area 임계 도달 후 정밀 정렬(전후진+회전 동시 보정) 진행 중
target_miss_count = 0  # precise_align 중 연속으로 타겟을 못 잡은 프레임 수 (TARGET_MISS_GRACE_FRAMES까지는 정지 대기)
gripper_prepped = False  # True면 이번 접근을 위해 그리퍼를 미리 열어둔 상태 (grip 전송 또는 취소 시 False로 복귀)
search_rotate_start        = None   # 제자리 회전 탐색이 연속으로 시작된 시각 (None=회전 중 아님, 로그 표시용)

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
openrb_grip_failed = False

def _read_openrb_loop():
    global openrb_gripped, openrb_grip_failed
    while True:
        if ser_openrb is None or not ser_openrb.is_open:
            time.sleep(0.5); continue
        try:
            if ser_openrb.in_waiting > 0:
                raw = ser_openrb.readline().decode("utf-8", errors="ignore").strip()
                if not raw:
                    time.sleep(0.01); continue
                try:
                    data = json.loads(raw)
                except json.JSONDecodeError:
                    time.sleep(0.01); continue
                if data.get("status") == "gripped":
                    openrb_gripped = True
                    print("\n[OpenRB] 집기+투하 완료")
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





# ── 시리얼 write 공통 헬퍼 ─────────────────────────────────
# USB 케이블 순간 접촉불량/전원 문제 등으로 write가 실패하면(SerialException) 그 프레임만
# 건너뛰고 계속 돌게 한다 — 예전엔 처리 안 돼 있어서 한 번 끊기면 프로그램 전체가 죽었음.
# control_wheels()가 매 프레임 _write_esp32를 부르므로, 연결이 끊긴 채 계속 돌면 경고가
# 초당 수십 번 찍혀 터미널이 도배될 수 있어 경고 출력 자체는 WRITE_FAIL_WARN_INTERVAL마다로 제한.
# 카메라 재연결(_reopen_camera)과 동일한 이유로, write가 연속 실패하면 죽은 fd를 들고
# 있는 ser_esp32/ser_openrb를 닫고 _find_port로 다시 찾아서 재연결 시도한다.
WRITE_FAIL_WARN_INTERVAL     = 2.0
SERIAL_RECONNECT_AFTER_FAILS = 20
_last_esp32_fail_warn   = 0.0
_last_openrb_fail_warn  = 0.0
_esp32_fail_count       = 0
_openrb_fail_count      = 0

def _reconnect_esp32():
    global ser_esp32
    try:
        if ser_esp32 is not None:
            ser_esp32.close()
    except Exception:
        pass
    port = _find_port(["1a86", "ch343", "ch34"], ESP32_PORT)
    ser_esp32 = _open_serial(port)
    return ser_esp32 is not None

def _reconnect_openrb():
    global ser_openrb
    try:
        if ser_openrb is not None:
            ser_openrb.close()
    except Exception:
        pass
    port = _find_port(["openrb", "robotis", "2ecc"], OPENRB_PORT)
    ser_openrb = _open_serial(port)
    return ser_openrb is not None

def _write_esp32(payload: dict):
    global _last_esp32_fail_warn, _esp32_fail_count
    if ser_esp32 is None or not ser_esp32.is_open:
        return
    try:
        ser_esp32.write((json.dumps(payload) + "\n").encode())
        _esp32_fail_count = 0
    except serial.SerialException as e:
        _esp32_fail_count += 1
        now = time.time()
        if now - _last_esp32_fail_warn >= WRITE_FAIL_WARN_INTERVAL:
            print(f"\n[경고] ESP32 write 실패(연결 확인 필요, 이후 {WRITE_FAIL_WARN_INTERVAL:.0f}초간 반복 로그 생략): {e}")
            _last_esp32_fail_warn = now
        if _esp32_fail_count % SERIAL_RECONNECT_AFTER_FAILS == 0:
            print(f"\n[ESP32] 연속 {_esp32_fail_count}회 write 실패 — 재연결 시도")
            if _reconnect_esp32():
                print("[ESP32] 재연결 성공")
                _esp32_fail_count = 0
            else:
                print("[ESP32] 재연결 실패 — 계속 재시도")

def _write_openrb(payload: dict):
    global _last_openrb_fail_warn, _openrb_fail_count
    if ser_openrb is None or not ser_openrb.is_open:
        return
    try:
        ser_openrb.write((json.dumps(payload) + "\n").encode())
        _openrb_fail_count = 0
    except serial.SerialException as e:
        _openrb_fail_count += 1
        now = time.time()
        if now - _last_openrb_fail_warn >= WRITE_FAIL_WARN_INTERVAL:
            print(f"\n[경고] OpenRB write 실패(연결 확인 필요, 이후 {WRITE_FAIL_WARN_INTERVAL:.0f}초간 반복 로그 생략): {e}")
            _last_openrb_fail_warn = now
        if _openrb_fail_count % SERIAL_RECONNECT_AFTER_FAILS == 0:
            print(f"\n[OpenRB] 연속 {_openrb_fail_count}회 write 실패 — 재연결 시도")
            if _reconnect_openrb():
                print("[OpenRB] 재연결 성공")
                _openrb_fail_count = 0
            else:
                print("[OpenRB] 재연결 실패 — 계속 재시도")


# ── 바퀴 제어 ────────────────────────────────────────────
# 급가속으로 인한 배터리 순간 피크전류 완화 — 목표 속도로 바로 점프하지 않고
# 초당 MAX_ACCEL_PER_SEC만큼씩만 접근하게 램핑한다 (UGV 배터리 BMS가 부하 급증 시
# 과전류 보호로 순간 뚝 떨어지는 현상 확인됨 — 실측 후 값 조정 필요).
MAX_ACCEL_PER_SEC = 1.0
_last_wheel_l = 0.0
_last_wheel_r = 0.0
_last_wheel_t = None

def _ramp_speed(target_v, last_v, dt):
    max_step = MAX_ACCEL_PER_SEC * dt
    if target_v > last_v:
        return min(target_v, last_v + max_step)
    return max(target_v, last_v - max_step)

def control_wheels(target: dict | None, override_l: float | None = None, override_r: float | None = None):
    """
    override 지정 시 직접 속도 전송 (고정 경로 이동용).
    target 있으면 mm 또는 픽셀 기반 차동 조향.
    target=None이면 정지.
    최종 L/R은 항상 _ramp_speed()를 거쳐 전송 — 급가속 방지.
    """
    global _last_wheel_l, _last_wheel_r, _last_wheel_t
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

    now = time.time()
    dt = (now - _last_wheel_t) if _last_wheel_t is not None else 999.0  # 첫 호출은 램프 없이 그대로
    L = _ramp_speed(L, _last_wheel_l, dt)
    R = _ramp_speed(R, _last_wheel_r, dt)
    _last_wheel_l, _last_wheel_r, _last_wheel_t = L, R, now

    _write_esp32({"T": 1, "L": round(L, 2), "R": round(R, 2)})

    elapsed_str = f"{now - match_start_time:.2f}" if 'match_start_time' in globals() else ""
    bat_str     = f"{battery_v:.2f}" if battery_v is not None else ""
    _wheel_log_f.write(f"{now:.3f},{elapsed_str},{robot_state.value},{L:.2f},{R:.2f},{bat_str}\n")


def _is_at_target(target: dict) -> bool:
    """도달(=정밀 정렬 진입) 여부 판단. mm 모드 → 거리, 픽셀 모드 → area 임계 도달.
    중심 정렬(cx/cy)은 여기서 안 보고, 도달 이후 정밀 정렬 단계(전후진+회전 동시 보정)에서 맞춘다."""
    if target.get("mx") is not None:
        dist = (target["mx"] ** 2 + target["my"] ** 2) ** 0.5
        return dist < ARRIVE_THRESHOLD_MM
    return target.get("area", 0) >= AREA_GRIP_THRESHOLD


# ── OpenRB 명령 전송 ─────────────────────────────────────
def send_grip(target: dict):
    _write_openrb({
        "cmd": "grip",
        "cls": target["cls"],
        "mx":  target.get("mx", 0),
        "my":  target.get("my", 0),
    })

def send_start():
    """경기 시작 — 전원 켤 때 시작 크기 규정으로 올려둔 팔을 내림."""
    _write_openrb({"cmd": "start"})

def send_gripper_open():
    """물체 쪽으로 접근하기 직전 — 그리퍼를 열어서 물체가 들어올 공간을 만든다.
    (IDLE 기본값이 닫힘이라, 이걸 안 하면 grip 시점에 손가락이 이미 닫혀있어 못 집음)"""
    _write_openrb({"cmd": "gripper_open"})

def send_gripper_close():
    """접근을 포기하고 재탐색으로 돌아갈 때 — 열어뒀던 그리퍼를 대기 상태로 되돌린다."""
    _write_openrb({"cmd": "gripper_close"})

def send_cam_backward():
    """보관함으로 가기 직전 — 카메라를 뒤로 180도 돌려 후방을 보게 한다. 경기당 1회만 호출."""
    _write_openrb({"cmd": "cam_backward"})

def send_arm_up():
    """보관함으로 가기 직전 — 팔을 규정 크기 위치(올림)로 복귀. cam_backward와 동시에 호출.
    OpenRB가 IDLE 상태일 때만 처리됨(robot.ino) — GRIPPING/LIFTING 중이면 무시되지만
    그 경우 팔은 이미 해당 시퀀스 자체에서 올라가는 중이라 문제 없음."""
    _write_openrb({"cmd": "arm_up"})

_last_idle_t = 0.0
def send_idle():
    global _last_idle_t
    now = time.time()
    if now - _last_idle_t >= 1.0:
        _write_openrb({"cmd": "idle"})
        _last_idle_t = now


# ── 카메라 초기화 ────────────────────────────────────────
def _init_camera(index, name):
    # cv2.VideoCapture(index)처럼 정수만 주면 OpenCV가 백엔드를 자동으로 이것저것
    # 시도하는데, 이 과정에서 V4L2가 실패하면 엉뚱한 백엔드(obsensor 등)로 넘어가며
    # "index out of range" 같은 헷갈리는 에러를 냄(실제 확인됨). 장치 경로를
    # 직접 주고 백엔드를 V4L2로 명시해서 이 문제를 피한다.
    cap = cv2.VideoCapture(f"/dev/video{index}", cv2.CAP_V4L2)
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

USBDEVFS_RESET = 21780  # _IO('U', 20) — linux/usbdevice_fs.h

def _usb_reset_for_video(video_index):
    """/dev/videoN이 실제 USB 카메라에 물려있는 채로 응답이 끊긴(좀비) 경우,
    커널 목록엔 남아있어서 재오픈만으로는 안 풀리는 경우가 있다(확인됨:
    can't open camera by index / index out of range 하면서 이름 검색은
    여전히 그 장치를 찾아냄). USBDEVFS_RESET ioctl로 그 USB 장치만
    소프트 리셋 — 케이블 뽑았다 꽂는 것과 비슷한 효과. sudo 권한 필요할 수
    있음(권한 없으면 실패 로그만 남기고 계속 진행, 프로그램 안 죽음)."""
    try:
        real_path = os.path.realpath(f"/sys/class/video4linux/video{video_index}/device")
        d = real_path
        for _ in range(6):
            busnum_path = os.path.join(d, "busnum")
            devnum_path = os.path.join(d, "devnum")
            if os.path.exists(busnum_path) and os.path.exists(devnum_path):
                with open(busnum_path) as f:
                    busnum = int(f.read().strip())
                with open(devnum_path) as f:
                    devnum = int(f.read().strip())
                usb_path = f"/dev/bus/usb/{busnum:03d}/{devnum:03d}"
                fd = os.open(usb_path, os.O_WRONLY)
                try:
                    fcntl.ioctl(fd, USBDEVFS_RESET, 0)
                finally:
                    os.close(fd)
                print(f"[카메라] USB 리셋 성공 ({usb_path})")
                return True
            d = os.path.dirname(d)
        print("[카메라] USB 장치 경로를 못 찾음 — 리셋 실패")
        return False
    except Exception as e:
        print(f"[카메라] USB 리셋 실패({e}) — sudo 권한 필요할 수 있음")
        return False

def _reopen_camera():
    """재연결용 — 시작할 때 한 번 찾은 고정 번호(CAMERA_INDEX_OBJ) 대신, 시도할
    때마다 이름으로 다시 검색해서 실제 번호를 찾는다. USB 카메라가 실제로
    빠졌다 다시 잡히면 /dev/videoN 번호가 바뀌는 경우가 있어(재연결 시도가
    계속 옛날 번호로만 열려다 실패하는 문제 확인됨), 매번 새로 찾아야 함."""
    index = _find_camera_index(["arducam"], CAMERA_INDEX_OBJ)
    new_cap = _init_camera(index, "물체캠")
    if new_cap is None:
        # 재오픈 자체가 실패 — 좀비 USB 상태일 수 있으니 리셋 시도 후 한 번 더
        if _usb_reset_for_video(index):
            time.sleep(1.0)
            new_cap = _init_camera(index, "물체캠")
    return new_cap

cap = _init_camera(CAMERA_INDEX_OBJ, "물체캠")  # 물체+태극기 겸용 (전면)

if cap is None:
    print("[오류] 물체 카메라 없음 — 종료"); exit()

# 실제 카메라 해상도로 FRAME_W/H 보정 (calibration.json 없을 때)
if FRAME_W is None:
    _ret, _f = cap.read()
    if _ret:
        FRAME_H, FRAME_W = _f.shape[:2]

# ── 카메라 캡처 스레드 ────────────────────────────────────
# cap.read()가 USB 대역폭 타이밍 노이즈(select() timeout)로 몇 초씩 블로킹되는
# 경우가 있는데, 예전엔 메인 루프에서 직접 cap.read()를 불러서 그 몇 초 동안
# control_wheels()/send_idle()이 전혀 안 불렸다. ESP32는 ~3초 안에 새 속도
# 명령이 안 오면 하트비트 워치독으로 모터를 자동 정지시키는데(progress.md 기록),
# 이게 바로 "갑자기 전원 나간 것처럼 멈추는" 증상의 실제 원인으로 보임 —
# 배터리 전압 문제가 아니라 카메라 블로킹→명령 끊김→ESP32 자체 안전정지였음.
# 캡처를 별도 스레드로 분리해서 메인 루프는 항상 최신 프레임(또는 잠깐 멈췄으면
# 마지막 프레임)으로 계속 돌게 하고, control_wheels/send_idle이 카메라 상태와
# 무관하게 매 루프 계속 불리게 한다.
_cap_lock           = threading.Lock()
_cap_latest_frame   = None
_cap_fail_count     = 0
_cap_last_update_t  = time.time()
CAMERA_RECONNECT_AFTER_FAILS = 20   # cap.read()가 False를 이만큼 연속 반환하면 재연결
CAMERA_STALL_TIMEOUT_SECS    = 5.0  # cap.read() 자체가 이만큼 응답을 안 주면(진짜 hang) 통째로 새로 띄움

def _camera_capture_loop(own_cap):
    """own_cap 하나만 계속 읽는 캡처 루프. cap.read()가 완전히 hang되면(watchdog이
    감지) 이 함수는 그 안에서 영원히 멈춘 채로 버려지고, 새 own_cap으로 새
    스레드가 따로 뜬다 — 그래서 전역 cap을 직접 안 쓰고 인자로 고정해서, 만약
    이 스레드가 나중에 살아 돌아오더라도 이미 교체된 새 카메라 객체를 건드리지
    않게 한다(같은 VideoCapture 객체를 두 스레드가 동시에 read()하면 위험함)."""
    global cap, _cap_latest_frame, _cap_fail_count, _cap_last_update_t
    while True:
        ret, f = own_cap.read()
        with _cap_lock:
            if own_cap is not cap:
                return  # watchdog이 이미 이 스레드를 버리고 새 캡처로 교체함 — 조용히 종료
            if ret:
                _cap_latest_frame  = f
                _cap_fail_count    = 0
                _cap_last_update_t = time.time()
            else:
                _cap_fail_count += 1
            fail_count = _cap_fail_count

        if not ret and fail_count > 0 and fail_count % CAMERA_RECONNECT_AFTER_FAILS == 0:
            print(f"\n[카메라] 연속 {fail_count}회 읽기 실패 — 재연결 시도")
            try:
                own_cap.release()
            except Exception:
                pass
            # release 직후 바로 재오픈하면 OS/드라이버가 장치를 아직 안 놓아줘서
            # "device busy"로 실패하는 경우가 있어 잠깐 텀을 둔다.
            time.sleep(1.0)
            new_cap = _reopen_camera()
            if new_cap is not None:
                own_cap = new_cap
                with _cap_lock:
                    cap = new_cap
                print("[카메라] 재연결 성공")
            else:
                print("[카메라] 재연결 실패 — 1초 후 계속 재시도")
                time.sleep(1.0)

def _camera_watchdog_loop():
    """_camera_capture_loop는 cap.read()가 최소한 리턴은 해야 동작하는데, USB가
    완전히 맛이 가면 read() 자체가 영원히 안 돌아오는 경우가 있다(실패 카운트
    로직 자체가 발동할 기회가 없음). 이건 밖에서 "마지막 프레임 받은 지 얼마나
    지났는지"로 감시하다가, 너무 오래 조용하면 죽은 캡처 스레드는 버리고
    (daemon이라 프로세스 종료시 알아서 정리됨) 새 카메라 객체+새 캡처 스레드를
    통째로 새로 띄운다."""
    global cap, _cap_last_update_t
    while True:
        time.sleep(1.0)
        with _cap_lock:
            stale = time.time() - _cap_last_update_t
        if stale >= CAMERA_STALL_TIMEOUT_SECS:
            print(f"\n[카메라] {stale:.1f}초간 응답 없음(hang 의심) — 캡처 스레드 새로 띄움")
            new_cap = _reopen_camera()
            with _cap_lock:
                _cap_last_update_t = time.time()  # 실패해도 재시도 텀 확보(스팸 방지)
                if new_cap is not None:
                    cap = new_cap
            if new_cap is not None:
                threading.Thread(target=_camera_capture_loop, args=(new_cap,), daemon=True).start()
                print("[카메라] 새 캡처 스레드 시작")
            else:
                print("[카메라] 재오픈 실패 — 계속 재시도")

threading.Thread(target=_camera_capture_loop, args=(cap,), daemon=True).start()
threading.Thread(target=_camera_watchdog_loop, daemon=True).start()

HEADLESS    = True  # X11 imshow 비활성화 (SSH+WiFi 병목 방지)
WINDOW_NAME = "MERO_AI_ROBOT"
if not HEADLESS:
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE)

mode_str = "mm 모드" if MM_PER_PIXEL else "픽셀 모드 (캘리브 없음)"

# --cls 미지정 시: 카메라/모델 다 뜬 상태에서 경기 당일 타겟 클래스 2개를 직접 입력받음.
# Enter 누르는 순간이 곧 "경기 시작" 신호 — 이 직후 send_start()로 팔이 내려감.
# --storage-only는 어차피 SEARCHING을 안 타서 클래스 선택 자체가 무의미 — 입력 스킵.
if STORAGE_ONLY:
    TARGET_CLS = TARGET_CLS or set()
elif TARGET_CLS is None:
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
# 박스/오버레이 그리는 작업(results.plot() + 여러 cv2 draw 호출)을 예전엔 아무도
# 스트림을 안 보고 있어도 매 프레임 무조건 했음 — yolo_cam_test.py에서 검증한 것과
# 동일하게, 실제로 브라우저가 붙어있을 때만(+--record로 박스 그려진 걸 저장해야 할 때만)
# 그리도록 해서 아무도 안 볼 때 그 오버헤드를 통째로 스킵한다.
_stream_frame  = None
_stream_lock   = threading.Lock()
_stream_client_count = 0

class _MJPEGHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        global _stream_client_count
        self.send_response(200)
        self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
        self.end_headers()
        with _stream_lock:
            _stream_client_count += 1
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
        finally:
            with _stream_lock:
                _stream_client_count -= 1
    def log_message(self, *_):
        pass

threading.Thread(
    target=lambda: ThreadingHTTPServer(('0.0.0.0', 8080), _MJPEGHandler).serve_forever(),
    daemon=True
).start()

# ── 프레임 기록 (--record, 선택) ──────────────────────────
# 매 프레임 영상으로 인코딩(cv2.VideoWriter)하면 CPU 부하가 커서, 대신
# RECORD_INTERVAL_SECS 간격으로 프레임만 샘플링해 JPEG로 저장한다.
# 인코딩+디스크 쓰기는 백그라운드 스레드가 맡고, 큐가 밀리면(디스크가
# 못 따라오면) 새 프레임은 그냥 버려서 메인 루프가 절대 안 막히게 한다.
RECORD_INTERVAL_SECS = 0.2   # 저장 주기 (5fps) — 리뷰용으로 충분, 부하 최소화
_record_dir   = None
_record_queue = None
_record_idx   = 0

def _record_worker(q):
    while True:
        path, frame = q.get()
        try:
            cv2.imwrite(path, frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        except Exception as e:
            print(f"[녹화] 저장 실패: {e}")

if RECORD:
    _record_dir = os.path.join(BASE_DIR, "records", time.strftime("%Y%m%d_%H%M%S"))
    os.makedirs(_record_dir, exist_ok=True)
    _record_queue = queue.Queue(maxsize=30)
    threading.Thread(target=_record_worker, args=(_record_queue,), daemon=True).start()
    print(f"[녹화] 활성화 — {_record_dir} (5fps 샘플링, 부하 최소화)")

# ── 바퀴 명령/전압 로그 (배터리 전압 급강하 원인 진단용, 항상 켜짐) ──
# control_wheels()가 실제로 ESP32에 보내는 L/R과 그 순간의 battery_v를
# 매 호출마다 CSV로 남긴다. 언제/어떤 명령 직후에 전압이 뚝 떨어지는지
# 나중에 타임스탬프로 맞춰볼 수 있게 하기 위함 — line-buffered라 중간에
# 죽어도 그 직전까지는 파일에 남아있음.
_wheel_log_dir  = os.path.join(BASE_DIR, "logs")
os.makedirs(_wheel_log_dir, exist_ok=True)
_wheel_log_path = os.path.join(_wheel_log_dir, time.strftime("%Y%m%d_%H%M%S") + "_wheel.csv")
_wheel_log_f    = open(_wheel_log_path, "w", buffering=1, encoding="utf-8")
_wheel_log_f.write("timestamp,elapsed,state,L,R,battery_v\n")
print(f"[로그] 바퀴/전압 기록 → {_wheel_log_path}")

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
_last_print_t  = 0.0
_last_record_t = 0.0
frame = None  # 최초 루프 진입 전 초기화

if STORAGE_ONLY:
    # SEARCHING/GRIPPING 완전히 건너뛰고 시작하자마자 GO_TO_STORAGE 진입 —
    # 태극기 탐색+감지 즉시정지 로직만 단독으로 테스트할 때 사용.
    robot_state        = RobotState.GO_TO_STORAGE
    flag_arrived         = False
    storage_enter_time   = time.time()
    send_cam_backward()
    send_arm_up()
    print("[시작] --storage-only: GO_TO_STORAGE로 바로 진입 (카메라 후방 회전 + 팔 올림)")
    match_start_time = time.time()
else:
    # match_start_time은 "이 프로세스가 이 줄에 도달한 시각"일 뿐이라, 경기 도중
    # 카메라 hang 등으로 main.py를 재시작해야 하면 PICK_PHASE_SECS(150s) 카운트다운이
    # 재시작 시점부터 다시 시작돼서 실제 경기 시계랑 완전히 어긋난다(실제로 확인된
    # 문제 — 재시작 시점에 남은 실제 시간보다 소프트웨어가 더 많이 남았다고 착각해서
    # GO_TO_STORAGE 전환 시점을 영영 못 맞추는 경우가 생김). 그래서 시작할 때 "남은
    # 경기 시간"을 직접 입력받아 match_start_time을 그만큼 과거로 당겨서 보정한다.
    while True:
        raw_remain = input(f"[시작] 남은 경기 시간 입력 (MM:SS, 처음 시작이면 그냥 Enter = {int(MATCH_DURATION_SECS)//60}:{int(MATCH_DURATION_SECS)%60:02d}): ").strip()
        if raw_remain == "":
            remaining_secs = MATCH_DURATION_SECS
            break
        try:
            mm, ss = raw_remain.split(":")
            remaining_secs = int(mm) * 60 + int(ss)
            if 0 < remaining_secs <= MATCH_DURATION_SECS:
                break
            print(f"[오류] 0~{int(MATCH_DURATION_SECS)}초(0:00~{int(MATCH_DURATION_SECS)//60}:{int(MATCH_DURATION_SECS)%60:02d}) 사이로 입력하세요.")
        except ValueError:
            print("[오류] MM:SS 형식으로 입력하세요 (예: 1:38).")
    elapsed_already = MATCH_DURATION_SECS - remaining_secs
    send_start()  # 시작 크기 규정으로 올려둔 팔을 내림 (전진 시작과 함께)
    match_start_time = time.time() - elapsed_already  # 이미 지난 시간만큼 과거로 당겨서 보정
    if elapsed_already > 0:
        print(f"[시작] 남은 시간 {int(remaining_secs)//60}:{int(remaining_secs)%60:02d} 반영 — 이미 {elapsed_already:.0f}초 경과한 것으로 시작")

# ── 메인 루프 ────────────────────────────────────────────
try:
    while True:
        # 카메라 1대로 모든 상태(SEARCHING/GRIPPING/GO_TO_STORAGE)에서 동일하게 탐지
        # GO_TO_STORAGE는 아래에서 detected 중 cls=='flag'만 걸러서 씀
        # 캡처는 별도 스레드가 담당 — cap.read()가 잠깐 블로킹돼도(select() timeout
        # 등) 여기선 마지막으로 받은 프레임을 그대로 써서 control_wheels/send_idle이
        # 계속 불리게 한다 (ESP32 하트비트 워치독이 3초 무명령시 모터를 자동 정지시킴).
        with _cap_lock:
            frame      = _cap_latest_frame
            fail_count = _cap_fail_count

        if frame is None:
            time.sleep(0.01)  # 시작 직후 캡처 스레드의 첫 프레임 대기 (매우 짧음)
            continue

        if fail_count >= FRAME_FAIL_LIMIT:
            print(f"[오류] 프레임 읽기 연속 {FRAME_FAIL_LIMIT}회 실패 — 종료")
            break

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
            _write_esp32({"T": 126})
            _last_imu_req = _now_loop

        # ── 픽업 시간(2분30초) 마감 체크 — SEARCHING/GRIPPING/POST_GRIP_SCAN 중
        # 어느 상태에 있든(grip 응답 대기 중이든 집기후 스캔 중이든) 즉시 중단하고
        # 보관함 이동으로 전환. GRIPPING/POST_GRIP_SCAN에서도 걸리게 해야 최악의 경우
        # (grip 타임아웃 15초 + 스캔 4초)에도 남은 30초를 거의 다 까먹지 않는다.
        if (robot_state in (RobotState.SEARCHING, RobotState.GRIPPING, RobotState.POST_GRIP_SCAN)
                and time.time() - match_start_time >= PICK_PHASE_SECS):
            control_wheels(None)
            if gripper_prepped:
                send_gripper_close()
                gripper_prepped = False
            precise_align        = False
            align_phase          = 0
            align_final_forward  = False
            gripper_open_wait    = False
            fb_final_forward     = False
            robot_state          = RobotState.GO_TO_STORAGE
            flag_arrived         = False
            storage_enter_time   = time.time()
            send_cam_backward()   # 경기당 1회 — 이후 다시 정면으로 돌릴 일 없음
            send_arm_up()         # 카메라 회전과 동시에 팔도 규정 크기 위치로 복귀
            print(f"\n[상태] 픽업 시간 종료({PICK_PHASE_SECS:.0f}s) → GO_TO_STORAGE 전환")

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

            elif gripper_open_wait:
                # gripper_open 명령 보낸 직후 — 응답 확인은 안 하고, 그리퍼가 실제로 열리는
                # 물리 시간(GRIPPER_OPEN_LEAD_SECS)만큼만 정지한 채 기다렸다가 직진 시작.
                control_wheels(None)
                if time.time() - gripper_open_wait_start >= GRIPPER_OPEN_LEAD_SECS:
                    gripper_open_wait      = False
                    fb_final_forward       = True
                    fb_final_forward_start = time.time()
                    fb_final_forward_cls   = gripper_open_wait_cls
                    print(f"\n[상태] 그리퍼 열림 대기 완료 → 직진 접근 시작")

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
                # 실제 grip 정밀 정렬: 전진/후진(상하 cy)과 회전(좌우 cx)을 동시에 섞어서 조향
                # (예전엔 cy 먼저 맞추고 나서 cx를 맞추는 순차 방식이었는데, 둘 다 한 번에
                # 보정하도록 변경 — L/R에 전후진 성분과 회전 성분을 더해서 같이 움직인다).
                # select_target()을 매 프레임 다시 부르지 않고 last_target_id로 같은 물체만 계속
                # 추적한다 — 후보가 여러 개고 점수가 비슷하면 프레임마다 다른 물체로 선택이 튈 수
                # 있어서, 한 번 정하면 그 물체가 완전히 사라지기(+grace) 전까진 안 바꾼다.
                locked = next((o for o in detected if o["id"] == last_target_id), None)
                if not locked:
                    target_miss_count += 1
                    if target_miss_count <= TARGET_MISS_GRACE_FRAMES:
                        # 모션블러 등으로 순간적으로 놓친 것일 수 있음 — 몇 프레임은 정지하고 재등장을 기다린다
                        control_wheels(None)
                        print(f"[상태] 정밀 정렬 중 순간 놓침 ({target_miss_count}/{TARGET_MISS_GRACE_FRAMES}) — 정지 대기", end="\r")
                    else:
                        precise_align     = False
                        last_target_id    = -1
                        target_miss_count = 0
                        if gripper_prepped:
                            send_gripper_close()
                            gripper_prepped = False
                        print("\n[상태] 정밀 정렬 중 타겟 놓침 → 재탐색 (그리퍼 닫음)")
                else:
                    target_miss_count = 0
                    frame_w = FRAME_W or 640
                    frame_h = FRAME_H or 480
                    cx_ref  = frame_w / 2 + CENTER_OFFSET_X_PX
                    cy_ref  = frame_h / 2 + CENTER_OFFSET_Y_PX
                    cx_aligned = abs(locked["cx"] - cx_ref) <= CENTER_MARGIN_PX
                    cy_aligned = abs(locked["cy"] - cy_ref) <= CENTER_MARGIN_Y_PX

                    if cx_aligned and cy_aligned:
                        control_wheels(None)
                        precise_align           = False
                        gripper_open_wait       = True
                        gripper_open_wait_start = time.time()
                        gripper_open_wait_cls   = locked["cls"]
                        send_gripper_open()
                        gripper_prepped = True
                        print(f"\n[상태] 정렬 완료 (cx={locked['cx']:.0f}, cy={locked['cy']:.0f}) → 그리퍼 열기 ({GRIPPER_OPEN_LEAD_SECS:.1f}s 대기 후 직진)")
                    else:
                        # cy_ref보다 위(작음)=목표가 더 멀리 있음 → 전진, 아래(큼)=너무 가까움 → 후진
                        fwd = 0.0
                        if not cy_aligned:
                            fwd = PRECISE_ALIGN_FB_SPEED if locked["cy"] < cy_ref else -PRECISE_ALIGN_FB_SPEED
                        turn = 0.0
                        if not cx_aligned:
                            turn = max(-1.0, min(1.0, (locked["cx"] - cx_ref) / (frame_w / 2)))
                        control_wheels(None,
                                       override_l=fwd + PRECISE_ALIGN_TURN_SPEED * turn,
                                       override_r=fwd - PRECISE_ALIGN_TURN_SPEED * turn)
                        print(f"[상태] 동시 정렬중... cy={locked['cy']:.0f} cx={locked['cx']:.0f}", end="\r")

            elif target:
                search_rotate_start = None
                if time.time() - _last_print_t >= 0.5:
                    print(f"[타겟] {target['cls']} | area={target['area']}")
                    _last_print_t = time.time()

                # 타겟이 보이면 area/중앙정렬 상관없이 바로 정밀 정렬(전후진+회전 동시 보정) 진입 —
                # 예전 --align-fwd-first와 동일한 방식 (거리 무관하게 즉시 시작)
                # 그리퍼는 정렬 중엔 닫힌 채로 두고, 좌우 정렬까지 끝나 최종 직진 접근
                # 직전에만 연다 (엉뚱한 물체가 정렬 중 벌어진 집게에 끼는 것 방지).
                control_wheels(None)
                precise_align     = True
                target_miss_count = 0  # 이전 정렬 시도가 grace 소진 없이 중간에 끊겼을 수 있어 새로 시작할 때 항상 리셋
                last_target_id    = target["id"]  # 이 물체 id로 락 — 이후 select_target() 재호출 없이 이 id만 추적
                print(f"\n[상태] 타겟 발견 (area={target['area']}) → 정밀 정렬 시작 (그리퍼는 닫힌 채 유지)")

            else:
                align_phase   = 0
                precise_align = False

                # 타겟 미검출 — 클래스/신뢰도 상관없이(flag는 제외) 탐지된 물체가 2개 이상이면
                # 물체가 가장 많이 몰려있는 방향(밀집도 가중 중심)으로 이동하며 탐색한다.
                # 고립된 물체 하나 쪽으로 잘못 쏠리지 않도록, 각 물체의 조향 기여도를
                # "주변 CLUSTER_RADIUS_PX 이내 이웃 개수+1"로 가중해서 평균낸다.
                # 1개 이하면(=밀집도 비교 불가) 제자리 회전만 계속한다 — 안 보이는 방향으로
                # 무작정 전진하면 벽에 부딪힐 수 있어서 전진은 절대 안 함. 뭔가 보일 때까지 회전.
                explorable = [o for o in detected if o['cls'] != 'flag']
                can_explore = len(explorable) >= MIN_DETECTED_FOR_EXPLORE

                if can_explore:
                    def _neighbor_count(o):
                        return sum(
                            1 for other in explorable
                            if other is not o and ((other['cx'] - o['cx']) ** 2 + (other['cy'] - o['cy']) ** 2) ** 0.5 <= CLUSTER_RADIUS_PX
                        )

                    weights     = [_neighbor_count(o) + 1 for o in explorable]
                    weight_sum  = sum(weights)
                    dense_cx    = sum(o["cx"] * w for o, w in zip(explorable, weights)) / weight_sum
                    dense_area  = sum(o["area"] * w for o, w in zip(explorable, weights)) / weight_sum

                    control_wheels({"cx": dense_cx, "area": dense_area})
                    search_rotate_start = None
                    if time.time() - _last_print_t >= 0.5:
                        print(f"[탐색] 물체 {len(explorable)}개 감지, 밀집 방향(cx={dense_cx:.0f}) 으로 이동")
                        _last_print_t = time.time()

                else:
                    if search_rotate_start is None:
                        search_rotate_start = time.time()
                    control_wheels(None, override_l=-SEARCH_ROTATE_SPEED, override_r=SEARCH_ROTATE_SPEED)
                    print(f"[탐색] 제자리 회전중... ({time.time() - search_rotate_start:.1f}s)", end="\r")

                send_idle()

        elif robot_state == RobotState.GRIPPING:
            control_wheels(None)
            elapsed = time.time() - grip_sent_at
            if openrb_gripped:
                openrb_gripped       = False
                openrb_grip_failed   = False
                gripped_cls          = None
                confirm_count        = 0
                last_target_id       = -1
                robot_state          = RobotState.POST_GRIP_SCAN
                post_grip_scan_start = time.time()
                print(f"[상태] GRIPPING → POST_GRIP_SCAN ({elapsed:.1f}s)")
            elif openrb_grip_failed:
                openrb_grip_failed   = False
                openrb_gripped       = False
                confirm_count        = 0
                last_target_id       = -1
                robot_state          = RobotState.POST_GRIP_SCAN
                post_grip_scan_start = time.time()
                print(f"\n[상태] GRIPPING → POST_GRIP_SCAN (집기 실패)")
            elif elapsed > GRIP_TIMEOUT_SECS:
                print(f"\n[경고] grip 타임아웃 → POST_GRIP_SCAN")
                confirm_count        = 0
                last_target_id       = -1
                robot_state          = RobotState.POST_GRIP_SCAN
                post_grip_scan_start = time.time()
            else:
                print(f"[상태] 집어서 컨테이너 투하중... ({elapsed:.1f}s)", end="\r")

        elif robot_state == RobotState.POST_GRIP_SCAN:
            # 집기 시도(성공/실패 무관) 직후 — 먼저 POST_GRIP_BACKUP_SECS(1초) 동안 뒤로
            # 후진한 다음, 제자리에서 한 바퀴 돌며(POST_GRIP_SCAN_SECS) 주변에 바로 이어서
            # 집을 만한 타겟이 있는지 확인한다. 발견하거나 스캔 시간이 다 차면 SEARCHING으로
            # 넘겨서 이후 정밀 정렬/탐색은 기존 로직이 그대로 처리한다.
            elapsed_scan = time.time() - post_grip_scan_start
            if target:
                control_wheels(None)
                robot_state = RobotState.SEARCHING
                print(f"\n[상태] 스캔 중 타겟 발견 ({target['cls']}) → SEARCHING 복귀")
            elif elapsed_scan < POST_GRIP_BACKUP_SECS:
                control_wheels(None, override_l=-POST_GRIP_BACKUP_SPEED, override_r=-POST_GRIP_BACKUP_SPEED)
                print(f"[상태] 집기 후 후진중... ({elapsed_scan:.1f}/{POST_GRIP_BACKUP_SECS:.1f}s)", end="\r")
            elif elapsed_scan >= POST_GRIP_BACKUP_SECS + POST_GRIP_SCAN_SECS:
                control_wheels(None)
                robot_state = RobotState.SEARCHING
                print(f"\n[상태] 주변 스캔 완료 (새 타겟 없음) → SEARCHING 복귀")
            else:
                scan_elapsed = elapsed_scan - POST_GRIP_BACKUP_SECS
                control_wheels(None, override_l=-SEARCH_ROTATE_SPEED, override_r=SEARCH_ROTATE_SPEED)
                print(f"[상태] 집기 후 주변 스캔중... ({scan_elapsed:.1f}/{POST_GRIP_SCAN_SECS:.1f}s)", end="\r")
            send_idle()

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

            elif flag_arrived:
                # 이미 감지되어 정지 완료 — 더 이상 아무 것도 안 함(경기 종료 취급)
                control_wheels(None)

            else:
                # 태극기 탐색 — 제자리 회전하다 1프레임이라도 감지되면 정렬/접근 없이 즉시 정지.
                flag_candidates = [o for o in detected if o['cls'] == 'flag' and o['conf'] >= FLAG_CONF_THRESHOLD]
                if flag_candidates:
                    control_wheels(None)
                    flag_arrived = True
                    print(f"\n[상태] 태극기 감지 → 정지 (경기 종료 취급)")
                else:
                    control_wheels(None, override_l=FLAG_SEARCH_SPEED, override_r=-FLAG_SEARCH_SPEED)
                    print(f"[상태] 태극기 탐색 회전중... ({total_elapsed:.1f}s)", end="\r")

        if frame is None:
            continue

        # ── FPS 카운트 — 그리기(시각화) 여부와 무관하게 항상 집계 ──
        fps_counter += 1
        elapsed_fps = time.time() - fps_timer
        if elapsed_fps >= 1.0:
            fps_display = fps_counter / elapsed_fps
            fps_counter = 0
            fps_timer   = time.time()
            print(f"[FPS] {fps_display:.1f}")

        # ── 시각화 ──────────────────────────────────────
        # results.plot() + 여러 cv2 draw 호출은 공짜가 아님 — 아무도 브라우저 스트림을
        # 안 보고 있고 --record(원본 아닌 박스버전)도 아니면 이 블록 자체를 통째로 스킵.
        with _stream_lock:
            _watching = _stream_client_count > 0
        if _watching or (RECORD and not RECORD_RAW) or not HEADLESS:
            annotated_frame = results[0].plot() if results is not None else frame.copy()

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

            # 타겟 노란 테두리 — 정밀 정렬 중엔 실제로 추적 중인 last_target_id를 표시
            # (그 순간 select_target()이 고르는 것과 다를 수 있어서 혼동 방지)
            _highlight_id = last_target_id if (precise_align or gripper_open_wait or fb_final_forward) else (target["id"] if target else None)
            if _highlight_id is not None and boxes is not None:
                ids = boxes.id
                for i, box in enumerate(boxes):
                    if (int(ids[i]) if ids is not None else -1) == _highlight_id:
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
            if SHOW_TIMER:
                remaining = max(0.0, MATCH_DURATION_SECS - (time.time() - match_start_time))
                mins = int(remaining // 60)
                secs = int(remaining % 60)
                tc = (0, 0, 255) if remaining < 30 else (0, 165, 255) if remaining < 60 else (0, 255, 255)
                cv2.putText(annotated_frame, f"{mins}:{secs:02d}",
                            (w // 2 - 25, 35), cv2.FONT_HERSHEY_SIMPLEX, 1.0, tc, 2)

            cv2.putText(annotated_frame, f"FPS: {fps_display:.1f}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

            with _stream_lock:
                _stream_frame = annotated_frame.copy()

        if RECORD and time.time() - _last_record_t >= RECORD_INTERVAL_SECS:
            _last_record_t = time.time()
            _record_idx += 1
            try:
                _record_queue.put_nowait((
                    os.path.join(_record_dir, f"frame_{_record_idx:06d}.jpg"),
                    frame if RECORD_RAW else annotated_frame,
                ))
            except queue.Full:
                pass  # 디스크가 못 따라오면 그냥 이번 프레임은 버림 (메인 루프 안 막음)

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
