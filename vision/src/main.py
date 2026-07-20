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
  PATH_NAV       — --path-test 시 진입하는 그리드 기반 고정 경로 주행 (실험용, 벽 감지/회전시간 미보정)
                   → 직진1/3(벽 탐색 구간)에서 목표 물건 감지 시 SEARCHING으로 전환해 잡음
                   → 경로 완료 시 GO_TO_STORAGE로 전환 (후면 카메라로 태극기 인식 후 접근 → dump)
  PATH_RETURN    — SEARCHING 중 이동/회전한 시간만큼 반대로 움직여 원위치 복귀 후 PATH_NAV 재개

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
                    help='타겟 클래스 목록 (예: --cls d8 apple). 미지정 시 모든 클래스 대상')
parser.add_argument('--timer', action='store_true',
                    help='3분 경기 타이머 표시')
parser.add_argument('--test', action='store_true',
                    help='테스트 모드: 집으면 1초 직진 후 바로 drop')
parser.add_argument('--align-only', action='store_true',
                    help='정렬 테스트용: 중앙정렬+area 도달 시 정지만 하고 직진/grip 생략')
parser.add_argument('--no-wheels', action='store_true',
                    help='바퀴 명령을 ESP32로 보내지 않음 (탐지/그리퍼만 테스트할 때)')
parser.add_argument('--align-fwd-first', action='store_true',
                    help='정렬 순서 테스트용 (참고 보관): --align-only와 반대로 전진/후진(상하 cy) 먼저 → 회전(좌우 cx) 나중. '
                         '실제 grip 로직은 이 순서를 이미 기본값으로 사용함 (SEARCHING의 precise_align)')
parser.add_argument('--path-test', action='store_true',
                    help='경로 주행 테스트 모드: SEARCHING 대신 PATH_NAV로 시작 (그리드 기반 고정 경로, 실험용)')
args       = parser.parse_args()
TARGET_CLS    = set(args.cls) if args.cls else None
TEST_MODE     = args.test
ALIGN_ONLY      = args.align_only
NO_WHEELS       = args.no_wheels
ALIGN_FWD_FIRST = args.align_fwd_first
PATH_TEST_MODE  = args.path_test
SHAPE_CLASSES = {'d6', 'd8', 'd12', 'd20'}
FRUIT_CLASSES = {'apple', 'banana', 'orange', 'pineapple'}

# ── 모델 로드 ────────────────────────────────────────────
BASE_DIR        = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODEL_PATH      = os.path.join(BASE_DIR, "model", "best.pt")
FLAG_MODEL_PATH = os.path.join(BASE_DIR, "model", "flag.pt")
# Jetson TensorRT 변환 후: MODEL_PATH = os.path.join(BASE_DIR, "model", "best.engine")
model      = YOLO(MODEL_PATH)
if os.path.exists(FLAG_MODEL_PATH):
    flag_model = YOLO(FLAG_MODEL_PATH)
    print(f"[모델] flag.pt 로드 완료")
else:
    flag_model = None
    print(f"[모델] flag.pt 없음 — GO_TO_STORAGE 태극기 감지 비활성화")

# ── 카메라 인덱스 자동 감지 ──────────────────────────────
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

CAMERA_INDEX_OBJ  = _find_camera_index(["arducam"], 2)             # 물체 카메라 (전면)
CAMERA_INDEX_FLAG = _find_camera_index(["nv76", "cm400"], 0)       # 태극기 카메라 (후면)

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
AREA_SIMILAR_TOLERANCE = 3000  # 최대 area와 이 이내로 차이나면 "비슷한 크기"로 보고 오른쪽(cx 큰 것) 우선

def select_target(objects: list) -> dict | None:
    """--cls 필터 + 클래스별 confidence 임계값 통과한 것 중 area가 큰 것 우선.
    area가 최대값과 AREA_SIMILAR_TOLERANCE 이내로 비슷한 후보가 여럿이면 화면 오른쪽(cx 큰 것) 우선."""
    if not objects:
        return None
    filtered = []
    for o in objects:
        if TARGET_CLS and o['cls'] not in TARGET_CLS:
            continue
        threshold = CONF_THRESHOLD_FRUIT if o['cls'] in FRUIT_CLASSES else CONF_THRESHOLD_SHAPE
        if o['conf'] >= threshold:
            filtered.append(o)
    if not filtered:
        return None
    max_area   = max(o['area'] for o in filtered)
    candidates = [o for o in filtered if max_area - o['area'] <= AREA_SIMILAR_TOLERANCE]
    return max(candidates, key=lambda o: o['cx'])


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

# ── 경로 주행 파라미터 (--path-test, 그리드 기반 고정 경로 실험용) ──
# 가상 좌표계: 가로 7칸 × 세로 6칸, 간격 50cm, 왼쪽 아래 = (1,1)
# 경로: 시작점에서 1m 직진 → 우회전
#       → (5,1)-(6,1) 사이 통과하며 직진 → 벽 근접 시 좌회전
#       → 직진해서 (2,6)-(3,6) 사이 도달 → 좌회전
#       → 직진 → 벽 근접 시 좌회전
#       → 완료 시 GO_TO_STORAGE 전환 (후면 카메라로 태극기 인식 → 접근 → dump)
# 회전시간(PATH_TURN_90_SECS)/직진시간은 실측값(2026-07-21) 반영. 벽 감지(is_near_wall)는
# 아직 자리만 잡아둔 상태 — YOLO 벽 감지 모델이 붙기 전까지는 정확히 동작하지 않음
GRID_SPACING_M       = 0.5    # 그리드 한 칸 간격 (참고용)
PATH_FORWARD_SPEED   = MOVE_SPEED
PATH_TURN_SPEED      = 0.2
PATH_SECS_PER_METER  = 4.0    # 실측: 1m 직진에 약 4초 소요 (PATH_FORWARD_SPEED 기준)
PATH_TURN_90_SECS    = 1.5    # 실측: 제자리 90도 회전에 약 1.5초 소요
# 좌회전 시 L/R 부호. 실제 로봇에서 좌회전 방향이 맞는지 확인 후 반대면 부호 스왑
PATH_LEFT_L          = -PATH_TURN_SPEED
PATH_LEFT_R          =  PATH_TURN_SPEED
# 우회전 시 L/R 부호 — 좌회전의 반대 부호. 마찬가지로 실제 방향 확인 필요
PATH_RIGHT_L         =  PATH_TURN_SPEED
PATH_RIGHT_R         = -PATH_TURN_SPEED
# (5,1)-(6,1) → (2,6)-(3,6) 구간: 가로 3칸 = 1.5m 이동
PATH_TO_POINT_SECS   = PATH_SECS_PER_METER * (GRID_SPACING_M * 3)
# 시작점에서 1m 직진 구간
PATH_START_FORWARD_SECS = PATH_SECS_PER_METER * 1.0
# TODO: is_near_wall()에 YOLO 벽 감지 모델 연동 전까지 쓰는 안전 타임아웃
PATH_WALL_TIMEOUT_SECS = 20.0

# 직진1/직진3(path_phase 2, 6) 중 목표 물건을 발견해 SEARCHING으로 빠졌다가 grip 후
# 원위치로 되돌아올 때 쓰는 상태 (PATH_RETURN). 후진 이동 속도는 PATH_FORWARD_SPEED 재사용
PATH_INTERRUPT_PHASES = (2, 6)

# ── 상태 머신 ────────────────────────────────────────────
class RobotState(Enum):
    SEARCHING     = "SEARCHING"
    GRIPPING      = "GRIPPING"
    GO_TO_STORAGE = "GO_TO_STORAGE"
    DROPPING      = "DROPPING"
    PATH_NAV      = "PATH_NAV"
    PATH_RETURN   = "PATH_RETURN"

robot_state         = RobotState.PATH_NAV if PATH_TEST_MODE else RobotState.SEARCHING
grip_sent_at        = 0.0
drop_sent_at        = 0.0
_frame_fail_count   = 0
storage_phase       = 0   # 0=탐색회전, 1=후진접근
storage_phase_start = 0.0
storage_enter_time  = 0.0
confirm_count       = 0
last_target_id      = -1
gripped_cls         = None
final_approach       = False  # True면 정지 후 직진 접근 중 (area 임계 도달~grip 전송 사이)
final_approach_start = 0.0
final_approach_cls   = None
align_phase          = 0      # --align-only 전용: 0=회전으로 좌우(cx) 정렬, 1=전진/후진으로 상하(cy) 정렬
align_final_forward       = False  # --align-only 전용: cy 정렬 완료 후 1초 직진 중
align_final_forward_start = 0.0
align_final_forward_cls   = None
fb_phase          = 0      # --align-fwd-first 전용 (참고 보관): 0=전진/후진으로 상하(cy) 정렬, 1=회전으로 좌우(cx) 정렬
fb_final_forward       = False  # --align-fwd-first 전용: cx 정렬 완료 후 직진 중
fb_final_forward_start = 0.0
fb_final_forward_cls   = None
approach_phase = 0      # 실제 grip 정렬: 0=전진/후진으로 상하(cy) 정렬, 1=회전으로 좌우(cx) 정렬
precise_align  = False  # True면 area 임계 도달 후 정밀 정렬(전진/후진→회전) 진행 중

# PATH_NAV: 0=시작직진(1m) 1=우회전 2=직진1(벽까지) 3=좌회전1 4=직진2(지점까지)
#           5=좌회전2 6=직진3(벽까지) 7=좌회전3 8=완료
path_phase        = 0
path_phase_start  = time.time() if PATH_TEST_MODE else 0.0
path_resume_phase = None  # 직진1/3 중 목표 물건 발견 시 중단된 path_phase 저장 (원위치 복귀 후 재개용)

# SEARCHING으로 빠져 있는 동안(path_resume_phase가 설정된 구간) 누적한 이동/회전 시간.
# PATH_RETURN에서 이 시간만큼 반대로 움직여 원위치 근처로 되돌아오는 데 사용
search_move_secs        = 0.0
search_turn_secs_signed = 0.0  # 부호: +면 우회전(PATH_RIGHT) 방향 우세, -면 좌회전(PATH_LEFT) 방향 우세

# PATH_RETURN: 0=후진(search_move_secs만큼) 1=반대 방향 회전(search_turn_secs만큼)
path_return_stage       = 0
path_return_stage_start = 0.0


def _after_grip_return_state():
    """GRIPPING 종료 후 돌아갈 상태. PATH_NAV 이동 중 grip으로 빠진 경우면 PATH_RETURN(원위치 복귀)으로,
    아니면 SEARCHING으로."""
    global path_return_stage, path_return_stage_start
    if path_resume_phase is not None:
        path_return_stage       = 0
        path_return_stage_start = time.time()
        return RobotState.PATH_RETURN, path_resume_phase
    return RobotState.SEARCHING, None


def _track_search_motion(L, R, dt):
    """PATH_NAV 중단 중(path_resume_phase 설정 상태)에만 이동/회전 시간을 누적."""
    global search_move_secs, search_turn_secs_signed
    if path_resume_phase is None or dt <= 0 or (L == 0.0 and R == 0.0):
        return
    if (L > 0) == (R > 0):
        search_move_secs += dt
    else:
        # L>0,R<0 → PATH_RIGHT(우회전) 방향, L<0,R>0 → PATH_LEFT(좌회전) 방향
        search_turn_secs_signed += dt if L > 0 else -dt

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
    if NO_WHEELS or ser_esp32 is None or not ser_esp32.is_open:
        return 0.0, 0.0

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
    return L, R


def _is_at_target(target: dict) -> bool:
    """도달(=정밀 정렬 단계 진입) 여부 판단. mm 모드 → 거리, 픽셀 모드 → area 임계 도달.
    중심 정렬(cx/cy)은 여기서 보지 않고, 도달 이후 정밀 정렬 단계(전진/후진→회전)에서 별도로 맞춘다."""
    if target.get("mx") is not None:
        dist = (target["mx"] ** 2 + target["my"] ** 2) ** 0.5
        return dist < ARRIVE_THRESHOLD_MM
    return target.get("area", 0) >= AREA_GRIP_THRESHOLD


def is_near_wall(frame) -> bool:
    """벽 근접 여부 판단. TODO: YOLO 벽 감지 모델(wall.pt 등) 연동 예정 — 현재는 항상 False."""
    return False


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
_last_frame_t = time.time()  # 이동/회전 시간 누적용 dt 기준
frame = None  # 최초 루프 진입 전 초기화

send_start()  # 시작 크기 규정으로 올려둔 팔을 내림 (전진 시작과 함께)

# ── 메인 루프 ────────────────────────────────────────────
try:
    while True:
        # GO_TO_STORAGE 중에는 cap2+flag_model 사용 (GO_TO_STORAGE 블록 내부에서 처리)
        # 그 외 상태는 cap1+model로 물체 탐지
        if robot_state == RobotState.GO_TO_STORAGE:
            cap.read()  # 버퍼 비우기만
            results   = None
            boxes     = None
            detected  = []
            target    = None
            at_target = False
        else:
            # PATH_NAV도 여기로 들어옴 — 이동 중 목표 물건 감지를 위해 물체 탐지 계속 수행
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

        _frame_now    = time.time()
        _frame_dt     = max(0.0, _frame_now - _last_frame_t)
        _last_frame_t = _frame_now

        # ── IMU 주기 요청 (GO_TO_STORAGE, PATH_NAV, PATH_RETURN 제외) ──
        _now_loop = time.time()
        if robot_state not in (RobotState.GO_TO_STORAGE, RobotState.PATH_NAV, RobotState.PATH_RETURN) and _now_loop - _last_imu_req >= 0.15:
            if ser_esp32 and ser_esp32.is_open:
                ser_esp32.write((json.dumps({"T": 126}) + "\n").encode())
            _last_imu_req = _now_loop

        # ── 상태 머신 ──────────────────────────────────
        if robot_state == RobotState.SEARCHING:
            if final_approach:
                # area 임계 도달 직후 — 정지 상태 유지하며 정면으로 직진
                L, R = control_wheels(None, override_l=FINAL_APPROACH_SPEED - FORWARD_TRIM / 2, override_r=FINAL_APPROACH_SPEED + FORWARD_TRIM / 2)
                _track_search_motion(L, R, _frame_dt)
                elapsed_fa = time.time() - final_approach_start
                print(f"[상태] 직진 접근중... ({elapsed_fa:.1f}s)", end="\r")
                if elapsed_fa >= FINAL_APPROACH_SECS:
                    control_wheels(None)
                    final_approach = False
                    last_target_id = -1
                    gripped_cls    = final_approach_cls
                    openrb_gripped     = False
                    openrb_dumped      = False
                    openrb_grip_failed = False
                    send_grip({"cls": final_approach_cls})
                    print(f"\n[상태] grip 전송 ({final_approach_cls})")
                    if TEST_MODE:
                        print("[테스트] 1회성 테스트 — grip 전송 후 종료")
                        break
                    robot_state  = RobotState.GRIPPING
                    grip_sent_at = time.time()
                    print(f"[상태] SEARCHING → GRIPPING (grip: {final_approach_cls})")

            elif align_final_forward:
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
                # cx 정렬 완료 후 1초 직진 → grip 전송 → GRIPPING (완료되면 다시 SEARCHING으로 반복, --align-fwd-first 참고용)
                control_wheels(None, override_l=FINAL_APPROACH_SPEED - FORWARD_TRIM / 2, override_r=FINAL_APPROACH_SPEED + FORWARD_TRIM / 2)
                elapsed_fb = time.time() - fb_final_forward_start
                print(f"[테스트] 직진중... ({elapsed_fb:.1f}s)", end="\r")
                if elapsed_fb >= FINAL_APPROACH_SECS:
                    control_wheels(None)
                    fb_final_forward   = False
                    last_target_id     = -1
                    gripped_cls        = fb_final_forward_cls
                    openrb_gripped     = False
                    openrb_dumped      = False
                    openrb_grip_failed = False
                    send_grip({"cls": fb_final_forward_cls})
                    print(f"\n[테스트] grip 전송 ({fb_final_forward_cls})")
                    robot_state  = RobotState.GRIPPING
                    grip_sent_at = time.time()
                    print(f"[상태] SEARCHING → GRIPPING (grip: {fb_final_forward_cls})")

            elif target and ALIGN_FWD_FIRST:
                # 방향 검증 전용 (순서 반대, 참고 보관): 1단계 전진/후진(상하 cy) → 2단계 제자리 회전(좌우 cx)
                # 실제 grip 로직(precise_align)이 이미 이 순서를 기본값으로 사용 중
                frame_w = FRAME_W or 640
                frame_h = FRAME_H or 480
                cx_ref  = frame_w / 2 + CENTER_OFFSET_X_PX
                cy_ref  = frame_h / 2 + CENTER_OFFSET_Y_PX
                cx_aligned = abs(target["cx"] - cx_ref) <= CENTER_MARGIN_PX
                cy_aligned = abs(target["cy"] - cy_ref) <= CENTER_MARGIN_Y_PX

                if fb_phase == 0:
                    if cy_aligned:
                        control_wheels(None)
                        print(f"\n[테스트] 상하 정렬 완료 (cy={target['cy']:.0f}) → 1초 대기 후 회전 정렬 시작")
                        time.sleep(1.0)
                        fb_phase = 1
                    else:
                        # cy_ref보다 위(작음)=목표가 더 멀리 있음 → 전진, 아래(큼)=너무 가까움 → 후진
                        fwd = SLOW_SPEED if target["cy"] < cy_ref else -SLOW_SPEED
                        control_wheels(None, override_l=fwd, override_r=fwd)
                        direction = "전진" if fwd > 0 else "후진"
                        print(f"[테스트] {direction} 정렬중... cy={target['cy']:.0f}", end="\r")

                else:
                    if not cy_aligned:
                        # 회전 중 상하가 틀어지면 전후 단계로 복귀
                        fb_phase = 0
                    elif cx_aligned:
                        control_wheels(None)
                        fb_phase                = 0
                        fb_final_forward        = True
                        fb_final_forward_start  = time.time()
                        fb_final_forward_cls    = target["cls"]
                        print(f"\n[테스트] 좌우 정렬 완료 (cx={target['cx']:.0f}) → 1초 직진")
                    else:
                        turn = max(-1.0, min(1.0, (target["cx"] - cx_ref) / (frame_w / 2)))
                        control_wheels(None, override_l=TURN_ONLY_SPEED * turn, override_r=-TURN_ONLY_SPEED * turn)
                        print(f"[테스트] 회전 정렬중... cx={target['cx']:.0f}", end="\r")

            elif precise_align:
                # 실제 grip 정렬: area 임계 도달 후 1단계 전진/후진(상하 cy) → 2단계 제자리 회전(좌우 cx)
                if not target:
                    # 정밀 정렬 중 타겟을 놓침 — 코스 탐색으로 복귀
                    precise_align  = False
                    approach_phase = 0
                    print("\n[상태] 정밀 정렬 중 타겟 놓침 → 재탐색")
                else:
                    frame_w = FRAME_W or 640
                    frame_h = FRAME_H or 480
                    cx_ref  = frame_w / 2 + CENTER_OFFSET_X_PX
                    cy_ref  = frame_h / 2 + CENTER_OFFSET_Y_PX
                    cx_aligned = abs(target["cx"] - cx_ref) <= CENTER_MARGIN_PX
                    cy_aligned = abs(target["cy"] - cy_ref) <= CENTER_MARGIN_Y_PX

                    if approach_phase == 0:
                        if cy_aligned:
                            control_wheels(None)
                            approach_phase = 1
                            print(f"\n[상태] 상하 정렬 완료 (cy={target['cy']:.0f}) → 좌우 정렬")
                        else:
                            # cy_ref보다 위(작음)=목표가 더 멀리 있음 → 전진, 아래(큼)=너무 가까움 → 후진
                            fwd = SLOW_SPEED if target["cy"] < cy_ref else -SLOW_SPEED
                            L, R = control_wheels(None, override_l=fwd, override_r=fwd)
                            _track_search_motion(L, R, _frame_dt)
                            direction = "전진" if fwd > 0 else "후진"
                            print(f"[상태] {direction} 정렬중... cy={target['cy']:.0f}", end="\r")

                    else:
                        if not cy_aligned:
                            # 회전 중 상하가 틀어지면 전후 단계로 복귀
                            approach_phase = 0
                        elif cx_aligned:
                            control_wheels(None)
                            precise_align        = False
                            approach_phase       = 0
                            final_approach       = True
                            final_approach_start = time.time()
                            final_approach_cls   = target["cls"]
                            print(f"\n[상태] 좌우 정렬 완료 (cx={target['cx']:.0f}) → 직진 접근 시작")
                        else:
                            turn = max(-1.0, min(1.0, (target["cx"] - cx_ref) / (frame_w / 2)))
                            L, R = control_wheels(None, override_l=TURN_ONLY_SPEED * turn, override_r=-TURN_ONLY_SPEED * turn)
                            _track_search_motion(L, R, _frame_dt)
                            print(f"[상태] 회전 정렬중... cx={target['cx']:.0f}", end="\r")

            elif target:
                if time.time() - _last_print_t >= 0.5:
                    print(f"[타겟] {target['cls']} | area={target['area']}")
                    _last_print_t = time.time()

                if at_target:
                    control_wheels(None)
                    # area 임계 최초 도달 — 정밀 정렬(전진/후진→회전) 단계 진입
                    precise_align  = True
                    approach_phase = 0
                    print(f"\n[상태] 목표 크기 도달 (area={target['area']}) → 정밀 정렬 시작")
                else:
                    L, R = control_wheels(target)
                    _track_search_motion(L, R, _frame_dt)

            else:
                align_phase    = 0
                fb_phase       = 0
                approach_phase = 0
                precise_align  = False
                L, R = control_wheels(None, override_l=-SEARCH_ROTATE_SPEED, override_r=SEARCH_ROTATE_SPEED)
                _track_search_motion(L, R, _frame_dt)
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
                robot_state, _resumed = _after_grip_return_state()
                if _resumed is not None:
                    print(f"[상태] GRIPPING → PATH_RETURN ({elapsed:.1f}s, 복귀 후 phase={_resumed} 재개)")
                else:
                    print(f"[상태] GRIPPING → SEARCHING ({elapsed:.1f}s)")
            elif openrb_grip_failed:
                openrb_grip_failed = False
                openrb_gripped     = False
                openrb_dumped      = False
                confirm_count      = 0
                last_target_id     = -1
                robot_state, _resumed = _after_grip_return_state()
                if _resumed is not None:
                    print(f"\n[상태] GRIPPING → PATH_RETURN (집기 실패, 복귀 후 phase={_resumed} 재개)")
                else:
                    print(f"\n[상태] GRIPPING → SEARCHING (집기 실패)")
            elif elapsed > GRIP_TIMEOUT_SECS:
                confirm_count  = 0
                last_target_id = -1
                robot_state, _resumed = _after_grip_return_state()
                if _resumed is not None:
                    print(f"\n[경고] grip 타임아웃 → PATH_RETURN (복귀 후 phase={_resumed} 재개)")
                else:
                    print(f"\n[경고] grip 타임아웃 → SEARCHING 복귀")
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
            elif cap2 is None or flag_model is None:
                print("[경고] 태극기 카메라 또는 flag.pt 없음 — SEARCHING 복귀")
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
                robot_state = RobotState.SEARCHING
                print(f"[상태] DROPPING → SEARCHING ({elapsed:.1f}s)")
            elif elapsed > DROP_TIMEOUT_SECS:
                print(f"\n[경고] dump 타임아웃 → SEARCHING 복귀")
                confirm_count  = 0
                last_target_id = -1
                robot_state    = RobotState.SEARCHING
            else:
                print(f"[상태] 컨테이너 쏟는중... ({elapsed:.1f}s)", end="\r")

        elif robot_state == RobotState.PATH_NAV:
            if target and path_phase in PATH_INTERRUPT_PHASES:
                # 직진1/3(벽 탐색 중)에서만 목표 물건 감지 시 경로 중단 → SEARCHING(정렬→접근→grip) 전환.
                # 이동/회전 시간은 SEARCHING 동안 _track_search_motion()이 누적하고,
                # grip 완료 후 PATH_RETURN이 그 시간만큼 반대로 움직여 원위치로 되돌아온 뒤 phase를 재개시킴
                path_resume_phase       = path_phase
                search_move_secs        = 0.0
                search_turn_secs_signed = 0.0
                control_wheels(None)
                robot_state = RobotState.SEARCHING
                print(f"\n[경로] 이동 중 목표 물건 감지 ({target['cls']}) → SEARCHING 전환 (phase={path_phase} 저장)")

            else:
                path_now     = time.time()
                path_elapsed = path_now - path_phase_start

                if path_phase == 0:
                    # 시작직진 — 1m (임시로 시간 기반 근사)
                    if path_elapsed >= PATH_START_FORWARD_SECS:
                        control_wheels(None)
                        path_phase       = 1
                        path_phase_start = path_now
                        print(f"\n[경로] 시작 1m 직진 완료(추정) → 우회전")
                    else:
                        control_wheels(None, override_l=PATH_FORWARD_SPEED, override_r=PATH_FORWARD_SPEED)
                        print(f"[경로] 시작직진 (1m 이동중)... ({path_elapsed:.1f}s)", end="\r")

                elif path_phase == 1:
                    # 우회전
                    control_wheels(None, override_l=PATH_RIGHT_L, override_r=PATH_RIGHT_R)
                    if path_elapsed >= PATH_TURN_90_SECS:
                        control_wheels(None)
                        path_phase       = 2
                        path_phase_start = path_now
                        print(f"[경로] 우회전 완료 → 직진1 (벽까지)")

                elif path_phase == 2:
                    # 직진1 — 벽 근접까지 (is_near_wall 미구현이라 현재는 타임아웃으로만 종료)
                    if is_near_wall(frame):
                        control_wheels(None)
                        path_phase       = 3
                        path_phase_start = path_now
                        print(f"\n[경로] 벽 근접 감지 → 좌회전 1")
                    elif path_elapsed > PATH_WALL_TIMEOUT_SECS:
                        control_wheels(None)
                        path_phase       = 3
                        path_phase_start = path_now
                        print(f"\n[경로] 벽 감지 타임아웃({PATH_WALL_TIMEOUT_SECS}s) → 좌회전 1 (임시)")
                    else:
                        control_wheels(None, override_l=PATH_FORWARD_SPEED, override_r=PATH_FORWARD_SPEED)
                        print(f"[경로] 직진1 (벽 탐색중)... ({path_elapsed:.1f}s)", end="\r")

                elif path_phase == 3:
                    # 좌회전 1
                    control_wheels(None, override_l=PATH_LEFT_L, override_r=PATH_LEFT_R)
                    if path_elapsed >= PATH_TURN_90_SECS:
                        control_wheels(None)
                        path_phase       = 4
                        path_phase_start = path_now
                        print(f"[경로] 좌회전 1 완료 → 직진2 ((2,6)-(3,6) 지점까지)")

                elif path_phase == 4:
                    # 직진2 — (2,6)-(3,6) 지점까지 (임시로 시간 기반 근사)
                    if path_elapsed >= PATH_TO_POINT_SECS:
                        control_wheels(None)
                        path_phase       = 5
                        path_phase_start = path_now
                        print(f"\n[경로] (2,6)-(3,6) 지점 도달(추정) → 좌회전 2")
                    else:
                        control_wheels(None, override_l=PATH_FORWARD_SPEED, override_r=PATH_FORWARD_SPEED)
                        print(f"[경로] 직진2 (지점 이동중)... ({path_elapsed:.1f}s)", end="\r")

                elif path_phase == 5:
                    # 좌회전 2
                    control_wheels(None, override_l=PATH_LEFT_L, override_r=PATH_LEFT_R)
                    if path_elapsed >= PATH_TURN_90_SECS:
                        control_wheels(None)
                        path_phase       = 6
                        path_phase_start = path_now
                        print(f"[경로] 좌회전 2 완료 → 직진3 (벽까지)")

                elif path_phase == 6:
                    # 직진3 — 벽 근접까지
                    if is_near_wall(frame):
                        control_wheels(None)
                        path_phase       = 7
                        path_phase_start = path_now
                        print(f"\n[경로] 벽 근접 감지 → 좌회전 3")
                    elif path_elapsed > PATH_WALL_TIMEOUT_SECS:
                        control_wheels(None)
                        path_phase       = 7
                        path_phase_start = path_now
                        print(f"\n[경로] 벽 감지 타임아웃({PATH_WALL_TIMEOUT_SECS}s) → 좌회전 3 (임시)")
                    else:
                        control_wheels(None, override_l=PATH_FORWARD_SPEED, override_r=PATH_FORWARD_SPEED)
                        print(f"[경로] 직진3 (벽 탐색중)... ({path_elapsed:.1f}s)", end="\r")

                elif path_phase == 7:
                    # 좌회전 3
                    control_wheels(None, override_l=PATH_LEFT_L, override_r=PATH_LEFT_R)
                    if path_elapsed >= PATH_TURN_90_SECS:
                        control_wheels(None)
                        path_phase = 8
                        print(f"[경로] 좌회전 3 완료 → 경로 종료")

                else:
                    # path_phase == 8, 완료 → 후면 카메라 태극기 인식 기반 보관함 접근(GO_TO_STORAGE)으로 전환
                    control_wheels(None)
                    robot_state         = RobotState.GO_TO_STORAGE
                    storage_phase       = 0
                    storage_phase_start = path_now
                    storage_enter_time  = path_now
                    print(f"\n[경로] PATH_NAV 완료 → GO_TO_STORAGE 전환 (태극기 탐색 시작)")

        elif robot_state == RobotState.PATH_RETURN:
            # SEARCHING 중 누적된 이동/회전 시간만큼 반대로 움직여 원위치 근처로 복귀
            return_now     = time.time()
            return_elapsed = return_now - path_return_stage_start

            if path_return_stage == 0:
                # 후진 — search_move_secs만큼
                if return_elapsed >= search_move_secs:
                    control_wheels(None)
                    path_return_stage       = 1
                    path_return_stage_start = return_now
                    print(f"\n[경로] 원위치 후진 완료({search_move_secs:.1f}s) → 반대 방향 회전")
                else:
                    control_wheels(None, override_l=-PATH_FORWARD_SPEED, override_r=-PATH_FORWARD_SPEED)
                    print(f"[경로] 원위치 복귀 후진중... ({return_elapsed:.1f}/{search_move_secs:.1f}s)", end="\r")

            else:
                # 반대 방향 회전 — search_turn_secs_signed 부호의 반대 방향으로, 그 크기만큼
                turn_secs = abs(search_turn_secs_signed)
                if search_turn_secs_signed > 0:
                    rl, rr = PATH_LEFT_L, PATH_LEFT_R    # SEARCHING 중 우회전 우세 → 반대인 좌회전으로 복귀
                else:
                    rl, rr = PATH_RIGHT_L, PATH_RIGHT_R  # SEARCHING 중 좌회전 우세 → 반대인 우회전으로 복귀

                if return_elapsed >= turn_secs:
                    control_wheels(None)
                    resumed_phase           = path_resume_phase
                    robot_state             = RobotState.PATH_NAV
                    path_phase              = resumed_phase
                    path_phase_start        = return_now
                    path_resume_phase       = None
                    search_move_secs        = 0.0
                    search_turn_secs_signed = 0.0
                    print(f"\n[경로] 원위치 회전 복귀 완료({turn_secs:.1f}s) → PATH_NAV phase={resumed_phase} 재개")
                else:
                    control_wheels(None, override_l=rl, override_r=rr)
                    print(f"[경로] 원위치 복귀 회전중... ({return_elapsed:.1f}/{turn_secs:.1f}s)", end="\r")

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
            RobotState.PATH_NAV:      (255, 0, 255),
            RobotState.PATH_RETURN:   (180, 0, 180),
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
