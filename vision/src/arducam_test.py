"""
MERO_AI_ROBOT 메인 실행 파일
─────────────────────────────
실행: python src/arducam_test.py

동작 순서:
  1. YOLO 모델 로드 (best.pt 또는 best.engine)
  2. 캘리브레이션 파일 로드 (있으면 mm 좌표 계산)
  3. Arducam USB 카메라 열기
  4. 매 프레임: 탐지 → 트래킹 → 타겟 선택 → ESP32로 JSON 전송
"""

import cv2
import os
import json
import serial
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
# 시리얼 초기화 (Jetson → ESP32 USB 연결)
# ──────────────────────────────────────────────
# Jetson에서 ESP32 연결 시 포트 확인 방법: ls /dev/tty*
# 보통 /dev/ttyUSB0 또는 /dev/ttyACM0 로 잡힘
SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE   = 115200  # ESP32 기본 통신 속도

ser = None
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"✅ ESP32 시리얼 연결 성공: {SERIAL_PORT}")
except Exception:
    # ESP32가 연결 안 돼있어도 에러 없이 카메라 단독 모드로 실행됨
    print(f"⚠️ ESP32 시리얼 연결 실패 ({SERIAL_PORT}) — 카메라 단독 모드로 실행합니다.")

def send_to_robot(objects: list, target: dict | None):
    """
    탐지된 전체 물체 목록 + 집을 타겟 1개를 JSON으로 ESP32에 전송.

    ESP32가 받는 데이터 형식 (한 줄, \\n 으로 끝남):
    {
      "objects": [
        {"id": 1, "cls": "d8", "cx": 342.5, "cy": 218.3, "mx": 12.3, "my": -5.1, "conf": 0.91}
      ],
      "target": {"id": 1, "cls": "d8", "cx": 342.5, "cy": 218.3, "mx": 12.3, "my": -5.1, "conf": 0.91}
    }

    탐지 없으면: {"objects": [], "target": null}
    mx, my 는 캘리브레이션 파일이 있을 때만 포함됨 (mm 단위, 이미지 중심 기준)
    """
    if ser is None or not ser.is_open:
        return
    payload = json.dumps({"objects": objects, "target": target}) + "\n"
    ser.write(payload.encode())

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
        if target:
            print(f"[타겟]   ID={target['id']} | {target['cls']} → 집게 이동")

        # 탐지 결과 + 타겟 ESP32로 전송
        send_to_robot(detected, target)

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

        cv2.imshow(WINDOW_NAME, annotated_frame)

        # 'q' 누르면 루프 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # 'q' 종료든 Ctrl+C든 예외 발생이든 반드시 자원 해제
    cap.release()
    if ser and ser.is_open:
        ser.close()
    cv2.destroyAllWindows()
    print("카메라 및 시리얼 자원이 안전하게 해제되었습니다.")
