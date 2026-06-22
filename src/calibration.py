"""
카메라 캘리브레이션 툴
─────────────────────────────
실행: python src/calibration.py

목적:
  카메라가 찍은 픽셀 좌표(cx, cy)를 실제 거리(mm)로 변환하기 위한
  mm/pixel 비율을 측정하고 model/calibration.json 에 저장.

사용법:
  1. 카메라 아래 테이블에 실제 크기를 아는 물체를 놓기
     예) 자(300mm), A4 용지(210mm), 주사위 3개 나란히 등
  2. 이 스크립트 실행
  3. 화면에서 기준 물체의 양쪽 끝을 마우스로 클릭 (총 2번)
  4. 터미널에 실제 거리(mm) 입력
  5. model/calibration.json 저장 완료

저장 후 arducam_test.py 실행 시 자동으로 불러와서 mm 좌표 계산에 사용됨.
"""

import cv2
import json
import os

BASE_DIR     = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CALIB_PATH   = os.path.join(BASE_DIR, "model", "calibration.json")
CAMERA_INDEX = 1   # Arducam 포트 (내장캠이면 0)

points = []   # 마우스로 클릭한 좌표 저장 (최대 2개)

def on_click(event, x, y, flags, param):
    """마우스 왼쪽 클릭 시 좌표 저장 (2개까지만)."""
    if event == cv2.EVENT_LBUTTONDOWN and len(points) < 2:
        points.append((x, y))
        print(f"  클릭 {len(points)}: ({x}, {y})")

# 카메라 열기
cap = cv2.VideoCapture(CAMERA_INDEX)
if not cap.isOpened():
    print(f"⚠️ {CAMERA_INDEX}번 카메라 없음. 0번으로 재시도합니다.")
    cap = cv2.VideoCapture(0)

cv2.namedWindow("Calibration")
cv2.setMouseCallback("Calibration", on_click)   # 마우스 이벤트 등록

print("=== 카메라 캘리브레이션 ===")
print("기준 물체의 양쪽 끝을 순서대로 클릭하세요. (총 2번)")
print("클릭 완료 후 자동으로 다음 단계로 넘어갑니다.")

frame = None
while True:
    ret, frame = cap.read()
    if not ret:
        break

    display = frame.copy()
    h, w = display.shape[:2]

    # 이미지 중심 십자선 표시 (mm 좌표 원점이 되는 지점)
    cv2.line(display, (w // 2, 0), (w // 2, h), (200, 200, 200), 1)
    cv2.line(display, (0, h // 2), (w, h // 2), (200, 200, 200), 1)
    cv2.putText(display, "center (0,0)", (w // 2 + 5, h // 2 - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

    # 클릭한 점 시각화
    for p in points:
        cv2.circle(display, p, 6, (0, 255, 0), -1)

    # 두 점 모두 클릭됐으면 선 + 픽셀 거리 표시
    if len(points) == 2:
        cv2.line(display, points[0], points[1], (0, 255, 0), 2)
        pixel_dist = ((points[1][0] - points[0][0]) ** 2 +
                      (points[1][1] - points[0][1]) ** 2) ** 0.5
        cv2.putText(display, f"pixel dist: {pixel_dist:.1f} px", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    # 안내 텍스트
    cv2.putText(display, f"clicks: {len(points)}/2  |  q: quit",
                (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    cv2.imshow("Calibration", display)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    if len(points) == 2:
        # 2번 클릭 완료 → 루프 종료 후 거리 입력 단계로
        break

cap.release()
cv2.destroyAllWindows()

if len(points) < 2:
    print("클릭이 부족합니다. 다시 실행하세요.")
    exit()

# 두 점 사이의 픽셀 거리 계산 (피타고라스)
pixel_dist = ((points[1][0] - points[0][0]) ** 2 +
              (points[1][1] - points[0][1]) ** 2) ** 0.5

print(f"\n픽셀 거리: {pixel_dist:.1f} px")
real_mm = float(input("실제 거리를 입력하세요 (mm): "))

# mm/pixel 비율 계산
mm_per_pixel = real_mm / pixel_dist

# 결과를 JSON으로 저장
h, w = frame.shape[:2]
calib = {
    "mm_per_pixel": round(mm_per_pixel, 6),   # 1픽셀 = 몇 mm
    "frame_width":  w,                         # 프레임 가로 크기 (mm 원점 계산에 사용)
    "frame_height": h,                         # 프레임 세로 크기
}

with open(CALIB_PATH, "w") as f:
    json.dump(calib, f, indent=2)

print(f"\n✅ 캘리브레이션 완료!")
print(f"   비율: {mm_per_pixel:.4f} mm/pixel")
print(f"   저장: {CALIB_PATH}")
print(f"\n이제 arducam_test.py 실행 시 mm 좌표가 함께 출력됩니다.")
