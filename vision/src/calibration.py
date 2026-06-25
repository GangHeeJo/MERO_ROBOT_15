"""
카메라 캘리브레이션 툴
─────────────────────────────
모드 1 — 헤드리스 (SSH, 모니터 없을 때):
  python src/calibration.py --capture
  → calib_frame.jpg 저장 후 PC에서 열어서 좌표 확인

  python src/calibration.py --calc x1 y1 x2 y2 실제거리mm
  → 예) python src/calibration.py --calc 120 300 540 300 210

모드 2 — GUI (모니터 있을 때):
  python src/calibration.py
  → 화면에서 마우스로 두 점 클릭 후 실제 거리 입력
"""

import cv2
import json
import os
import sys

BASE_DIR     = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CALIB_PATH   = os.path.join(BASE_DIR, "model", "calibration.json")
CAPTURE_PATH = os.path.join(BASE_DIR, "model", "calib_frame.jpg")
CAMERA_INDEX = 1

def open_camera():
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        cap = cv2.VideoCapture(0)
    return cap

def save_calib(pixel_dist, real_mm, w, h):
    mm_per_pixel = real_mm / pixel_dist
    calib = {
        "mm_per_pixel": round(mm_per_pixel, 6),
        "frame_width":  w,
        "frame_height": h,
    }
    with open(CALIB_PATH, "w") as f:
        json.dump(calib, f, indent=2)
    print(f"\n✅ 캘리브레이션 완료!")
    print(f"   비율: {mm_per_pixel:.4f} mm/pixel")
    print(f"   저장: {CALIB_PATH}")

# ── 헤드리스 모드: 사진 촬영 ──────────────────
if "--capture" in sys.argv:
    cap = open_camera()
    for _ in range(10):  # 카메라 워밍업
        cap.read()
    ret, frame = cap.read()
    cap.release()
    if not ret:
        print("❌ 카메라 읽기 실패")
        exit()
    h, w = frame.shape[:2]
    # 중심 십자선 그리기
    cv2.line(frame, (w//2, 0), (w//2, h), (200, 200, 200), 1)
    cv2.line(frame, (0, h//2), (w, h//2), (200, 200, 200), 1)
    cv2.imwrite(CAPTURE_PATH, frame)
    print(f"✅ 사진 저장: {CAPTURE_PATH}")
    print(f"   해상도: {w}x{h}")
    print(f"\n다음 단계:")
    print(f"  1. PC에서 vision/model/calib_frame.jpg 열기")
    print(f"  2. 기준 물체(자, A4 등) 양 끝 픽셀 좌표 확인")
    print(f"  3. 아래 명령 실행:")
    print(f"     python vision/src/calibration.py --calc x1 y1 x2 y2 실제거리mm")
    exit()

# ── 헤드리스 모드: 좌표로 계산 ────────────────
if "--calc" in sys.argv:
    idx = sys.argv.index("--calc")
    try:
        x1, y1, x2, y2, real_mm = map(float, sys.argv[idx+1:idx+6])
    except (IndexError, ValueError):
        print("사용법: python calibration.py --calc x1 y1 x2 y2 실제거리mm")
        exit()
    pixel_dist = ((x2-x1)**2 + (y2-y1)**2) ** 0.5
    print(f"픽셀 거리: {pixel_dist:.1f} px  |  실제 거리: {real_mm} mm")
    # 프레임 크기 가져오기
    cap = open_camera()
    ret, frame = cap.read()
    cap.release()
    h, w = frame.shape[:2] if ret else (480, 640)
    save_calib(pixel_dist, real_mm, w, h)
    exit()

# ── GUI 모드 (모니터 있을 때) ─────────────────
points = []

def on_click(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN and len(points) < 2:
        points.append((x, y))
        print(f"  클릭 {len(points)}: ({x}, {y})")

cap = open_camera()
cv2.namedWindow("Calibration")
cv2.setMouseCallback("Calibration", on_click)
print("=== 카메라 캘리브레이션 ===")
print("기준 물체의 양쪽 끝을 순서대로 클릭하세요. (총 2번)")

frame = None
while True:
    ret, frame = cap.read()
    if not ret:
        break
    display = frame.copy()
    h, w = display.shape[:2]
    cv2.line(display, (w//2, 0), (w//2, h), (200, 200, 200), 1)
    cv2.line(display, (0, h//2), (w, h//2), (200, 200, 200), 1)
    for p in points:
        cv2.circle(display, p, 6, (0, 255, 0), -1)
    if len(points) == 2:
        cv2.line(display, points[0], points[1], (0, 255, 0), 2)
    cv2.putText(display, f"clicks: {len(points)}/2  |  q: quit",
                (10, h-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
    cv2.imshow("Calibration", display)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q') or len(points) == 2:
        break

cap.release()
cv2.destroyAllWindows()

if len(points) < 2:
    print("클릭이 부족합니다. 다시 실행하세요.")
    exit()

pixel_dist = ((points[1][0]-points[0][0])**2 + (points[1][1]-points[0][1])**2) ** 0.5
print(f"\n픽셀 거리: {pixel_dist:.1f} px")
real_mm = float(input("실제 거리를 입력하세요 (mm): "))
h, w = frame.shape[:2]
save_calib(pixel_dist, real_mm, w, h)
