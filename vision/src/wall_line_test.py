"""
wall_line_test.py — 바닥-벽 경계선(화면을 가로지르는 직선, 수평/사선 둘 다) 감지 테스트
─────────────────────────────────────────────────────────────
목적: YOLO 없이 고전 CV(Hough 직선 변환)로 바닥과 벽 사이 경계선을 찾을 수 있는지 확인.
      로봇/시리얼 전혀 안 건드림 — 카메라 입력만 사용.

원리: Canny 엣지 위에서 cv2.HoughLinesP로 직선 세그먼트를 찾는다 — 순수 수평선만
      가정하지 않고 사선도 그대로 검출됨. 세그먼트를 화면 좌우 끝까지 연장해서
      실제로 그려주고, 그 기울기(각도)를 함께 보고한다.
      각도가 0°에 가까우면 로봇이 벽과 평행, 기울어질수록(+ 또는 -) 로봇이 벽에 대해
      틀어진 정도와 방향을 나타낸다 — 실제 도(度) 값은 카메라 화각 보정이 없어 근사치지만,
      부호와 크기는 회전 제어 신호(정렬용 turn 값)로 바로 쓸 수 있음.
      y가 클수록(화면 아래쪽) 벽이 가깝다고 해석.

실행: python vision/src/wall_line_test.py
브라우저에서 http://<젯슨IP>:8084 접속하면 감지된 선 확인 가능
"""

import cv2
import glob
import math
import numpy as np
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

# ── 파라미터 ─────────────────────────────────────────────
CANNY_LOW              = 15   # Canny 하위 임계값
CANNY_HIGH             = 60   # Canny 상위 임계값 (보통 하위의 2~3배)
HOUGH_THRESHOLD        = 80   # 이 표 수 이상 누적돼야 직선으로 인정
MIN_LINE_LENGTH_RATIO  = 0.3  # 프레임 폭의 이 비율 이상 길어야 후보로 인정
MAX_LINE_GAP           = 30   # 이 픽셀 이내 끊김은 같은 선으로 이어붙임
MAX_ANGLE_DEG          = 40   # 수평 기준 이 각도보다 더 세우면(수직에 가까우면) 제외
MIN_LINE_DISTANCE_PX   = 25   # 화면 중앙 y 기준 이 거리 이내의 선은 같은 경계로 보고 병합(NMS)
MAX_LINES              = 5    # 화면당 최대 몇 개 선까지 보고할지


def _find_camera_index(keywords, fallback):
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


def find_wall_lines(gray, w):
    """Canny+Hough로 화면을 가로지르는 (수평 또는 사선) 경계선들을 찾아 화면 좌우
    끝까지 연장한 좌표와 각도를 반환. y가 작은(먼) 것부터 순서대로.
    반환값: [{"y0": 왼쪽끝y, "yw": 오른쪽끝y, "y_mid": 중앙y, "angle": 도(수평=0), "length": px}, ...]"""
    edges = cv2.Canny(gray, CANNY_LOW, CANNY_HIGH)
    min_len = w * MIN_LINE_LENGTH_RATIO
    segments = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=HOUGH_THRESHOLD,
                                minLineLength=min_len, maxLineGap=MAX_LINE_GAP)
    if segments is None:
        return []

    candidates = []
    for seg in segments:
        x1, y1, x2, y2 = seg[0]
        if x2 == x1:
            continue  # 수직선 제외
        angle = math.degrees(math.atan2(y2 - y1, x2 - x1))
        if abs(angle) > MAX_ANGLE_DEG:
            continue
        length = math.hypot(x2 - x1, y2 - y1)
        slope  = (y2 - y1) / (x2 - x1)
        y_at_0 = y1 - slope * x1        # x=0까지 연장
        y_at_w = y_at_0 + slope * w      # x=w까지 연장
        candidates.append({
            "y0": y_at_0, "yw": y_at_w, "y_mid": (y_at_0 + y_at_w) / 2,
            "angle": angle, "length": length,
        })

    # 긴 선 우선으로 정렬 후 NMS — 이미 뽑은 선과 중앙 y가 너무 가까우면 같은 선으로 보고 스킵
    candidates.sort(key=lambda c: c["length"], reverse=True)
    picked = []
    for c in candidates:
        if all(abs(c["y_mid"] - p["y_mid"]) >= MIN_LINE_DISTANCE_PX for p in picked):
            picked.append(c)
        if len(picked) >= MAX_LINES:
            break

    picked.sort(key=lambda c: c["y_mid"])  # 화면 위(먼 것)부터 아래(가까운 것) 순
    return picked


CAM_INDEX = _find_camera_index(["arducam"], 0)

cap = cv2.VideoCapture(CAM_INDEX)
if not cap.isOpened():
    print("[오류] 카메라 열기 실패 — 종료")
    exit()
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1200)
cap.set(cv2.CAP_PROP_FPS, 50)

_stream_frame = None
_lock         = threading.Lock()
_client_count = 0  # 브라우저가 실제로 보고 있을 때만 오버레이 그리기 수행


class _Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        global _client_count
        self.send_response(200)
        self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
        self.end_headers()
        with _lock:
            _client_count += 1
        try:
            while True:
                with _lock:
                    f = _stream_frame
                if f is None:
                    time.sleep(0.03)
                    continue
                _, jpg = cv2.imencode('.jpg', f, [cv2.IMWRITE_JPEG_QUALITY, 70])
                self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpg.tobytes() + b'\r\n')
        except Exception:
            pass
        finally:
            with _lock:
                _client_count -= 1

    def log_message(self, *_):
        pass


threading.Thread(
    target=lambda: ThreadingHTTPServer(('0.0.0.0', 8084), _Handler).serve_forever(),
    daemon=True
).start()


def _local_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    except Exception:
        return "127.0.0.1"
    finally:
        s.close()


print(f"[스트림] http://{_local_ip()}:8084 에서 확인 (Ctrl+C로 종료)")

fps_counter   = 0
fps_timer     = time.time()
_last_print_t = 0.0

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        h, w = gray.shape
        lines = find_wall_lines(gray, w)

        if time.time() - _last_print_t >= 0.3:
            if lines:
                desc = ", ".join(f"y_mid={l['y_mid']:.0f}({l['y_mid']/h*100:.0f}%) angle={l['angle']:+.1f}°" for l in lines)
                print(f"[벽] {len(lines)}개 후보: {desc}")
            else:
                print(f"[벽] 후보 없음")
            _last_print_t = time.time()

        fps_counter += 1
        elapsed = time.time() - fps_timer
        if elapsed >= 1.0:
            print(f"[FPS] {fps_counter / elapsed:.1f}")
            fps_counter = 0
            fps_timer   = time.time()

        with _lock:
            watching = _client_count > 0
        if watching:
            annotated = frame.copy()
            for i, l in enumerate(lines):
                color = (0, 255, 0) if i == len(lines) - 1 else (0, 255, 255)
                cv2.line(annotated, (0, int(l["y0"])), (w, int(l["yw"])), color, 2)
                label_y = int(max(20, min(h - 10, l["y_mid"] - 8)))
                cv2.putText(annotated, f"y={l['y_mid']:.0f} ({l['y_mid']/h*100:.0f}%) angle={l['angle']:+.1f}",
                            (10, label_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            with _lock:
                _stream_frame = annotated

except KeyboardInterrupt:
    pass
finally:
    cap.release()
    print("\n종료")
