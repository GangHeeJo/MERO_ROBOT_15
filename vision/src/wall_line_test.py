"""
wall_line_test.py — 바닥-벽 경계선(가로 방향으로 화면 전체를 가로지르는 선) 감지 테스트
─────────────────────────────────────────────────────────────
목적: YOLO 없이 고전 CV(그라디언트 기반)로 바닥과 벽 사이 경계선을 찾을 수 있는지 확인.
      로봇/시리얼 전혀 안 건드림 — 카메라 입력만 사용.

원리: 세로 방향 그라디언트(Sobel y)가 강한 픽셀 = 가로 방향 밝기 변화(=수평 경계선 후보).
      각 행(y)마다 "그라디언트가 강한 픽셀이 전체 폭의 몇 %인가"를 계산해서 coverage
      프로파일을 만들고, 그 안에서 극댓값(local peak)들을 전부 찾는다 — 벽이 낮아서
      "바닥→벽 앞면" 경계와 "벽 윗면→먼 바닥(또는 배경)" 경계처럼 선이 여러 개 동시에
      존재하는 경우까지 대응. y가 클수록(화면 아래쪽) 가깝다고 해석.

실행: python vision/src/wall_line_test.py
브라우저에서 http://<젯슨IP>:8084 접속하면 감지된 선 확인 가능
"""

import cv2
import glob
import numpy as np
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

# ── 파라미터 ─────────────────────────────────────────────
GRADIENT_THRESHOLD  = 15    # 이 값 이상이면 "강한 가로 경계"로 판단 (0~255 스케일, 실측 후 조정)
MIN_PEAK_COVERAGE   = 0.15  # 한 행에서 이 비율 이상 폭에 걸쳐 경계가 있어야 후보로 인정
MIN_PEAK_DISTANCE_PX = 25   # 이 거리 이내의 극댓값은 같은 선으로 보고 하나만 남김(NMS)
MAX_LINES           = 5     # 화면당 최대 몇 개 선까지 보고할지
SMOOTH_WINDOW       = 5     # coverage 프로파일 노이즈 완화용 이동평균 윈도우(행 단위)


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


def find_wall_lines(gray):
    """coverage 프로파일에서 극댓값(local peak)을 전부 찾아 y가 작은 순(먼 것부터)으로
    반환. 벽이 낮아 "바닥→벽 앞면"/"벽 윗면→먼 바닥" 선이 동시에 잡히는 상황 대응.
    반환값: [(y, coverage_ratio), ...] — MIN_PEAK_COVERAGE 미만은 아예 제외."""
    sobel_y = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
    edge_mask = np.abs(sobel_y) >= GRADIENT_THRESHOLD

    h, w = gray.shape
    row_coverage = edge_mask.sum(axis=1) / w  # 행별 "경계 픽셀 비율" (0~1)

    # 이동평균으로 노이즈 완화 (한두 행만 우연히 튀는 것 방지)
    kernel = np.ones(SMOOTH_WINDOW) / SMOOTH_WINDOW
    smoothed = np.convolve(row_coverage, kernel, mode="same")

    # 극댓값 후보: 양옆보다 크거나 같은 행
    candidates = [
        (y, smoothed[y]) for y in range(1, h - 1)
        if smoothed[y] >= MIN_PEAK_COVERAGE
        and smoothed[y] >= smoothed[y - 1]
        and smoothed[y] >= smoothed[y + 1]
    ]

    # coverage 높은 순으로 정렬 후 NMS — 이미 뽑은 선과 너무 가까우면 스킵
    candidates.sort(key=lambda c: c[1], reverse=True)
    picked = []
    for y, cov in candidates:
        if all(abs(y - py) >= MIN_PEAK_DISTANCE_PX for py, _ in picked):
            picked.append((y, cov))
        if len(picked) >= MAX_LINES:
            break

    picked.sort(key=lambda c: c[0])  # 화면 위(먼 것)부터 아래(가까운 것) 순으로 반환
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
        lines = find_wall_lines(gray)

        if time.time() - _last_print_t >= 0.3:
            h = gray.shape[0]
            if lines:
                desc = ", ".join(f"y={y}({y/h*100:.0f}%,cov={cov*100:.0f}%)" for y, cov in lines)
                print(f"[벽] {len(lines)}개 후보: {desc}")
            else:
                print(f"[벽] 후보 없음 (필요 coverage={MIN_PEAK_COVERAGE*100:.0f}%)")
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
            h, w = annotated.shape[:2]
            # 가까운(아래쪽/큰 y) 선일수록 진한 초록, 먼 선일수록 연한 색으로 구분
            for i, (y, cov) in enumerate(lines):
                color = (0, 255, 0) if i == len(lines) - 1 else (0, 255, 255)
                cv2.line(annotated, (0, y), (w, y), color, 2)
                cv2.putText(annotated, f"y={y} ({y/h*100:.0f}%) cov={cov*100:.0f}%",
                            (10, max(20, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            with _lock:
                _stream_frame = annotated

except KeyboardInterrupt:
    pass
finally:
    cap.release()
    print("\n종료")
