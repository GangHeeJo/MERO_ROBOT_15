"""
wall_line_test.py — 바닥-벽 경계선(가로 방향으로 화면 전체를 가로지르는 선) 감지 테스트
─────────────────────────────────────────────────────────────
목적: YOLO 없이 고전 CV(그라디언트 기반)로 바닥과 벽 사이 경계선을 찾을 수 있는지 확인.
      로봇/시리얼 전혀 안 건드림 — 카메라 입력만 사용.

원리: 세로 방향 그라디언트(Sobel y)가 강한 픽셀 = 가로 방향 밝기 변화(=수평 경계선 후보).
      각 행(y)마다 "그라디언트가 강한 픽셀이 전체 폭의 몇 %인가"를 계산해서,
      그 비율이 가장 높은(=화면 왼쪽부터 오른쪽까지 쭉 이어진) 행을 경계선으로 판단.
      y가 클수록(화면 아래쪽) 벽이 가깝다고 해석 — 카메라 고정 각도 기준 원근법상
      멀리 있는 경계는 위쪽(작은 y), 가까운 경계는 아래쪽(큰 y)에 보임.

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
GRADIENT_THRESHOLD  = 25    # 이 값 이상이면 "강한 가로 경계"로 판단 (0~255 스케일, 실측 후 조정)
MIN_COVERAGE_RATIO  = 0.3   # 한 행에서 이 비율 이상 폭에 걸쳐 경계가 있어야 "쭉 이어진 선"으로 인정
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


def find_wall_line(gray):
    """가장 넓게 이어진 가로 경계선의 y좌표를 반환. 임계값 통과 여부와 무관하게
    항상 최선 후보를 반환 — 튜닝 중엔 위치가 맞는지 눈으로 먼저 확인하기 위함.
    반환값: (best_y, coverage_ratio, passed_threshold)"""
    sobel_y = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
    edge_mask = np.abs(sobel_y) >= GRADIENT_THRESHOLD

    h, w = gray.shape
    row_coverage = edge_mask.sum(axis=1) / w  # 행별 "경계 픽셀 비율" (0~1)

    # 이동평균으로 노이즈 완화 (한두 행만 우연히 튀는 것 방지)
    kernel = np.ones(SMOOTH_WINDOW) / SMOOTH_WINDOW
    smoothed = np.convolve(row_coverage, kernel, mode="same")

    best_y = int(np.argmax(smoothed))
    best_coverage = smoothed[best_y]
    return best_y, best_coverage, best_coverage >= MIN_COVERAGE_RATIO


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
        wall_y, coverage, passed = find_wall_line(gray)

        if time.time() - _last_print_t >= 0.3:
            h = gray.shape[0]
            tag = "확정" if passed else "후보(미달)"
            print(f"[벽:{tag}] y={wall_y} ({wall_y/h*100:.0f}% 지점, coverage={coverage*100:.0f}%, 필요={MIN_COVERAGE_RATIO*100:.0f}%)")
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
            color = (0, 255, 0) if passed else (0, 165, 255)  # 확정=초록, 미달 후보=주황
            cv2.line(annotated, (0, wall_y), (w, wall_y), color, 2)
            tag = "WALL" if passed else "candidate(below threshold)"
            cv2.putText(annotated, f"{tag} y={wall_y} ({wall_y/h*100:.0f}%) cov={coverage*100:.0f}%",
                        (10, max(30, wall_y - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            with _lock:
                _stream_frame = annotated

except KeyboardInterrupt:
    pass
finally:
    cap.release()
    print("\n종료")
