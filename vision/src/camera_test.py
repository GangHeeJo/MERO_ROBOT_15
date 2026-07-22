"""
camera_test.py — 카메라만 단독 가동 (YOLO 추론 없음)
─────────────────────────────────────────────
목적: 연결/해상도/FPS 확인, 서보 회전(cam_servo_test.py) 전후로 화면 확인할 때 사용
실행: python vision/src/camera_test.py
브라우저에서 http://<젯슨IP>:8082 접속하면 원본 화면 실시간으로 볼 수 있음
"""

import cv2
import glob
import os
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _find_camera_index(keywords, fallback):
    """장치 이름 키워드로 카메라 인덱스 자동 탐지 (main.py와 동일한 방식)."""
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


CAM_INDEX = _find_camera_index(["arducam"], 0)

cap = cv2.VideoCapture(CAM_INDEX)
if not cap.isOpened():
    print("[오류] 카메라 열기 실패 — 종료")
    exit()
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1200)
cap.set(cv2.CAP_PROP_FPS, 50)

ret, f = cap.read()
if ret:
    h, w = f.shape[:2]
    print(f"[카메라] 준비 완료: {w}x{h}")

_stream_frame = None
_lock = threading.Lock()


class _Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        self.send_response(200)
        self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
        self.end_headers()
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

    def log_message(self, *_):
        pass


threading.Thread(
    target=lambda: ThreadingHTTPServer(('0.0.0.0', 8082), _Handler).serve_forever(),
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


print(f"[스트림] http://{_local_ip()}:8082 에서 확인 (Ctrl+C로 종료)")

fps_counter = 0
fps_timer   = time.time()
try:
    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        fps_counter += 1
        elapsed = time.time() - fps_timer
        if elapsed >= 1.0:
            print(f"[FPS] {fps_counter / elapsed:.1f}")
            fps_counter = 0
            fps_timer = time.time()

        with _lock:
            _stream_frame = frame

except KeyboardInterrupt:
    pass
finally:
    cap.release()
    print("\n종료")
