"""
yolo_cam_test.py — 카메라 + YOLO 탐지만 테스트 (로봇/시리얼 전혀 안 건드림)
─────────────────────────────────────────────
목적: TensorRT 엔진 전환 후 탐지 품질/FPS 확인용. main.py와 달리 ESP32/OpenRB에
      아예 연결하지 않아서 바퀴·그리퍼·팔이 절대 움직이지 않음.
실행: python vision/src/yolo_cam_test.py          # best.engine (TensorRT, 기본값)
      python vision/src/yolo_cam_test.py --pt     # best.pt (PyTorch) — 속도 A/B 비교용
브라우저에서 http://<젯슨IP>:8083 접속하면 전체 클래스 탐지 박스 확인 가능
"""

import argparse
import cv2
import glob
import os
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from ultralytics import YOLO

parser = argparse.ArgumentParser()
parser.add_argument('--pt', action='store_true',
                    help='best.engine(TensorRT) 대신 best.pt(PyTorch)로 실행 — 속도 비교용')
args = parser.parse_args()

BASE_DIR   = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODEL_PATH = os.path.join(BASE_DIR, "model", "best.pt" if args.pt else "best.engine")
model = YOLO(MODEL_PATH)
print(f"[모델] {MODEL_PATH} 로드 완료 — 클래스: {sorted(model.names.values())}")


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
    target=lambda: ThreadingHTTPServer(('0.0.0.0', 8083), _Handler).serve_forever(),
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


print(f"[스트림] http://{_local_ip()}:8083 에서 확인 (Ctrl+C로 종료)")

fps_counter   = 0
fps_timer     = time.time()
_last_print_t = 0.0
try:
    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        results = model(frame, conf=0.25, verbose=False, device="cuda")
        boxes = results[0].boxes

        if boxes is not None and len(boxes) > 0 and time.time() - _last_print_t >= 0.5:
            names = [model.names[int(c)] for c in boxes.cls.tolist()]
            print(f"[탐지] {names}")
            _last_print_t = time.time()

        fps_counter += 1
        elapsed = time.time() - fps_timer
        if elapsed >= 1.0:
            print(f"[FPS] {fps_counter / elapsed:.1f}")
            fps_counter = 0
            fps_timer = time.time()

        annotated = results[0].plot()
        with _lock:
            _stream_frame = annotated

except KeyboardInterrupt:
    pass
finally:
    cap.release()
    print("\n종료")
