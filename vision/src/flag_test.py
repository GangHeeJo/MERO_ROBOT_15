"""
flag_test.py — 전면 카메라로 best.pt(통합 모델)의 flag 탐지 품질 확인용 테스트 스크립트
─────────────────────────────────────────────
실행: python vision/src/flag_test.py
브라우저에서 http://<젯슨IP>:8081 접속하면 탐지 박스 실시간으로 볼 수 있음
"""

import cv2
import glob
import os
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from ultralytics import YOLO

BASE_DIR        = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FLAG_MODEL_PATH = os.path.join(BASE_DIR, "model", "best.pt")
model = YOLO(FLAG_MODEL_PATH)
FLAG_CLASS_IDS = {i for i, n in model.names.items() if n == "flag"}


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


CAM_INDEX = _find_camera_index(["arducam"], 0)  # 전면 물체캠 재사용

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
    target=lambda: ThreadingHTTPServer(('0.0.0.0', 8081), _Handler).serve_forever(),
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


print(f"[스트림] http://{_local_ip()}:8081 에서 확인 (Ctrl+C로 종료)")

_last_print_t = 0.0
try:
    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        results = model(frame, conf=0.5, verbose=False)
        boxes = results[0].boxes
        if boxes is not None and len(boxes) > 0:
            flag_idx = [i for i, c in enumerate(boxes.cls.tolist()) if int(c) in FLAG_CLASS_IDS]
            results[0].boxes = boxes[flag_idx]
        boxes = results[0].boxes

        if boxes is not None and len(boxes) > 0 and time.time() - _last_print_t >= 0.5:
            for b in boxes:
                conf = float(b.conf[0])
                x1, y1, x2, y2 = b.xyxy[0].tolist()
                area = (x2 - x1) * (y2 - y1)
                print(f"[탐지] flag conf={conf:.2f} area={area:.0f}")
            _last_print_t = time.time()

        annotated = results[0].plot()
        with _lock:
            _stream_frame = annotated

except KeyboardInterrupt:
    pass
finally:
    cap.release()
    print("\n종료")
