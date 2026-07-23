"""
motion_test.py — YOLO 없이 카메라 프레임 차이만으로 "이동중/정지" 판별하는 단독 테스트

목적: 바퀴에 이동 명령을 보냈는데 실제로는 안 움직이는(벽에 막힘/바퀴 헛돎 등) 상황을
YOLO 추론 없이 값싸게 감지할 수 있는지 확인. 연속 프레임을 흑백+축소해서 절대차이를
구하고, 변한 픽셀 비율이 임계값 밑으로 STOPPED_CONFIRM_FRAMES 프레임 연속 유지되면
"정지(STOPPED)", 한 프레임이라도 임계값을 넘으면 즉시 "이동중(MOVING)"으로 판단한다.
정지 판정만 여러 프레임 확인하는 이유는 조명 플리커/노이즈로 한두 프레임 우연히 변화가
적게 나온 것을 실제 정지로 오판하지 않기 위함(그 반대로 실제 움직임은 최대한 빠르게 잡음).

⚠️ PIXEL_DIFF_THRESHOLD/MOVING_RATIO_THRESHOLD/STOPPED_CONFIRM_FRAMES 전부 실측 필요한
임의값 — 텍스처 없는 벽만 보고 있으면 실제로 움직여도 픽셀 차이가 거의 안 나서 정지로
오판할 수 있고, 조명이 깜빡이면 반대로 정지 상태를 이동중으로 오판할 수 있음. 실제
로봇 주행 중(wheels_test.py 등으로 명령 보내면서) 이 스크립트를 같이 띄워서 로그를
보며 값을 조정할 것.

사용법:
  python vision/src/motion_test.py

브라우저 http://<젯슨IP>:8085 에서 STOPPED/MOVING 상태 + 변화율 오버레이 확인 가능
(Ctrl+C로 종료)
"""

import cv2
import glob
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

STREAM_PORT = 8085  # main.py=8080, camera_test.py=8082, yolo_cam_test.py=8083, arm_angle_test.py=8084과 안 겹치게

DOWNSCALE_SIZE          = (320, 240)  # 처리 속도용 축소 크기
PIXEL_DIFF_THRESHOLD    = 25    # ⚠️ 실측 필요 — 이 값보다 크게 변한 픽셀만 "변화"로 셈 (0~255)
MOVING_RATIO_THRESHOLD  = 0.02  # ⚠️ 실측 필요 — 변화 픽셀 비율이 이 이상이면 그 프레임은 "움직임"
STOPPED_CONFIRM_FRAMES  = 10    # ⚠️ 실측 필요 — 이 프레임 수만큼 연속으로 움직임이 없어야 "정지" 확정


def find_camera_index(keywords, fallback):
    """장치 이름 키워드로 카메라 인덱스 자동 탐지 (main.py/camera_test.py와 동일한 방식)."""
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
        return min(matches)
    return fallback


def local_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    except Exception:
        return "127.0.0.1"
    finally:
        s.close()


def motion_ratio(prev_gray, curr_gray):
    """두 흑백 프레임 사이에서 PIXEL_DIFF_THRESHOLD보다 크게 변한 픽셀의 비율(0~1)."""
    diff = cv2.absdiff(curr_gray, prev_gray)
    _, changed_mask = cv2.threshold(diff, PIXEL_DIFF_THRESHOLD, 255, cv2.THRESH_BINARY)
    return cv2.countNonZero(changed_mask) / changed_mask.size


# ── 스트리밍용 최신 프레임 공유 ──────────────────────────
_frame_lock   = threading.Lock()
_stream_frame = None


class _MJPEGHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        self.send_response(200)
        self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
        self.end_headers()
        try:
            while True:
                with _frame_lock:
                    frame = _stream_frame
                if frame is None:
                    time.sleep(0.03)
                    continue
                _, jpg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
                self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpg.tobytes() + b'\r\n')
        except Exception:
            pass

    def log_message(self, *_):
        pass


def main():
    global _stream_frame

    cam_index = find_camera_index(["arducam"], 0)
    cap = cv2.VideoCapture(cam_index)
    if not cap.isOpened():
        print(f"[오류] 카메라 열기 실패(index={cam_index}) — 종료")
        return

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1200)
    cap.set(cv2.CAP_PROP_FPS, 50)
    for _ in range(5):
        cap.read()

    threading.Thread(
        target=lambda: ThreadingHTTPServer(('0.0.0.0', STREAM_PORT), _MJPEGHandler).serve_forever(),
        daemon=True
    ).start()
    print(f"[스트림] http://{local_ip()}:{STREAM_PORT} 에서 STOPPED/MOVING 확인 가능 (Ctrl+C로 종료)")

    prev_gray         = None
    low_motion_streak = 0
    status            = "MOVING"  # 판단 전 초기값은 안전 쪽(움직이는 중)으로 가정
    last_print_t      = 0.0

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                continue

            small = cv2.resize(frame, DOWNSCALE_SIZE)
            gray  = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
            gray  = cv2.GaussianBlur(gray, (5, 5), 0)

            ratio = 0.0
            if prev_gray is not None:
                ratio = motion_ratio(prev_gray, gray)
                if ratio >= MOVING_RATIO_THRESHOLD:
                    status            = "MOVING"
                    low_motion_streak = 0
                else:
                    low_motion_streak += 1
                    if low_motion_streak >= STOPPED_CONFIRM_FRAMES:
                        status = "STOPPED"
            prev_gray = gray

            if time.time() - last_print_t >= 0.5:
                print(f"[모션] {status:7s} ratio={ratio:.4f} streak={low_motion_streak}", end="\r")
                last_print_t = time.time()

            color = (0, 165, 255) if status == "MOVING" else (0, 0, 255)
            disp = frame.copy()
            cv2.putText(disp, f"{status}  ratio={ratio:.4f}",
                        (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 3)
            with _frame_lock:
                _stream_frame = disp

    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        print("\n종료")


if __name__ == "__main__":
    main()
