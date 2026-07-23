"""
wheels_motion_test.py — 바퀴(ESP32) 구동 + 카메라 프레임 차이 기반 이동중/정지 판별을 합친 테스트

wheels_test.py(바퀴 명령 입력)와 motion_test.py(YOLO 없이 프레임 차이로 이동/정지 판별)를
한 프로세스로 합친 것 — 터미널 두 개 띄울 필요 없이, 바퀴 명령을 보내고 나면 그 직후의
실제 이동 여부가 곧바로 출력된다. 카메라 캡처+판별은 백그라운드 스레드가 계속 돌고,
메인 스레드는 wheels_test.py와 동일한 명령 입력 루프를 담당.

⚠️ PIXEL_DIFF_THRESHOLD/MOVING_RATIO_THRESHOLD/STOPPED_CONFIRM_FRAMES는 motion_test.py와
동일하게 전부 실측 필요한 임의값 (자세한 설명은 motion_test.py 참고).

사용법:
  python vision/src/wheels_motion_test.py

브라우저 http://<젯슨IP>:8086 에서 STOPPED/MOVING 오버레이 확인 가능

명령 (Enter로 입력):
  w        전진 0.5초
  s        정지
  a        제자리 좌회전 0.5초
  d        제자리 우회전 0.5초
  f <초>   SPEED로 입력한 초만큼 직진 (예: f 3)
  r <초>   SPEED로 입력한 초만큼 제자리 회전 (양수=우회전, 음수=좌회전, 예: r 2 / r -2)
  L R      L,R 속도 직접 지정해서 0.5초 구동 (예: 0.2 -0.2)
  q        종료

명령을 실행할 때마다 그 직후의 모션 판별 상태(MOVING/STOPPED)와 변화율을 출력한다 —
바퀴에 명령은 나갔는데 실제로는 안 움직이면(벽에 막힘/헛돎 등) STOPPED로 뜨는지 확인.
"""

import cv2
import glob as _glob
import json
import os
import serial
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

BAUD_RATE   = 115200
DRIVE_SECS  = 0.5
SPEED       = 0.2

STREAM_PORT = 8086  # main.py=8080, camera_test.py=8082, yolo_cam_test.py=8083,
                    # arm_angle_test.py=8084, motion_test.py=8085와 안 겹치게

DOWNSCALE_SIZE          = (320, 240)
PIXEL_DIFF_THRESHOLD    = 25    # ⚠️ 실측 필요 (motion_test.py 참고)
MOVING_RATIO_THRESHOLD  = 0.02  # ⚠️ 실측 필요
STOPPED_CONFIRM_FRAMES  = 10    # ⚠️ 실측 필요


# ── ESP32 바퀴 ────────────────────────────────────────────
def find_port(keywords, default):
    for p in _glob.glob("/dev/serial/by-id/*"):
        if any(k in p.lower() for k in keywords):
            return os.path.realpath(p)
    return default


def send(ser, l, r):
    ser.write((json.dumps({"T": 1, "L": round(l, 2), "R": round(r, 2)}) + "\n").encode())


def drive(ser, l, r, secs):
    deadline = time.time() + secs
    while time.time() < deadline:
        send(ser, l, r)
        time.sleep(0.05)
    send(ser, 0, 0)


# ── 카메라 + 모션 판별 (백그라운드 스레드) ────────────────
def find_camera_index(keywords, fallback):
    """장치 이름 키워드로 카메라 인덱스 자동 탐지 (main.py/camera_test.py와 동일한 방식)."""
    matches = []
    for path in _glob.glob("/sys/class/video4linux/video*/name"):
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
    diff = cv2.absdiff(curr_gray, prev_gray)
    _, changed_mask = cv2.threshold(diff, PIXEL_DIFF_THRESHOLD, 255, cv2.THRESH_BINARY)
    return cv2.countNonZero(changed_mask) / changed_mask.size


_status_lock  = threading.Lock()
_status       = "MOVING"  # 판단 전 초기값은 안전 쪽(움직이는 중)으로 가정
_ratio        = 0.0
_stream_frame = None
_cam_available = False


class _MJPEGHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        self.send_response(200)
        self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
        self.end_headers()
        try:
            while True:
                with _status_lock:
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


def _motion_capture_loop(cap):
    global _status, _ratio, _stream_frame

    prev_gray         = None
    low_motion_streak = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.05)
            continue

        small = cv2.resize(frame, DOWNSCALE_SIZE)
        gray  = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
        gray  = cv2.GaussianBlur(gray, (5, 5), 0)

        ratio = 0.0
        if prev_gray is not None:
            ratio = motion_ratio(prev_gray, gray)
            with _status_lock:
                if ratio >= MOVING_RATIO_THRESHOLD:
                    _status = "MOVING"
                    low_motion_streak = 0
                else:
                    low_motion_streak += 1
                    if low_motion_streak >= STOPPED_CONFIRM_FRAMES:
                        _status = "STOPPED"
                _ratio = ratio
        prev_gray = gray

        with _status_lock:
            st = _status
        color = (0, 165, 255) if st == "MOVING" else (0, 0, 255)
        disp = frame.copy()
        cv2.putText(disp, f"{st}  ratio={ratio:.4f}",
                    (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 3)
        with _status_lock:
            _stream_frame = disp


def start_motion_detector():
    """카메라를 열고 캡처+판별+스트리밍 스레드를 띄운다. 실패해도 바퀴 조작 자체는
    계속 가능하게 False만 반환한다 (모션 판별은 확인용 부가 기능)."""
    global _cam_available
    cam_index = find_camera_index(["arducam"], 0)
    cap = cv2.VideoCapture(cam_index)
    if not cap.isOpened():
        print(f"[카메라] 열기 실패(index={cam_index}) — 모션 판별 없이 바퀴 조작만 가능")
        return False

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1200)
    cap.set(cv2.CAP_PROP_FPS, 50)
    for _ in range(5):
        cap.read()

    threading.Thread(target=_motion_capture_loop, args=(cap,), daemon=True).start()
    threading.Thread(
        target=lambda: ThreadingHTTPServer(('0.0.0.0', STREAM_PORT), _MJPEGHandler).serve_forever(),
        daemon=True
    ).start()

    print(f"[카메라] index={cam_index} 준비 완료")
    print(f"[스트림] http://{local_ip()}:{STREAM_PORT} 에서 확인 가능")
    _cam_available = True
    return True


def report_motion():
    with _status_lock:
        st, r = _status, _ratio
    if _cam_available:
        print(f"[모션] {st}  ratio={r:.4f}")
    else:
        print("[모션] 카메라 없음 — 판별 불가")


def main():
    start_motion_detector()

    esp_port = find_port(["1a86", "ch343", "ch34"], "/dev/ttyACM0")
    print(f"[연결] ESP32: {esp_port}")
    ser = serial.Serial(esp_port, BAUD_RATE, timeout=0.05, write_timeout=0.5)
    time.sleep(1.0)
    ser.reset_input_buffer()
    print("[준비 완료] w/s/a/d 또는 'L R' 속도 직접 입력, q로 종료")
    print("           (명령 실행 직후 모션 판별 결과가 출력됨)")

    try:
        while True:
            raw = input("> ").strip()
            if not raw:
                continue
            if raw == "q":
                break
            elif raw == "w":
                drive(ser, SPEED, SPEED, DRIVE_SECS)
                report_motion()
            elif raw == "s":
                send(ser, 0, 0)
                report_motion()
            elif raw == "a":
                drive(ser, -SPEED, SPEED, DRIVE_SECS)
                report_motion()
            elif raw == "d":
                drive(ser, SPEED, -SPEED, DRIVE_SECS)
                report_motion()
            else:
                parts = raw.split()
                if len(parts) == 2 and parts[0] == "f":
                    try:
                        secs = float(parts[1])
                        print(f"[직진] {secs:.1f}초...")
                        drive(ser, SPEED, SPEED, secs)
                        report_motion()
                    except ValueError:
                        print("[오류] 'f <초>' 형식으로 입력하세요 (예: f 3)")
                elif len(parts) == 2 and parts[0] == "r":
                    try:
                        secs = float(parts[1])
                        direction = "우" if secs >= 0 else "좌"
                        print(f"[회전] {direction}회전 {abs(secs):.1f}초...")
                        if secs >= 0:
                            drive(ser, SPEED, -SPEED, secs)
                        else:
                            drive(ser, -SPEED, SPEED, -secs)
                        report_motion()
                    except ValueError:
                        print("[오류] 'r <초>' 형식으로 입력하세요 (예: r 2 / r -2)")
                elif len(parts) == 2:
                    try:
                        l, r = float(parts[0]), float(parts[1])
                        drive(ser, l, r, DRIVE_SECS)
                        report_motion()
                    except ValueError:
                        print("[오류] 'L R' 형식의 숫자 두 개를 입력하세요 (예: 0.2 -0.2)")
                else:
                    print("[오류] w/s/a/d, 'f <초>', 'r <초>' 또는 'L R' 형식으로 입력하세요")
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        send(ser, 0, 0)
        ser.close()
        print("\n종료.")


if __name__ == "__main__":
    main()
