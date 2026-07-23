"""
arm_angle_test.py — 팔(ID2) 각도(raw)를 실시간으로 시험하는 단독 테스트 (카메라 확인 포함)

robot.ino(현재 올라가 있는 메인 펌웨어)에 이미 통합된 arm_to/gripper_open/gripper_close
명령을 OpenRB로 그대로 보내는 방식이라 재업로드 없이 바로 사용 가능(단, arm_to는 로직
자체가 새로 추가된 것이라 robot.ino 재업로드가 한 번 필요함). 이 명령들은 OpenRB가 IDLE
상태일 때만 처리됨(robot.ino) — 그리퍼가 grip/CHECKING/LIFTING 진행 중이면 무시됨.

카메라 스트리밍은 camera_test.py와, Enter로 사진 저장하는 방식은 record.py의
--shutter 모드와 동일한 패턴을 그대로 재사용함.

주로 GRIP_CHECK 중간 정지 각도(arm.ino의 ARM_CHECK_RAW)를 실측할 때 사용한다:
1. o로 그리퍼를 열어 물체를 손으로 물려두거나 x로 닫아 빈 상태를 만들고
2. 팔 raw 값을 이것저것 옮겨보며
3. 브라우저(http://<젯슨IP>:8084)로 그 각도에서 그리퍼 안(집은 물체)이 보이는지 확인한 뒤
4. Enter만 누르면 그 순간 화면을 사진으로 저장(vision/records/arm_angle_test/)해서 비교
5. 딱 맞는 값을 찾으면 arm.ino의 ARM_CHECK_RAW 상수에 반영.

사용법:
  python vision/src/arm_angle_test.py

명령:
  <숫자> Enter   그 raw 값(0~4095)으로 팔 이동 (예: 2100)
  d Enter        ARM_DOWN_RAW(1480, 집기 위치)로 이동
  u Enter        ARM_UP_RAW(2850, 투하 위치)로 이동
  c Enter        ARM_CHECK_RAW(2100, arm.ino 현재값)로 이동
  o Enter        그리퍼 열기 (gripper_open)
  x Enter        그리퍼 닫기 (gripper_close, 대기 상태)
  그냥 Enter     현재 화면을 사진으로 저장 (파일명에 마지막으로 보낸 raw 값 포함)
  q Enter        종료
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

BASE_DIR   = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RECORD_DIR = os.path.join(BASE_DIR, "records", "arm_angle_test")

BAUD_RATE = 115200

# arm.ino에 실측 하드코딩된 값과 맞춰둘 것 — 바뀌면 여기도 같이 수정
ARM_DOWN_RAW  = 1480
ARM_UP_RAW    = 2850
ARM_CHECK_RAW = 2100

STREAM_PORT = 8084  # main.py=8080, camera_test.py=8082, yolo_cam_test.py=8083과 안 겹치게


def find_port(keywords, default):
    for p in _glob.glob("/dev/serial/by-id/*"):
        if any(k in p.lower() for k in keywords):
            return os.path.realpath(p)
    return default


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


# ── 카메라: 별도 스레드에서 계속 읽어두고, 메인 스레드는 input() 프롬프트만 담당 ──
_frame_lock    = threading.Lock()
_latest_frame  = None
_cam_available = False


def _capture_loop(cap):
    global _latest_frame
    while True:
        ret, frame = cap.read()
        if ret:
            with _frame_lock:
                _latest_frame = frame
        else:
            time.sleep(0.05)


class _MJPEGHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        self.send_response(200)
        self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
        self.end_headers()
        try:
            while True:
                with _frame_lock:
                    frame = _latest_frame
                if frame is None:
                    time.sleep(0.03)
                    continue
                _, jpg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
                self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpg.tobytes() + b'\r\n')
        except Exception:
            pass

    def log_message(self, *_):
        pass


def start_camera():
    """카메라를 열고 캡처+스트리밍 스레드를 띄운다. 실패해도 팔 각도 조작 자체는 계속 가능하게
    False만 반환하고 예외를 던지지 않는다 (카메라는 확인용 부가 기능)."""
    global _cam_available
    cam_index = find_camera_index(["arducam"], 0)
    cap = cv2.VideoCapture(cam_index)
    if not cap.isOpened():
        print(f"[카메라] 열기 실패(index={cam_index}) — 화면 확인 없이 각도 조작만 가능")
        return False

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1200)
    cap.set(cv2.CAP_PROP_FPS, 50)
    for _ in range(5):
        cap.read()

    threading.Thread(target=_capture_loop, args=(cap,), daemon=True).start()
    threading.Thread(
        target=lambda: ThreadingHTTPServer(('0.0.0.0', STREAM_PORT), _MJPEGHandler).serve_forever(),
        daemon=True
    ).start()

    print(f"[카메라] index={cam_index} 준비 완료")
    print(f"[스트림] http://{local_ip()}:{STREAM_PORT} 에서 확인 가능")
    _cam_available = True
    return True


def save_snapshot(last_raw, seq):
    with _frame_lock:
        frame = _latest_frame
    if frame is None:
        print("[오류] 아직 카메라 프레임이 없습니다 (카메라 연결 확인)")
        return seq
    os.makedirs(RECORD_DIR, exist_ok=True)
    raw_label = last_raw if last_raw is not None else "unset"
    path = os.path.join(RECORD_DIR, f"raw{raw_label}_{seq:03d}.jpg")
    cv2.imwrite(path, frame)
    print(f"[촬영] 저장: {path}")
    return seq + 1


# ── OpenRB 시리얼 ────────────────────────────────────────
def send_arm_to(ser, raw):
    ser.write((json.dumps({"cmd": "arm_to", "raw": raw}) + "\n").encode())
    print(f"[전송] arm_to raw={raw}")


def send_gripper_open(ser):
    ser.write((json.dumps({"cmd": "gripper_open"}) + "\n").encode())
    print("[전송] gripper_open")


def send_gripper_close(ser):
    ser.write((json.dumps({"cmd": "gripper_close"}) + "\n").encode())
    print("[전송] gripper_close")


def wait_response(ser, timeout=5.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if ser.in_waiting > 0:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if line:
                print(f"[OpenRB] {line}")
                if '"status"' in line:
                    return
        time.sleep(0.02)
    print("[경고] 응답 타임아웃")


def main():
    start_camera()

    openrb_port = find_port(["openrb", "robotis", "2ecc"], "/dev/ttyACM1")
    print(f"[연결] OpenRB: {openrb_port}")
    ser = serial.Serial(openrb_port, BAUD_RATE, timeout=0.05, write_timeout=0.5)
    time.sleep(1.0)
    ser.reset_input_buffer()

    print("[준비 완료] <숫자>=raw 이동, d(집기위치)/u(투하위치)/c(그립확인 중간위치),")
    print("            o(그리퍼 열기)/x(그리퍼 닫기), 그냥 Enter=사진 촬영, q=종료")

    last_raw    = None
    snapshot_no = 0

    try:
        while True:
            raw_in = input("> ").strip()

            if raw_in == "":
                snapshot_no = save_snapshot(last_raw, snapshot_no)
                continue
            if raw_in == "q":
                break
            elif raw_in == "d":
                last_raw = ARM_DOWN_RAW
                send_arm_to(ser, last_raw)
                wait_response(ser)
            elif raw_in == "u":
                last_raw = ARM_UP_RAW
                send_arm_to(ser, last_raw)
                wait_response(ser)
            elif raw_in == "c":
                last_raw = ARM_CHECK_RAW
                send_arm_to(ser, last_raw)
                wait_response(ser)
            elif raw_in == "o":
                send_gripper_open(ser)
                wait_response(ser)
            elif raw_in == "x":
                send_gripper_close(ser)
                wait_response(ser)
            else:
                try:
                    raw = int(raw_in)
                except ValueError:
                    print("[오류] 숫자(0~4095), d/u/c, o/x(그리퍼), 빈 Enter(촬영), q 중 하나를 입력하세요")
                    continue
                if not (0 <= raw <= 4095):
                    print("[오류] raw는 0~4095 범위여야 합니다")
                    continue
                last_raw = raw
                send_arm_to(ser, raw)
                wait_response(ser)
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        ser.close()
        print("\n종료.")


if __name__ == "__main__":
    main()
