"""
cam_servo_test.py — 카메라 회전 서보(ID4)만 단독으로 테스트 (카메라 스트림/YOLO/바퀴 전혀 안 건드림)

robot.ino(현재 올라가 있는 메인 펌웨어)에 이미 통합된 cam_backward/cam_forward 명령을
OpenRB로 그대로 보내는 방식이라 재업로드 없이 바로 사용 가능.

사용법:
  python vision/src/cam_servo_test.py

명령 (Enter로 입력):
  b   카메라 뒤로 180도 회전 (cam_backward)
  f   카메라 정면으로 복귀 (cam_forward)
  t   180도 회전 후 2초 대기하고 정면으로 복귀 (왕복 테스트)
  q   종료
"""

import serial
import json
import time
import glob as _glob
import os

BAUD_RATE = 115200


def find_port(keywords, default):
    for p in _glob.glob("/dev/serial/by-id/*"):
        if any(k in p.lower() for k in keywords):
            return os.path.realpath(p)
    return default


def send_cmd(ser, cmd):
    ser.write((json.dumps({"cmd": cmd}) + "\n").encode())
    print(f"[전송] {cmd}")


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
    openrb_port = find_port(["openrb", "robotis", "2ecc"], "/dev/ttyACM1")
    print(f"[연결] OpenRB: {openrb_port}")
    ser = serial.Serial(openrb_port, BAUD_RATE, timeout=0.05, write_timeout=0.5)
    time.sleep(1.0)
    ser.reset_input_buffer()
    print("[준비 완료] b(후방)/f(정면)/t(왕복테스트), q로 종료")

    try:
        while True:
            raw = input("> ").strip()
            if not raw:
                continue
            if raw == "q":
                break
            elif raw == "b":
                send_cmd(ser, "cam_backward")
                wait_response(ser)
            elif raw == "f":
                send_cmd(ser, "cam_forward")
                wait_response(ser)
            elif raw == "t":
                send_cmd(ser, "cam_backward")
                wait_response(ser)
                time.sleep(2.0)
                send_cmd(ser, "cam_forward")
                wait_response(ser)
            else:
                print("[오류] b/f/t 또는 q만 입력하세요")
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        ser.close()
        print("\n종료.")


if __name__ == "__main__":
    main()
