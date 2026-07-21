"""
wheels_test.py — 바퀴(ESP32)만 단독으로 구동 (카메라/YOLO/OpenRB 전혀 안 건드림)

OpenRB가 응답 없을 때 ESP32/바퀴 쪽은 멀쩡한지 분리해서 확인하는 용도.

사용법:
  python3 vision/src/wheels_test.py

명령 (Enter로 입력):
  w      전진 0.5초
  s      정지
  a      제자리 좌회전 0.5초
  d      제자리 우회전 0.5초
  L R    L,R 속도 직접 지정해서 0.5초 구동 (예: 0.2 -0.2)
  q      종료
"""

import serial
import json
import time
import glob as _glob
import os

BAUD_RATE   = 115200
DRIVE_SECS  = 0.5
SPEED       = 0.2


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


def main():
    esp_port = find_port(["1a86", "ch343", "ch34"], "/dev/ttyACM0")
    print(f"[연결] ESP32: {esp_port}")
    ser = serial.Serial(esp_port, BAUD_RATE, timeout=0.05, write_timeout=0.5)
    time.sleep(1.0)
    ser.reset_input_buffer()
    print("[준비 완료] w/s/a/d 또는 'L R' 속도 직접 입력, q로 종료")

    try:
        while True:
            raw = input("> ").strip()
            if not raw:
                continue
            if raw == "q":
                break
            elif raw == "w":
                drive(ser, SPEED, SPEED, DRIVE_SECS)
            elif raw == "s":
                send(ser, 0, 0)
            elif raw == "a":
                drive(ser, -SPEED, SPEED, DRIVE_SECS)
            elif raw == "d":
                drive(ser, SPEED, -SPEED, DRIVE_SECS)
            else:
                parts = raw.split()
                if len(parts) == 2:
                    try:
                        l, r = float(parts[0]), float(parts[1])
                        drive(ser, l, r, DRIVE_SECS)
                    except ValueError:
                        print("[오류] 'L R' 형식의 숫자 두 개를 입력하세요 (예: 0.2 -0.2)")
                else:
                    print("[오류] w/s/a/d 또는 'L R' 형식으로 입력하세요")
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        send(ser, 0, 0)
        ser.close()
        print("\n종료.")


if __name__ == "__main__":
    main()
