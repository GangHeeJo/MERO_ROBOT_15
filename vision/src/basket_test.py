"""
basket_test.py — 바스켓(컨테이너, ID3)만 단독으로 열고 닫는 임시 테스트 스크립트

robot.ino의 기존 "dump" 명령은 열고 500ms 후 자동으로 닫히는 방식이라 열린 채로
유지가 안 됨. 이 스크립트는 robot.ino에 새로 추가한 basket_open/basket_close
명령을 사용 — 열린 상태를 그대로 유지할 수 있어서 바스켓 안을 직접 확인하거나
수동으로 비울 때 쓴다.

⚠️ robot.ino에 basket_open/basket_close 명령이 새로 추가됐으므로 OpenRB에
   펌웨어 재업로드가 되어 있어야 동작함.

사용법:
  python vision/src/basket_test.py

명령 (Enter로 입력):
  o   바스켓 열기 (열린 채로 유지)
  c   바스켓 닫기
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
    print("[준비 완료] o(열기)/c(닫기), q로 종료")

    try:
        while True:
            raw = input("> ").strip()
            if not raw:
                continue
            if raw == "q":
                break
            elif raw == "o":
                send_cmd(ser, "basket_open")
                wait_response(ser)
            elif raw == "c":
                send_cmd(ser, "basket_close")
                wait_response(ser)
            else:
                print("[오류] o/c 또는 q만 입력하세요")
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        ser.close()
        print("\n종료.")


if __name__ == "__main__":
    main()
