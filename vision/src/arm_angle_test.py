"""
arm_angle_test.py — 팔(ID2) 각도(raw)를 실시간으로 시험하는 단독 테스트

robot.ino(현재 올라가 있는 메인 펌웨어)에 이미 통합된 arm_to 명령("cmd":"arm_to","raw":N)을
OpenRB로 그대로 보내는 방식이라 재업로드 없이 바로 사용 가능. arm_to는 OpenRB가 IDLE
상태일 때만 처리됨(robot.ino) — 그리퍼가 grip/CHECKING/LIFTING 진행 중이면 무시됨.

주로 GRIP_CHECK 중간 정지 각도(arm.ino의 ARM_CHECK_RAW)를 실측할 때 사용한다:
1. 이 스크립트로 그리퍼가 물체를 문 채로 팔을 이 raw 값 저 raw 값으로 옮겨보고
2. 다른 터미널에서 camera_test.py 또는 yolo_cam_test.py 스트림(:8082/:8083)을 띄워
   그 각도에서 카메라에 그리퍼 안(집은 물체)이 보이는지 눈으로 확인한 뒤
3. 딱 맞는 값을 찾으면 arm.ino의 ARM_CHECK_RAW 상수에 반영.

사용법:
  python vision/src/arm_angle_test.py

명령 (Enter로 입력):
  <숫자>   그 raw 값(0~4095)으로 팔 이동 (예: 2100)
  d        ARM_DOWN_RAW(1480, 집기 위치)로 이동
  u        ARM_UP_RAW(2850, 투하 위치)로 이동
  c        ARM_CHECK_RAW(2100, arm.ino 현재값)로 이동
  q        종료
"""

import serial
import json
import time
import glob as _glob
import os

BAUD_RATE = 115200

# arm.ino에 실측 하드코딩된 값과 맞춰둘 것 — 바뀌면 여기도 같이 수정
ARM_DOWN_RAW  = 1480
ARM_UP_RAW    = 2850
ARM_CHECK_RAW = 2100


def find_port(keywords, default):
    for p in _glob.glob("/dev/serial/by-id/*"):
        if any(k in p.lower() for k in keywords):
            return os.path.realpath(p)
    return default


def send_arm_to(ser, raw):
    ser.write((json.dumps({"cmd": "arm_to", "raw": raw}) + "\n").encode())
    print(f"[전송] arm_to raw={raw}")


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
    print("[준비 완료] <숫자>=raw 이동, d(집기위치)/u(투하위치)/c(그립확인 중간위치), q로 종료")

    try:
        while True:
            raw_in = input("> ").strip()
            if not raw_in:
                continue
            if raw_in == "q":
                break
            elif raw_in == "d":
                send_arm_to(ser, ARM_DOWN_RAW)
                wait_response(ser)
            elif raw_in == "u":
                send_arm_to(ser, ARM_UP_RAW)
                wait_response(ser)
            elif raw_in == "c":
                send_arm_to(ser, ARM_CHECK_RAW)
                wait_response(ser)
            else:
                try:
                    raw = int(raw_in)
                except ValueError:
                    print("[오류] 숫자(0~4095) 또는 d/u/c/q만 입력하세요")
                    continue
                if not (0 <= raw <= 4095):
                    print("[오류] raw는 0~4095 범위여야 합니다")
                    continue
                send_arm_to(ser, raw)
                wait_response(ser)
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        ser.close()
        print("\n종료.")


if __name__ == "__main__":
    main()
