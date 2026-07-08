"""
imu_test.py — IMU 정확도 테스트 (제자리 회전)

사용법:
  python3 vision/src/imu_test.py

Enter → 10바퀴 제자리 회전 후 최종 yaw 출력.
시작 yaw와 끝 yaw 차이가 작을수록 IMU 정확도 높음.
"""

import serial
import json
import time
import glob as _glob
import os
import threading

BAUD_RATE   = 115200
TURN_SPEED  = 0.25   # 회전 속도
TURN_SECS   = 20.0   # 10바퀴 예상 시간 (조정 필요)

def find_port(keywords, default):
    for p in _glob.glob("/dev/serial/by-id/*"):
        if any(k in p.lower() for k in keywords):
            return os.path.realpath(p)
    return default

def send(ser, msg):
    ser.write((json.dumps(msg) + "\n").encode())

def main():
    esp_port = find_port(["1a86", "ch343", "ch34"], "/dev/ttyACM0")
    print(f"[연결] ESP32: {esp_port}")
    ser = serial.Serial(esp_port, BAUD_RATE, timeout=0.05)
    time.sleep(1.0)
    ser.reset_input_buffer()

    latest_yaw = [None]
    lock = threading.Lock()

    def reader():
        while True:
            try:
                line = ser.readline().decode(errors="ignore").strip()
                d = json.loads(line)
                if d.get("T") == 126 and "y" in d:
                    with lock:
                        latest_yaw[0] = float(d["y"])
            except Exception:
                pass

    t = threading.Thread(target=reader, daemon=True)
    t.start()

    # IMU 요청 루프
    def imu_req():
        while True:
            send(ser, {"T": 126})
            time.sleep(0.1)

    threading.Thread(target=imu_req, daemon=True).start()

    # 초기 yaw 대기
    print("IMU 대기중...")
    deadline = time.time() + 5.0
    while time.time() < deadline:
        with lock:
            if latest_yaw[0] is not None:
                break
        time.sleep(0.1)

    with lock:
        yaw0 = latest_yaw[0]

    if yaw0 is None:
        print("[오류] IMU 수신 실패")
        ser.close()
        return

    print(f"초기 yaw: {yaw0:.2f}°")
    input(f"\nEnter → {TURN_SECS}초 동안 제자리 회전 (10바퀴 목표)...")

    # 회전
    print(f"회전 중... ({TURN_SECS}s)")
    send(ser, {"T": 1, "L": -TURN_SPEED, "R": TURN_SPEED})
    time.sleep(TURN_SECS)
    send(ser, {"T": 1, "L": 0, "R": 0})
    time.sleep(0.5)

    with lock:
        yaw1 = latest_yaw[0]

    diff = yaw1 - yaw0
    # -180~180 정규화
    while diff > 180:  diff -= 360
    while diff < -180: diff += 360

    print(f"\n── 결과 ──────────────────────────")
    print(f"  시작 yaw: {yaw0:.2f}°")
    print(f"  종료 yaw: {yaw1:.2f}°")
    print(f"  오차:     {diff:.2f}°  (10바퀴 = 3600° 회전, 오차 0에 가까울수록 정확)")

    ser.close()

if __name__ == "__main__":
    main()
