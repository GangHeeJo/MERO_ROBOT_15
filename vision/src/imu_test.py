"""
imu_test.py — 지자기 기반 yaw 정확도 테스트

사용법:
  python3 vision/src/imu_test.py

Enter → 10바퀴 제자리 회전 후 시작/종료 yaw 비교.
yaw = atan2(my, mx) 로 절대 방위각 계산.
"""

import serial
import json
import math
import time
import glob as _glob
import os
import threading

BAUD_RATE  = 115200
TURN_SPEED = 0.25
TURN_SECS  = 20.0   # 10바퀴 예상 시간 (속도에 따라 조정)

def find_port(keywords, default):
    for p in _glob.glob("/dev/serial/by-id/*"):
        if any(k in p.lower() for k in keywords):
            return os.path.realpath(p)
    return default

def send(ser, msg):
    ser.write((json.dumps(msg) + "\n").encode())

def mag_to_yaw(mx, my):
    return math.degrees(math.atan2(my, mx))

def angle_diff(a, b):
    d = a - b
    while d > 180:  d -= 360
    while d < -180: d += 360
    return d

def main():
    esp_port = find_port(["1a86", "ch343", "ch34"], "/dev/ttyACM0")
    print(f"[연결] ESP32: {esp_port}")
    ser = serial.Serial(esp_port, BAUD_RATE, timeout=0.05)
    time.sleep(1.0)
    ser.reset_input_buffer()

    latest = {"yaw": None}
    lock = threading.Lock()

    def reader():
        while True:
            try:
                line = ser.readline().decode(errors="ignore").strip()
                d = json.loads(line)
                if d.get("T") == 1001 and "mx" in d:
                    yaw = mag_to_yaw(d["mx"], d["my"])
                    with lock:
                        latest["yaw"] = yaw
            except Exception:
                pass

    threading.Thread(target=reader, daemon=True).start()

    # 초기값 대기
    print("지자기 데이터 대기중...")
    deadline = time.time() + 5.0
    while time.time() < deadline:
        with lock:
            if latest["yaw"] is not None:
                break
        time.sleep(0.1)

    with lock:
        yaw0 = latest["yaw"]

    if yaw0 is None:
        print("[오류] 데이터 수신 실패")
        ser.close()
        return

    print(f"초기 yaw: {yaw0:.2f}°")
    input(f"\nEnter → {TURN_SECS}초 제자리 회전 시작...")

    print(f"회전 중... ({TURN_SECS}s)")
    deadline = time.time() + TURN_SECS
    while time.time() < deadline:
        send(ser, {"T": 1, "L": -TURN_SPEED, "R": TURN_SPEED})
        time.sleep(0.1)
    send(ser, {"T": 1, "L": 0, "R": 0})
    time.sleep(0.5)

    with lock:
        yaw1 = latest["yaw"]

    diff = angle_diff(yaw1, yaw0)

    print(f"\n── 결과 ──────────────────────────────")
    print(f"  시작 yaw: {yaw0:.2f}°")
    print(f"  종료 yaw: {yaw1:.2f}°")
    print(f"  오차:     {diff:.2f}°  (0에 가까울수록 정확)")

    ser.close()

if __name__ == "__main__":
    main()
