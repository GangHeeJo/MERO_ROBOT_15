"""
imu_test.py — 지자기 기반 yaw 정확도 테스트

사용법:
  python3 vision/src/imu_test.py

Enter → 10바퀴 제자리 회전 후 시작/종료 yaw 비교.
yaw = atan2(my, mx) 로 절대 방위각 계산.
"""

import serial
import json
import time
import glob as _glob
import os
import threading

BAUD_RATE   = 115200
TURN_SPEED  = 0.25
TARGET_DEG  = 90.0

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

    # gz: z축 자이로 (단위: raw, 적분해서 각도 계산)
    # T=1001 주기 실측 필요 — 일단 dt=0.1s 가정
    latest = {"gz": None, "ts": None}
    lock = threading.Lock()

    def reader():
        while True:
            try:
                line = ser.readline().decode(errors="ignore").strip()
                d = json.loads(line)
                if d.get("T") == 1001 and "gz" in d:
                    with lock:
                        latest["gz"] = d["gz"]
                        latest["ts"] = time.time()
            except Exception:
                pass

    threading.Thread(target=reader, daemon=True).start()

    print("자이로 데이터 대기중...")
    deadline = time.time() + 5.0
    while time.time() < deadline:
        with lock:
            if latest["gz"] is not None:
                break
        time.sleep(0.05)

    if latest["gz"] is None:
        print("[오류] 데이터 수신 실패")
        ser.close()
        return

    # gz 스케일 팩터 추정: MPU9250 기본 ±250°/s → 131 LSB/(°/s)
    GZ_SCALE = 131.0

    print(f"초기 gz: {latest['gz']}  (스케일: {GZ_SCALE} LSB/°/s)")
    input(f"\nEnter → {TARGET_DEG:.0f}° 제자리 회전 시작...")

    print(f"회전 중... (목표 {TARGET_DEG:.0f}°)")
    cumulative = 0.0
    prev_ts = time.time()
    start_time = prev_ts

    while cumulative < TARGET_DEG:
        send(ser, {"T": 1, "L": -TURN_SPEED, "R": TURN_SPEED})
        time.sleep(0.05)
        with lock:
            gz = latest["gz"]
            ts = latest["ts"]
        if gz is None or ts is None:
            continue
        now = time.time()
        dt = now - prev_ts
        prev_ts = now
        deg_per_sec = gz / GZ_SCALE
        cumulative += abs(deg_per_sec) * dt
        print(f"  누적: {cumulative:.1f}° / {TARGET_DEG:.0f}°  gz={gz}", end="\r")

    elapsed = time.time() - start_time
    send(ser, {"T": 1, "L": 0, "R": 0})

    print(f"\n\n── 결과 ──────────────────────────────")
    print(f"  걸린 시간: {elapsed:.1f}s")
    print(f"  누적 회전: {cumulative:.1f}° (목표 {TARGET_DEG:.0f}°)")

    ser.close()

if __name__ == "__main__":
    main()
