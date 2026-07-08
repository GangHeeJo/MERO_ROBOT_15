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

    # gz = 순간 각속도 (LSB), 적분해서 각도 계산
    # MPU9250 기본 ±250°/s → 131 LSB/(°/s)
    GZ_SCALE = 16.1   # 실측 보정 (131.0 × 90.2/735)

    # 정지 상태에서 바이어스 측정 (1초)
    print("바이어스 측정중 (1초 정지)...")
    samples = []
    deadline = time.time() + 1.0
    prev_ts = None
    while time.time() < deadline:
        with lock:
            gz = latest["gz"]
            ts = latest["ts"]
        if ts != prev_ts and gz is not None:
            samples.append(gz)
            prev_ts = ts
        time.sleep(0.01)
    bias = sum(samples) / len(samples) if samples else 0.0
    print(f"바이어스: {bias:.1f}  (샘플 {len(samples)}개)")

    input(f"\nEnter → {TARGET_DEG:.0f}° 제자리 회전 시작...")

    print(f"회전 중... (목표 {TARGET_DEG:.0f}°)")
    cumulative = 0.0
    start_time = time.time()
    prev_ts = None

    while cumulative < TARGET_DEG:
        send(ser, {"T": 1, "L": -TURN_SPEED, "R": TURN_SPEED})
        time.sleep(0.02)
        with lock:
            gz = latest["gz"]
            ts = latest["ts"]
        if ts is None or ts == prev_ts:
            continue
        dt = ts - prev_ts if prev_ts is not None else 0.0
        prev_ts = ts
        if dt <= 0 or dt > 0.5:
            continue
        deg = abs((gz - bias) / GZ_SCALE) * dt
        cumulative += deg
        print(f"  누적: {cumulative:.1f}° / {TARGET_DEG:.0f}°  gz={gz:.0f}  dt={dt*1000:.0f}ms", end="\r")

    elapsed = time.time() - start_time
    send(ser, {"T": 1, "L": 0, "R": 0})

    print(f"\n\n── 결과 ──────────────────────────────")
    print(f"  걸린 시간: {elapsed:.1f}s")
    print(f"  적분 각도: {cumulative:.1f}°")
    print(f"  실제 각도 재서 비교 → GZ_SCALE = {GZ_SCALE} × (적분값/실제각도)")

    ser.close()

if __name__ == "__main__":
    main()
