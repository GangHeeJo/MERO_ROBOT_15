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

    # gz는 누적 적산값으로 추정 (odl/odr 방식과 동일)
    # 스케일은 실측으로 보정
    GZ_SCALE = 131.0   # LSB/° — 실측 후 조정

    with lock:
        gz0 = latest["gz"]

    print(f"초기 gz: {gz0}")
    input(f"\nEnter → {TARGET_DEG:.0f}° 제자리 회전 시작...")

    start_time = time.time()
    print(f"회전 중... (목표 {TARGET_DEG:.0f}°)")

    while True:
        send(ser, {"T": 1, "L": -TURN_SPEED, "R": TURN_SPEED})
        time.sleep(0.05)
        with lock:
            gz_now = latest["gz"]
        delta = abs(gz_now - gz0) / GZ_SCALE
        print(f"  누적: {delta:.1f}° / {TARGET_DEG:.0f}°  gz={gz_now}", end="\r")
        if delta >= TARGET_DEG:
            break

    elapsed = time.time() - start_time
    send(ser, {"T": 1, "L": 0, "R": 0})
    time.sleep(0.3)

    with lock:
        gz1 = latest["gz"]

    total_ticks = gz1 - gz0
    print(f"\n\n── 결과 ──────────────────────────────")
    print(f"  걸린 시간: {elapsed:.1f}s")
    print(f"  gz: {gz0} → {gz1}  (Δ{total_ticks:+d})")
    print(f"  계산 각도: {total_ticks/GZ_SCALE:.1f}°  (GZ_SCALE={GZ_SCALE})")
    print(f"\n실제 각도 측정 후: GZ_SCALE = {total_ticks} / 실제각도(°)")

    ser.close()

if __name__ == "__main__":
    main()
