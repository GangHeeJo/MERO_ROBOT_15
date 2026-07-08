"""
encoder_test.py — 1m 직진 후 엔코더 값 확인

사용법:
  python3 vision/src/encoder_test.py

Enter 누르면 1m 직진 시작 → 멈추면 odl/odr 출력
"""

import serial
import json
import time
import glob as _glob
import os
import threading

BAUD_RATE  = 115200
DRIVE_SECS = 3.0    # 1m 가는 데 걸리는 시간 (속도에 따라 조정)
SPEED      = 0.3    # 직진 속도

def find_port(keywords, default):
    for p in _glob.glob("/dev/serial/by-id/*"):
        if any(k in p.lower() for k in keywords):
            return os.path.realpath(p)
    return default

def send(ser, msg):
    ser.write((json.dumps(msg) + "\n").encode())

def read_encoder(ser, timeout=1.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            line = ser.readline().decode(errors="ignore").strip()
            d = json.loads(line)
            if d.get("T") == 1001 and "odl" in d:
                return d["odl"], d["odr"]
        except Exception:
            pass
    return None, None

def main():
    esp_port = find_port(["1a86", "ch343", "ch34"], "/dev/ttyACM0")
    print(f"[연결] ESP32: {esp_port}")
    ser = serial.Serial(esp_port, BAUD_RATE, timeout=0.1)
    time.sleep(1.0)

    # 초기 엔코더 값
    send(ser, {"T": 1001})
    odl0, odr0 = read_encoder(ser, timeout=3.0)
    if odl0 is None:
        print("[오류] 엔코더 수신 실패")
        ser.close()
        return
    print(f"초기값: odl={odl0}  odr={odr0}")

    input("\nEnter 누르면 1m 직진 시작...")

    # 직진
    print(f"직진 {DRIVE_SECS}초...")
    send(ser, {"T": 1, "L": SPEED, "R": SPEED})
    time.sleep(DRIVE_SECS)
    send(ser, {"T": 1, "L": 0, "R": 0})
    time.sleep(0.5)

    # 최종 엔코더 값
    send(ser, {"T": 1001})
    odl1, odr1 = read_encoder(ser, timeout=3.0)
    if odl1 is None:
        print("[오류] 엔코더 수신 실패")
        ser.close()
        return

    dl = odl1 - odl0
    dr = odr1 - odr0
    print(f"\n── 결과 ──────────────────────────")
    print(f"  odl: {odl0} → {odl1}  (Δ{dl:+d})")
    print(f"  odr: {odr0} → {odr1}  (Δ{dr:+d})")
    print(f"  평균 ticks: {(dl+dr)//2}")
    print(f"\n실제 이동 거리 자로 재서 비교하세요.")
    print(f"  TICKS_PER_M = {(dl+dr)//2} / 실제거리(m)")

    ser.close()

if __name__ == "__main__":
    main()
