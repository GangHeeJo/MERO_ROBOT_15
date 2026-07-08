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
DRIVE_SECS = 1.2
SPEED      = 0.3

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

    # 최신 T=1001 계속 읽는 스레드
    latest = {"odl": None, "odr": None}
    lock = threading.Lock()

    def reader():
        while True:
            try:
                line = ser.readline().decode(errors="ignore").strip()
                d = json.loads(line)
                if d.get("T") == 1001 and "odl" in d:
                    with lock:
                        latest["odl"] = d["odl"]
                        latest["odr"] = d["odr"]
            except Exception:
                pass

    t = threading.Thread(target=reader, daemon=True)
    t.start()

    # 초기값 대기
    print("초기 엔코더 대기중...")
    deadline = time.time() + 5.0
    while time.time() < deadline:
        with lock:
            if latest["odl"] is not None:
                break
        time.sleep(0.1)

    with lock:
        odl0 = latest["odl"]
        odr0 = latest["odr"]

    if odl0 is None:
        print("[오류] T=1001 수신 실패. ESP32 연결 확인.")
        ser.close()
        return

    print(f"초기값: odl={odl0}  odr={odr0}")
    input("\nEnter 누르면 직진 시작...")

    # 직진
    print(f"직진 {DRIVE_SECS}초...")
    send(ser, {"T": 1, "L": SPEED, "R": SPEED})
    time.sleep(DRIVE_SECS)
    send(ser, {"T": 1, "L": 0, "R": 0})
    time.sleep(0.5)

    with lock:
        odl1 = latest["odl"]
        odr1 = latest["odr"]

    dl = odl1 - odl0
    dr = odr1 - odr0
    print(f"\n── 결과 ──────────────────────────")
    print(f"  odl: {odl0} → {odl1}  (Δ{dl:+d})")
    print(f"  odr: {odr0} → {odr1}  (Δ{dr:+d})")
    print(f"  평균 ticks: {(dl+dr)//2}")
    print(f"\n실제 거리 자로 재서 → TICKS_PER_M = {(dl+dr)//2} / 실제거리(m)")

    ser.close()

if __name__ == "__main__":
    main()
