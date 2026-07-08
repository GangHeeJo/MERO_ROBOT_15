import serial
import json
import time
import glob as _glob
import os
import threading

BAUD_RATE = 115200
TARGET_DEG = 90.0

MAX_TURN_SPEED = 0.18
MID_TURN_SPEED = 0.12
MIN_TURN_SPEED = 0.07
STOP_MARGIN_DEG = 3.0

def find_port(keywords, default):
    for p in _glob.glob("/dev/serial/by-id/*"):
        if any(k in p.lower() for k in keywords):
            return os.path.realpath(p)
    return default

def send(ser, msg):
    ser.write((json.dumps(msg) + "\n").encode())

def wrap180(a):
    return (a + 180) % 360 - 180

def main():
    esp_port = find_port(["1a86", "ch343", "ch34"], "/dev/ttyACM0")
    print(f"[연결] ESP32: {esp_port}")

    ser = serial.Serial(esp_port, BAUD_RATE, timeout=0.05)
    time.sleep(1.0)
    ser.reset_input_buffer()

    latest = {"yaw": None, "ts": None}
    lock = threading.Lock()

    def reader():
        while True:
            try:
                line = ser.readline().decode(errors="ignore").strip()
                if not line:
                    continue
                d = json.loads(line)
                if d.get("T") == 126 and "y" in d:
                    with lock:
                        latest["yaw"] = float(d["y"])
                        latest["ts"] = time.time()
            except Exception:
                pass

    threading.Thread(target=reader, daemon=True).start()

    print("yaw 데이터 대기중...")

    deadline = time.time() + 5.0
    while time.time() < deadline:
        send(ser, {"T": 126})
        time.sleep(0.1)
        with lock:
            if latest["yaw"] is not None:
                break

    with lock:
        start_yaw = latest["yaw"]

    if start_yaw is None:
        print("[오류] T=126 yaw 데이터 수신 실패")
        ser.close()
        return

    # 부호는 실제 로봇에서 확인 필요.
    # 현재 main.py convention 기준: +90 목표
    target_yaw = (start_yaw + TARGET_DEG) % 360

    print(f"start_yaw={start_yaw:.1f}°, target_yaw={target_yaw:.1f}°")
    input("Enter → 90도 회전 시작...")

    try:
        while True:
            send(ser, {"T": 126})
            time.sleep(0.03)

            with lock:
                yaw = latest["yaw"]

            if yaw is None:
                send(ser, {"T": 1, "L": 0, "R": 0})
                continue

            diff = wrap180(target_yaw - yaw)
            adiff = abs(diff)

            if adiff <= STOP_MARGIN_DEG:
                send(ser, {"T": 1, "L": 0, "R": 0})
                print(f"\n[완료] yaw={yaw:.1f}°, diff={diff:.1f}°")
                break

            if adiff > 40:
                speed = MAX_TURN_SPEED
            elif adiff > 15:
                speed = MID_TURN_SPEED
            else:
                speed = MIN_TURN_SPEED

            # 방향이 반대로 돌면 이 if/else의 L/R을 서로 바꾸면 됨
            if diff < 0:
                L, R = -speed, speed
            else:
                L, R = speed, -speed

            send(ser, {"T": 1, "L": round(L, 2), "R": round(R, 2)})
            print(f"yaw={yaw:.1f} target={target_yaw:.1f} diff={diff:.1f} speed={speed:.2f}", end="\r")

    finally:
        send(ser, {"T": 1, "L": 0, "R": 0})
        ser.close()

if __name__ == "__main__":
    main()
