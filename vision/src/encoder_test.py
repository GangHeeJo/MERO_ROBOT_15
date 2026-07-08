"""
encoder_test.py — 엔코더 오차 측정 도구

사용법:
  python3 vision/src/encoder_test.py

Enter 누를 때마다 구간 거리 출력.
실제 이동 거리(자로 측정)와 비교해서 TICKS_PER_M 보정값 확인.

ESP32 T=1001 응답: {"T":1001, "odl":<int>, "odr":<int>, "v":<float>}
"""

import serial
import json
import time
import glob as _glob

# ── 설정 ─────────────────────────────────────────────────
BAUD_RATE      = 115200
TICKS_PER_M    = 1000       # 보정할 값 — 1m 실측 후 업데이트
WHEEL_BASE_M   = 0.30       # 바퀴 간격 (m), 대략값
# ─────────────────────────────────────────────────────────

def find_port():
    for p in _glob.glob("/dev/serial/by-id/*"):
        lower = p.lower()
        if any(k in lower for k in ["1a86", "ch343", "ch34"]):
            import os
            return os.path.realpath(p)
    return "/dev/ttyACM0"

def parse_line(line: str):
    try:
        d = json.loads(line.strip())
        if d.get("T") == 1001 and "odl" in d and "odr" in d:
            return d["odl"], d["odr"]
    except Exception:
        pass
    return None

def ticks_to_m(ticks):
    return ticks / TICKS_PER_M

def main():
    port = find_port()
    print(f"[연결] {port} @ {BAUD_RATE}")
    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=0.1)
    except Exception as e:
        print(f"[오류] {e}")
        return

    time.sleep(1.0)

    # 초기값 읽기
    odl0 = odr0 = None
    print("초기값 읽는 중...")
    deadline = time.time() + 5.0
    while time.time() < deadline:
        try:
            line = ser.readline().decode(errors="ignore")
        except Exception:
            continue
        result = parse_line(line)
        if result:
            odl0, odr0 = result
            print(f"  초기값: odl={odl0}  odr={odr0}")
            break

    if odl0 is None:
        print("[오류] T=1001 수신 실패. ESP32 연결 확인.")
        ser.close()
        return

    odl_prev, odr_prev = odl0, odr0
    odl_now,  odr_now  = odl0, odr0

    # 백그라운드로 계속 읽기
    import threading
    lock = threading.Lock()

    def reader():
        nonlocal odl_now, odr_now
        while True:
            try:
                line = ser.readline().decode(errors="ignore")
            except Exception:
                break
            result = parse_line(line)
            if result:
                with lock:
                    odl_now, odr_now = result

    t = threading.Thread(target=reader, daemon=True)
    t.start()

    print("\nEnter: 구간 측정 / q+Enter: 종료\n")
    lap = 1
    while True:
        key = input()
        if key.strip().lower() == 'q':
            break

        with lock:
            dl = odl_now - odl0
            dr = odr_now - odr0
            dl_lap = odl_now - odl_prev
            dr_lap = odr_now - odr_prev
            odl_prev, odr_prev = odl_now, odr_now

        dist_l = ticks_to_m(dl_lap)
        dist_r = ticks_to_m(dr_lap)
        dist_avg = (dist_l + dist_r) / 2

        total_l = ticks_to_m(dl)
        total_r = ticks_to_m(dr)
        total_avg = (total_l + total_r) / 2

        print(f"── 구간 {lap} ──────────────────────────────")
        print(f"  ticks  L={dl_lap:+6d}  R={dr_lap:+6d}")
        print(f"  거리   L={dist_l*100:+6.1f}cm  R={dist_r*100:+6.1f}cm  평균={dist_avg*100:+6.1f}cm")
        print(f"  누적   L={total_l*100:+6.1f}cm  R={total_r*100:+6.1f}cm  평균={total_avg*100:+6.1f}cm")
        print(f"  (TICKS_PER_M={TICKS_PER_M})")
        print(f"  → 실제 거리 자로 재서 비교하고 TICKS_PER_M 보정하세요")
        print()
        lap += 1

    ser.close()
    print("종료")

if __name__ == "__main__":
    main()
