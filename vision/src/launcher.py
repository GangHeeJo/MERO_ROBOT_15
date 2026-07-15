#!/usr/bin/env python3
"""
launcher.py — 물리 버튼으로 main.py 실행/정지

핀 연결 (BOARD 번호):
  버튼A (Pin 11) + GND : shape 순환 (d6→d8→d12→d20)
  버튼B (Pin 13) + GND : fruit 순환 (apple→banana→orange→pineapple)
  버튼C (Pin 15) + GND : 시작/정지

  OLED SSD1306 I2C:
    VCC → Pin 1  (3.3V)
    GND → Pin 6  (GND)
    SDA → Pin 3  (I2C1 SDA)
    SCL → Pin 5  (I2C1 SCL)

실행:
  python3 vision/src/launcher.py
"""

import os
import subprocess
import time
import signal
import sys
import threading

try:
    import Jetson.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False
    print("[경고] Jetson.GPIO 없음 — 버튼 비활성 (키보드로 테스트: s=shape, f=fruit, Enter=시작/정지)")

try:
    from luma.core.interface.serial import i2c as luma_i2c
    from luma.oled.device import ssd1306
    from luma.core.render import canvas
    from PIL import ImageFont
    OLED_AVAILABLE = True
except ImportError:
    OLED_AVAILABLE = False
    print("[경고] luma.oled 없음 — OLED 비활성 (터미널 출력으로 대체)")

# ── 핀 설정 (BOARD 번호) ─────────────────────────────────
BTN_SHAPE = 11   # 버튼A
BTN_FRUIT = 13   # 버튼B
BTN_START = 15   # 버튼C

# ── 클래스 목록 ──────────────────────────────────────────
SHAPES = ['d6', 'd8', 'd12', 'd20']
FRUITS = ['apple', 'banana', 'orange', 'pineapple']

MAIN_PY = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'src', 'main.py')

# ── 폰트 경로 후보 ───────────────────────────────────────
FONT_CANDIDATES = [
    '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf',
    '/usr/share/fonts/truetype/freefont/FreeSansBold.ttf',
    '/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf',
]


class Launcher:
    def __init__(self):
        self.shape_idx = 0
        self.fruit_idx = 0
        self.process = None
        self.status = 'READY'   # READY / RUNNING / DONE
        self.lock = threading.Lock()

        self._setup_gpio()
        self._setup_oled()
        self._update_display()

    # ── GPIO 초기화 ──────────────────────────────────────
    def _setup_gpio(self):
        if not GPIO_AVAILABLE:
            return
        GPIO.setmode(GPIO.BOARD)
        for pin in [BTN_SHAPE, BTN_FRUIT, BTN_START]:
            GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(BTN_SHAPE, GPIO.FALLING, callback=self._on_shape, bouncetime=300)
        GPIO.add_event_detect(BTN_FRUIT, GPIO.FALLING, callback=self._on_fruit, bouncetime=300)
        GPIO.add_event_detect(BTN_START, GPIO.FALLING, callback=self._on_start, bouncetime=500)

    # ── OLED 초기화 ──────────────────────────────────────
    def _setup_oled(self):
        self.oled = None
        self.font_lg = None
        self.font_sm = None

        if not OLED_AVAILABLE:
            return
        try:
            serial = luma_i2c(port=1, address=0x3C)
            self.oled = ssd1306(serial, width=128, height=64)
        except Exception as e:
            print(f"[OLED] 초기화 실패: {e}")
            return

        for path in FONT_CANDIDATES:
            try:
                self.font_lg = ImageFont.truetype(path, 16)
                self.font_sm = ImageFont.truetype(path.replace('Bold', '').replace('-Bold', ''), 11)
                break
            except Exception:
                continue
        if self.font_lg is None:
            self.font_lg = ImageFont.load_default()
            self.font_sm = ImageFont.load_default()

    # ── 버튼 콜백 ────────────────────────────────────────
    def _on_shape(self, _channel=None):
        if self.status == 'RUNNING':
            return
        with self.lock:
            self.shape_idx = (self.shape_idx + 1) % len(SHAPES)
        self._update_display()

    def _on_fruit(self, _channel=None):
        if self.status == 'RUNNING':
            return
        with self.lock:
            self.fruit_idx = (self.fruit_idx + 1) % len(FRUITS)
        self._update_display()

    def _on_start(self, _channel=None):
        if self.process and self.process.poll() is None:
            self._stop()
        else:
            self._start()

    # ── 실행 / 정지 ──────────────────────────────────────
    def _start(self):
        shape = SHAPES[self.shape_idx]
        fruit = FRUITS[self.fruit_idx]
        cmd = ['python3', MAIN_PY, '--cls', shape, fruit, '--timer']
        print(f"[Launcher] 실행: {' '.join(cmd)}")
        self.process = subprocess.Popen(cmd)
        self.status = 'RUNNING'
        self._update_display()
        threading.Thread(target=self._watch, daemon=True).start()

    def _stop(self):
        if self.process:
            self.process.terminate()
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.process.kill()
            self.process = None
        self.status = 'READY'
        self._update_display()
        print("[Launcher] 정지")

    def _watch(self):
        """main.py가 자연 종료됐을 때 상태 업데이트"""
        if self.process:
            self.process.wait()
        self.status = 'DONE'
        self._update_display()
        time.sleep(2)
        self.status = 'READY'
        self._update_display()

    # ── OLED 그리기 ──────────────────────────────────────
    def _update_display(self):
        shape = SHAPES[self.shape_idx]
        fruit = FRUITS[self.fruit_idx]
        print(f"[Launcher] SHAPE:{shape:>3}  FRUIT:{fruit:<12}  STATUS:{self.status}")

        if self.oled is None:
            return

        running = self.status == 'RUNNING'
        done    = self.status == 'DONE'

        with canvas(self.oled) as draw:
            # ── 상단 바: 선택된 클래스 (반전) ─────────────
            draw.rectangle([(0, 0), (127, 20)], fill='white')
            draw.text((4, 2),  shape, font=self.font_lg, fill='black')
            draw.text((68, 2), fruit, font=self.font_lg, fill='black')

            # ── 구분선 ─────────────────────────────────────
            draw.line([(0, 21), (127, 21)], fill='white')

            # ── 상태 ───────────────────────────────────────
            if running:
                draw.text((4, 25), 'RUNNING', font=self.font_lg, fill='white')
                draw.text((4, 52), 'C: STOP', font=self.font_sm, fill='white')
            elif done:
                draw.text((4, 25), 'DONE', font=self.font_lg, fill='white')
            else:
                draw.text((4, 25), 'READY', font=self.font_lg, fill='white')
                draw.text((4, 42), 'A:shape   B:fruit', font=self.font_sm, fill='white')
                draw.text((4, 54), 'C: START', font=self.font_sm, fill='white')

    # ── 종료 정리 ─────────────────────────────────────────
    def cleanup(self):
        self._stop()
        if GPIO_AVAILABLE:
            GPIO.cleanup()
        if self.oled:
            self.oled.cleanup()


# ── GPIO 없을 때 키보드 테스트용 ─────────────────────────
def _keyboard_loop(launcher):
    print("키보드 테스트 모드: s=shape순환  f=fruit순환  Enter=시작/정지  q=종료")
    while True:
        try:
            key = input()
        except EOFError:
            break
        if key == 's':
            launcher._on_shape()
        elif key == 'f':
            launcher._on_fruit()
        elif key == '':
            launcher._on_start()
        elif key == 'q':
            break


def main():
    launcher = Launcher()

    def on_exit(sig, frame):
        print("\n[Launcher] 종료")
        launcher.cleanup()
        sys.exit(0)

    signal.signal(signal.SIGINT, on_exit)
    signal.signal(signal.SIGTERM, on_exit)

    if not GPIO_AVAILABLE:
        _keyboard_loop(launcher)
        launcher.cleanup()
    else:
        print("[Launcher] 버튼 대기 중... (Ctrl+C로 종료)")
        while True:
            time.sleep(0.1)


if __name__ == '__main__':
    main()
