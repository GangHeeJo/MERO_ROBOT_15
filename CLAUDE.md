# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 프로젝트

pick-and-place 로봇 대회. 카메라로 물체 분류·트래킹 → 집게로 집어서 목표 지점 이동.
비전팀 코드(vision/)와 로봇팀 코드(robot/) 모두 이 리포에 있음.

**하드웨어**
- 보드: NVIDIA Jetson Orin Nano
- 로봇 플랫폼: Waveshare 6x4 UGV (내장 컨트롤러: ESP32 — 바퀴 제어)
- 팔·그리퍼 컨트롤러: ROBOTIS OpenRB-150 (Dynamixel 제어)
- 다이나믹셀: XC330 × 1 (그리퍼, 12V, 랙-피니언으로 양 손가락 구동) — 팔 없음
- 카메라: ArduCAM 2.3MP AR0234 글로벌 셔터 USB 3.0

**대회 태스크**
- shape-based: d6, d8, d12, d20
- image-based: apple, banana, orange, pineapple

## 폴더 구조

```
MERO_AI_ROBOT/
├── vision/                        # 비전팀 (Jetson Python)
│   ├── src/
│   │   ├── main.py                # 메인 실행 (트래킹 + ESP32/OpenRB 통신)
│   │   ├── calibration.py         # 카메라 캘리브레이션 (픽셀→mm, 1회 실행)
│   │   ├── trt_export.py          # TensorRT 변환 스크립트 (Jetson 전용)
│   │   └── video_to_frames.py
│   ├── train/
│   │   └── train.ipynb            # Colab 학습 노트북
│   ├── model/
│   │   ├── best.pt                # 학습된 가중치 (d6/d8/d12/d20)
│   │   ├── best.engine            # TensorRT 변환 파일 (Jetson 변환 후 생성)
│   │   └── calibration.json       # 캘리브레이션 결과 (calibration.py 실행 후 생성)
│   └── DATASET/
│       ├── shape-based/           # d6·d8·d12·d20
│       └── image-based/           # 과일 (미수집)
├── robot/                         # 로봇팀 (OpenRB Arduino — 그리퍼만)
│   ├── main.ino                   # JSON 수신 + 상태 머신
│   └── gripper.ino                # XC330 × 1 그리퍼 (랙-피니언)
├── ros2/                          # ROS2 패키지 (레퍼런스 보관용, 미사용)
├── rulebook.md                    # 대회 공식 룰북
├── progress.md                    # 전체 팀 인수인계 문서
└── CLAUDE.md
```

## 실행

```bash
# Jetson USB 권한 열기 (꽂을 때마다)
sudo chmod 666 /dev/ttyACM0   # ESP32 (UGV02 바퀴, CH343 드라이버 → ACM)
sudo chmod 666 /dev/ttyACM1   # OpenRB (팔·그리퍼)

# 메인 실행 (캘리브레이션 없어도 동작)
python vision/src/main.py

# 경기 당일 — 타겟 클래스 지정
python vision/src/main.py --cls d8

# 선택: 캘리브레이션 (1회, 카메라 높이 확정 후)
python vision/src/calibration.py

# 선택: TensorRT 변환 (1회, Jetson에서만)
python vision/src/trt_export.py
```

## 학습 파이프라인

```
Roboflow 라벨링 → train.ipynb(Colab) → best.pt → trt_export.py → best.engine
```

## 통신 구조

```
Jetson main.py
  ├─→ /dev/ttyACM0 → ESP32 (UGV 바퀴)   {"T":1, "L":speed, "R":speed}
  └─→ /dev/ttyACM1 → OpenRB-150          {"cmd":"grip"/"drop"/"idle", ...}
  └─← /dev/ttyACM1 ← OpenRB-150          {"status":"gripped"/"grip_failed"/"done"}
```

## 상태 머신

**Python (main.py):**
```
SEARCHING → (도달) → GRIPPING → (gripped)     → GO_TO_STORAGE → (직진완료) → DROPPING → (done) → SEARCHING
                                 (grip_failed) → SEARCHING
                                 (timeout)     → SEARCHING
```

**OpenRB (main.ino):**
```
IDLE → (grip 수신) → GRIPPING → (집기성공) → HOLDING → (drop 수신) → DROPPING → IDLE
                              → (전류미달) → IDLE (grip_failed 전송)
```

## 전원 구성

- 젯슨: 보조배터리 USB-C PD → 배럴잭 (내경 실측 필요)
- XL430 팔 + XC330 그리퍼: UGV 내장 12V 배터리 → OpenRB
