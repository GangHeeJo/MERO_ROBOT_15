# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 프로젝트

pick-and-place 로봇 대회. 카메라로 물체 분류·트래킹 → 집게로 집어서 목표 지점 이동.
비전팀 코드(vision/)와 로봇팀 코드(robot/) 모두 이 리포에 있음.

**하드웨어**
- 보드: NVIDIA Jetson Orin Nano
- 로봇 플랫폼: Waveshare UGV02 (내장 컨트롤러: ESP32 — 바퀴 제어)
- 팔·그리퍼 컨트롤러: ROBOTIS OpenRB-150 (Dynamixel 제어)
- 다이나믹셀: XL430 × 6 (팔 관절), XL330 × 2 (그리퍼 손가락)
- 카메라: Arducam USB

**대회 태스크**
- shape-based: d6, d8, d12, d20
- image-based: apple, banana, orange, pineapple

## 폴더 구조

```
MERO_AI_ROBOT/
├── vision/                        # 비전팀 (Jetson Python)
│   ├── src/
│   │   ├── main.py        # 메인 실행 (트래킹 + ESP32/OpenRB 통신)
│   │   ├── calibration.py         # 카메라 캘리브레이션 (픽셀→mm, 1회 실행)
│   │   ├── export_engine.py       # TensorRT 변환 스크립트 (Jetson 전용)
│   │   └── export_image_from_video.py
│   ├── train/
│   │   └── MERO_train.ipynb       # Colab 학습 노트북
│   ├── model/
│   │   ├── best.pt                # 학습된 가중치
│   │   ├── best.engine            # TensorRT 변환 파일 (Jetson 변환 후 생성)
│   │   └── calibration.json       # 캘리브레이션 결과 (calibration.py 실행 후 생성)
│   └── DATASET/
│       ├── shape-based/           # d6·d8·d12·d20
│       └── image-based/           # 과일 (미수집)
├── robot/                         # 로봇팀 (OpenRB Arduino — 팔·그리퍼만)
│   ├── main.ino                   # JSON 수신 + 상태 머신 (IDLE→PICK→DROP→RETURN)
│   ├── arm.ino                    # XL430 × 6 팔 관절 (구성 확정 후 작성)
│   └── gripper.ino                # XL330 × 2 그리퍼 손가락
├── progress.md                    # 전체 팀 인수인계 문서
└── CLAUDE.md
```

## 실행

```bash
# Jetson 연결 시 먼저 권한 열기 (USB 꽂을 때마다)
sudo chmod 666 /dev/ttyUSB0   # ESP32 (UGV02 바퀴)
sudo chmod 666 /dev/ttyACM0   # OpenRB (팔·그리퍼)

# Jetson 셋업 시 순서
python vision/src/calibration.py    # 1. 캘리브레이션 (1회)
python vision/src/export_engine.py  # 2. TensorRT 변환 (1회, Jetson에서만)
python vision/src/main.py   # 3. 메인 실행
```

## 학습 파이프라인

```
Roboflow 라벨링 → MERO_train.ipynb(Colab) → best.pt → export_engine.py → best.engine
```

## 통신 구조

```
Jetson main.py
  ├─→ /dev/ttyUSB0 → ESP32 (UGV02)   {"T":1, "L":speed, "R":speed}  ← 바퀴 직접 제어
  └─→ /dev/ttyACM0 → OpenRB-150      {"cmd":"pick", "cls":"d8", ...} ← 팔·그리퍼
```

**바퀴 제어 방식**: Python의 `control_wheels(target)`이 mx/my → L/R 속도 계산 후 Waveshare JSON 직접 전송. 타겟 30mm 이내 도달 시 OpenRB에 pick 명령 전송.

**OpenRB 상태 머신**: IDLE → PICK (팔+그리퍼 닫기) → DROP (드롭존+그리퍼 열기) → RETURN (팔 홈) → IDLE

탐지 없으면: ESP32에 `{"T":1,"L":0,"R":0}` (정지), OpenRB에 `{"cmd":"idle"}`
