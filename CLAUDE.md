# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 프로젝트

pick-and-place 로봇 대회. 카메라로 물체 분류·트래킹 → 집게로 집어서 목표 지점 이동.
비전팀 코드(vision/)와 로봇팀 코드(robot/) 모두 이 리포에 있음.

**하드웨어**
- 보드: NVIDIA Jetson Orin Nano
- 로봇 플랫폼: Waveshare 6x4 UGV (내장 컨트롤러: ESP32 — 바퀴 제어)
- 팔·그리퍼 컨트롤러: ROBOTIS OpenRB-150 (Dynamixel 제어)
- 다이나믹셀: XL430 × 5, 전부 12V, Baudrate 1000000
  - ID 1: 그리퍼 (랙-피니언, 단일 모터로 양 손가락 구동)
  - ID 2: 팔 관절 — 물리 모터 2개가 같은 ID 공유 (한쪽은 Dynamixel Wizard에서 미리 DRIVE_MODE=Reverse로 구워둠)
  - ID 3: 컨테이너 힌지 — 물리 모터 2개, 동일한 ID 공유 구조
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
├── robot/                         # 로봇팀 (OpenRB Arduino — 그리퍼+팔+컨테이너)
│   ├── robot.ino                  # JSON 수신 + 상태 머신 (진입파일 — 폴더명 robot과 일치해야 컴파일됨, main.ino 아님 주의)
│   ├── gripper.ino                # ID1 XL430 그리퍼 (랙-피니언)
│   ├── arm.ino                    # ID2 팔 + ID3 컨테이너 (각각 물리모터 2개, 같은 ID 공유)
│   ├── safety.ino                 # Dynamixel overload/hardware error 감시 + 자동 복구
│   └── test_sequence/             # 카메라·바퀴 없이 그리퍼→팔→컨테이너 1회 자동 테스트 (독립 스케치)
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
  └─→ /dev/ttyACM1 → OpenRB-150          {"cmd":"grip"/"dump"/"idle"/"reset_fault", ...}
  └─← /dev/ttyACM1 ← OpenRB-150          {"status":"gripped"/"grip_failed"/"dumped"/
                                           "motor_fault"/"motor_recovered"/"motion_aborted"/"fault_reset"}
```

## 상태 머신

> ⚠️ 2026-07-20 기준: 수집 개수는 세지 않고 **시간 기반**으로 전환함. 경기 시작(`match_start_time`) 후 `PICK_PHASE_SECS`(2분30초, 여유 있게 잡은 값)가 지나면 몇 개를 집었든 상관없이 `GO_TO_STORAGE`로 넘어가고, 남은 30초 동안 태극기 찾아 후진 접근→dump. 못 채운 개수는 감수하는 설계.

**Python (main.py) — 현재 실제 동작:**
```
SEARCHING → (도달 3프레임 확인) → grip 전송 → GRIPPING
GRIPPING → (gripped/grip_failed/timeout 무엇이든) → SEARCHING
              ↕ 이 두 상태 중 어디서든 경기 시작 후 2분30초 지나면 → GO_TO_STORAGE
GO_TO_STORAGE → (태극기 탐색→후진 접근→도달) → dump 전송 → DROPPING
DROPPING → (dumped/timeout) → SEARCHING
```
(`--test` 플래그: grip 전송 후 결과 기다리지 않고 바로 프로그램 종료)

**OpenRB (robot.ino):**
```
IDLE → (grip 수신) → GRIPPING → (집기 시도, 성공/실패 무관 항상 진행) → LIFTING
                                (팔 올림 → 그리퍼 열어 투하 → 팔 내림) → IDLE (gripped 전송)
     → (dump 수신) → DUMPING → (컨테이너 열고 500ms 후 닫기) → IDLE (dumped 전송)

safety.ino가 overload/hardware error를 감지하면 어느 상태에서든 즉시 IDLE로 복귀
(fatal fault면 사람이 reset_fault 보낼 때까지 명령 거부)
```

## 전원 구성

- 젯슨: 보조배터리 USB-C PD → 배럴잭 (내경 실측 필요)
- XL430 그리퍼+팔+컨테이너 전부: OpenRB 전용 별도 배터리(11.1V 3S LiPo, XT60) → OpenRB — UGV 내장 배터리 아님
