# MERO AI ROBOT — Progress

> 최종 업데이트: 2026-06-22  
> 비전 담당: 조강희

---

## 프로젝트 개요

카메라로 물체를 실시간 탐지·분류·트래킹 후 집게로 집어서 목표 지점으로 이동하는 pick-and-place 대회.  
이 리포는 **비전팀 + 로봇팀 코드** 모두 포함.

**하드웨어**
- 보드: NVIDIA Jetson Orin Nano
- 로봇 플랫폼: Waveshare UGV02 (내장 컨트롤러: ESP32 — 바퀴 제어)
- 팔·그리퍼 컨트롤러: ROBOTIS OpenRB-150 (Dynamixel 제어)
- 다이나믹셀: XL430 × 6 (팔 관절), XL330 × 2 (그리퍼 손가락)
- 카메라: Arducam USB

**대회 태스크**
- shape-based: d6, d8, d12, d20 (다면체 주사위)
- image-based: apple, banana, orange, pineapple

---

## 시스템 아키텍처

```
┌──────────────┐     USB Serial (/dev/ttyUSB0)    ┌─────────────┐
│              │ ──────────── JSON ───────────────▶ │    ESP32    │ → 바퀴 모터
│   Jetson     │                                    └─────────────┘
│  Orin Nano   │
│  (vision/    │     USB-C (/dev/ttyACM0)           ┌─────────────┐
│  arducam_    │ ──────────── JSON ───────────────▶ │  OpenRB-150 │ → XL430 × 6 (팔)
│  test.py)    │                                    │             │ → XL330 × 2 (그리퍼)
└──────────────┘                                    └─────────────┘
      ▲
      │ Arducam USB
  카메라
```

**Jetson에서 나가는 신호 두 가지:**
1. `/dev/ttyUSB0` → ESP32: 바퀴 이동 명령 (탐지 결과 + 타겟 좌표 JSON)
2. `/dev/ttyACM0` → OpenRB: 팔·그리퍼 명령 (pick / drop / home / idle)

---

## 환경 세팅

```bash
pip install ultralytics opencv-python pyserial
```

---

## 실행 방법

```bash
# Jetson 최초 세팅 시 (순서대로)
python vision/src/calibration.py    # 1. 카메라 캘리브레이션 (1회)
python vision/src/export_engine.py  # 2. TensorRT 변환 (1회)
python vision/src/arducam_test.py   # 3. 메인 실행

# 이후 실행은 항상
python vision/src/arducam_test.py
```

---

## 폴더 구조

```
MERO_AI_ROBOT/
├── vision/       # 비전팀 (Jetson Python)
│   ├── src/
│   │   ├── arducam_test.py             # 메인 실행 (트래킹 + 통신)
│   │   ├── calibration.py              # 픽셀→mm 캘리브레이션 (1회)
│   │   ├── export_engine.py            # TensorRT 변환 (Jetson 1회)
│   │   └── export_image_from_video.py  # 동영상 → 프레임 추출
│   ├── train/
│   │   └── MERO_train.ipynb            # Colab 학습 노트북
│   └── model/
│       ├── best.pt                     # 학습 가중치 (현재 d8만)
│       ├── best.engine                 # TensorRT 파일 (Jetson 변환 후)
│       └── calibration.json            # 캘리브레이션 결과 (1회 실행 후)
├── robot/        # 로봇팀 (OpenRB Arduino)
│   ├── main.ino      # JSON 수신 + 상태 머신
│   ├── mobility.ino  # ESP32 바퀴 제어
│   ├── arm.ino       # XL430 × 6 팔 관절 (구성 확정 후)
│   └── gripper.ino   # XL330 × 2 그리퍼 손가락
└── progress.md
```

---

## 파일별 역할 (vision/)

| 파일 | 역할 |
|------|------|
| `vision/src/arducam_test.py` | 메인 실행 파일. 탐지·트래킹·타겟선정·ESP32/OpenRB 전송 전부 담당 |
| `vision/src/calibration.py` | 픽셀 좌표 → 실제 mm 변환 비율 측정. `vision/model/calibration.json` 생성 |
| `vision/src/export_engine.py` | `best.pt` → `best.engine` TensorRT 변환 (Jetson에서만 실행) |
| `vision/src/export_image_from_video.py` | 동영상에서 프레임 추출해서 데이터셋 생성 |
| `vision/train/MERO_train.ipynb` | Google Colab 학습 노트북 |
| `vision/model/best.pt` | 학습된 모델 가중치 (현재 d8만 학습된 임시본) |

---

## 파일별 역할 (robot/)

| 파일 | 역할 |
|------|------|
| `robot/main.ino` | Jetson JSON 수신 → 파싱 → 상태 머신 실행. 다른 .ino 함수 호출 |
| `robot/mobility.ino` | ESP32 바퀴 모터 제어. Waveshare JSON 포맷으로 전송 |
| `robot/arm.ino` | XL430 × 6 팔 관절 제어 (구성 확정 대기 중, 스텁 상태) |
| `robot/gripper.ino` | XL330 × 2 그리퍼 손가락 제어. Dynamixel2Arduino 사용 |

### 상태 머신 흐름

```
IDLE → target 수신
  ↓
APPROACH → mx, my 기반으로 물체 위치로 이동 (ESP32 바퀴)
  ↓
PICK → 그리퍼 닫기 (OpenRB 그리퍼)
  ↓
CARRY → cls에 따라 목표 드롭존으로 이동 (ESP32 바퀴)
  ↓
DROP → 그리퍼 열기 (OpenRB 그리퍼)
  ↓
RETURN → 초기 위치 복귀 → IDLE
```

### 로봇팀 TODO

| 파일 | 항목 | 내용 |
|------|------|------|
| `mobility.ino` | `MOTOR_SERIAL` | `Serial2` 등 실제 포트로 변경 (현재 `Serial` 임시) |
| `mobility.ino` | `DROP_ZONES[]` | 대회 환경 실측 후 mm 좌표 입력 |
| `mobility.ino` | `ARRIVE_THRESHOLD_MM` | 실물 테스트 후 도착 판정 거리 조정 |
| `gripper.ino` | `FINGER_OPEN_DEG` / `FINGER_CLOSE_DEG` | 실물 테스트 후 실제 각도 측정·수정 |
| `arm.ino` | 전체 구현 | 팔 관절 구성 확정 후 역기구학 + 동작 시퀀스 작성 |

### 필요 라이브러리 (Arduino IDE 라이브러리 매니저)

- **ArduinoJson** (Benoit Blanchon) — JSON 파싱 (`main.ino`)
- **Dynamixel2Arduino** (ROBOTIS) — Dynamixel 제어 (`gripper.ino`, `arm.ino`)

---

## 통신 프로토콜

### 1. Jetson → ESP32 (바퀴 제어)

**연결**: `/dev/ttyUSB0`, 115200 baud, JSON per line

매 프레임 아래 형식으로 전송:

```json
{
  "objects": [
    {"id": 1, "cls": "d8", "cx": 342.5, "cy": 218.3, "mx": 12.3, "my": -5.1, "conf": 0.91}
  ],
  "target": {"id": 1, "cls": "d8", "cx": 342.5, "cy": 218.3, "mx": 12.3, "my": -5.1, "conf": 0.91}
}
```

탐지 없으면: `{"objects": [], "target": null}`

### 2. Jetson → OpenRB (팔·그리퍼 제어)

**연결**: `/dev/ttyACM0`, 115200 baud, JSON per line

```json
{"cmd": "pick",  "cls": "d8", "mx": 12.3, "my": -5.1}
{"cmd": "drop",  "cls": "d8"}
{"cmd": "home"}
{"cmd": "idle"}
```

### 3. OpenRB → Dynamixel (팔·그리퍼 직접)

Dynamixel2Arduino 라이브러리 사용. Protocol 2.0, 57600 baud.  
OpenRB 내장 Dynamixel 포트 (`Serial1`) 사용 — 방향핀 별도 불필요.

| 서보 | ID | 모델 | 전원 |
|------|-----|------|------|
| 그리퍼 좌측 손가락 | 1 | XL330 | 5V |
| 그리퍼 우측 손가락 | 2 | XL330 | 5V |
| 팔 관절 1~6 | 3~8 (TBD) | XL430 | **12V** |

> ⚠️ XL430은 동작 전압 12V. OpenRB 초록 단자에 12V 배터리 직결 필수.  
> ⚠️ 두꺼운 전선 사용 — 얇은 전선 사용 시 과열/합선 위험.

### 4. ESP32 → Waveshare 모터 드라이버

```json
{"T": 1, "L": 좌속도, "R": 우속도}
```

속도 범위: -0.5 ~ +0.5 (0.5 = 100% PWM, 음수 = 역방향)

---

## JSON 필드 설명

| 필드 | 설명 |
|------|------|
| `objects` | 이번 프레임 탐지된 전체 물체 목록 |
| `target` | 집게가 집을 대상 1개 (신뢰도 최고). 없으면 null |
| `id` | 트래킹 ID (프레임 간 유지) |
| `cls` | 물체 종류 (d6 / d8 / d12 / d20 / apple 등) |
| `cx`, `cy` | 픽셀 좌표 |
| `mx`, `my` | 카메라 중심 기준 실제 거리 mm (캘리브레이션 후 포함) |

---

## 데이터셋 현황

| 클래스 | 장수 | 비고 |
|--------|------|------|
| d6 | 85장 | 파일명 정리 완료 (d6_1 ~ d6_85) |
| d8 | 150장 | |
| d12 | 136장 | |
| d20 | 185장 | |
| apple | 0장 | 미수집 |
| banana | 0장 | 미수집 |
| orange | 0장 | 미수집 |
| pineapple | 0장 | 미수집 |

> ⚠️ 현재 데이터 전부 집에서 촬영. 대회 환경(바닥색, 조명)과 달라 재촬영 필요.  
> ⚠️ 현재 `best.pt`는 d8만 학습된 임시 가중치. 전체 클래스 재학습 필요.

---

## 학습 파이프라인

```
1. Roboflow 라벨링 (바운딩박스)
      ↓
2. vision/train/MERO_train.ipynb (Google Colab)
      ↓
3. vision/model/best.pt 저장
      ↓
4. python vision/src/export_engine.py (Jetson)
      ↓
5. vision/model/best.engine → arducam_test.py에서 자동 사용
```

Colab 노트북 실행 전 필요한 것:
- Roboflow API 키
- Google Drive 마운트

---

## 완료된 작업 ✅

- [x] YOLOv8 실시간 트래킹 구현 (`model.track(persist=True)`)
- [x] 타겟 선택 로직 (신뢰도 최고 물체 1개 자동 선정)
- [x] 화면 시각화 (바운딩박스 + TARGET 노란 강조)
- [x] 카메라 캘리브레이션 코드 (`vision/src/calibration.py`)
- [x] TensorRT 변환 스크립트 (`vision/src/export_engine.py`)
- [x] Colab 학습 노트북 (`vision/train/MERO_train.ipynb`)
- [x] d6 이미지 파일명 정리 (d6_1 ~ d6_85)
- [x] pyserial 설치 및 통신 코드 작성
- [x] **듀얼 시리얼 통신 구현** (`vision/src/arducam_test.py`)
  - ESP32 `/dev/ttyUSB0` — 바퀴 제어용 전체 탐지 JSON 전송
  - OpenRB `/dev/ttyACM0` — 팔·그리퍼 pick/idle 명령 전송
- [x] **OpenRB 메인 제어 코드** (`robot/main.ino`)
  - Jetson JSON 수신·파싱 (ArduinoJson)
  - 상태 머신 (IDLE → APPROACH → PICK → CARRY → DROP → RETURN)
- [x] **모빌리티 코드** (`robot/mobility.ino`)
  - Waveshare JSON 포맷 `{"T":1,"L":...,"R":...}` 전송
  - mx/my 기반 `moveToward()` 구현 (근접 감속 포함)
  - 클래스별 드롭존 좌표 배열 (실측 후 수정 필요)
- [x] **그리퍼 코드** (`robot/gripper.ino`)
  - Dynamixel2Arduino 라이브러리 사용
  - XL330 × 2 위치 제어 (Protocol 2.0, 57600 baud)
  - `gripperOpen()` / `gripperClose()` 구현
  - 토크 제한 60% (물체 파손 방지)
- [x] **팔 코드 스텁** (`robot/arm.ino`)
  - 함수 인터페이스 정의 완료
  - 구현은 팔 관절 구성 확정 후 작성

---

## 남은 작업 ⬜

| 우선순위 | 작업 | 비고 |
|----------|------|------|
| 🔴 높음 | 과일 데이터 촬영 (4종) | 현재 0장 |
| 🔴 높음 | 대회 환경에서 재촬영 | d6 우선 |
| 🔴 높음 | Roboflow 라벨링 (전체) | |
| 🟡 중간 | Colab 전체 클래스 학습 | 라벨링 완료 후 |
| 🟡 중간 | 모델 성능 확인 (mAP, confusion matrix) | |
| 🟡 중간 | `arm.ino` 구현 | 팔 관절 구성 확정 후 역기구학 작성 |
| 🟡 중간 | `mobility.ino` 드롭존 좌표 실측 | `DROP_ZONES[]` 값 수정 |
| 🟡 중간 | `gripper.ino` 각도 실측 | `FINGER_OPEN_DEG` / `FINGER_CLOSE_DEG` 수정 |
| 🟢 낮음 | Jetson 도착 후 TensorRT 변환 | |
| 🟢 낮음 | 캘리브레이션 실행 (1회) | 카메라 높이 확정 후 |
| 🟢 낮음 | ESP32 + OpenRB 실물 연동 테스트 | Jetson 도착 후 |
| 🟢 낮음 | 멀티스레딩 (비전 / 제어 분리) | 필요 시 |

---

## 인수인계 시 체크리스트

이어서 작업하는 사람이 확인할 것:

1. `vision/model/best.pt` GitHub에 포함 (현재 d8만 학습된 임시본)
2. Roboflow 프로젝트 접근 권한 확인
3. Colab 노트북 실행 전 Roboflow API 키 입력
4. Jetson 연결 포트 확인:
   - `ls /dev/tty*` 실행
   - ESP32: `ttyUSB0` → `arducam_test.py`의 `ESP32_PORT` 맞춰 수정
   - OpenRB: `ttyACM0` → `OPENRB_PORT` 맞춰 수정
5. OpenRB Arduino 업로드 시 보드: **OpenRB-150** 선택
6. Dynamixel Wizard로 서보 ID 및 Baudrate 사전 설정 (57600 baud)
7. XL430 전원: OpenRB 초록 단자에 12V 배터리 연결 (두꺼운 전선 필수)
8. 캘리브레이션은 카메라 설치 높이 확정 후 1회 실행
