# MERO AI ROBOT — Progress

> 최종 업데이트: 2026-06-25  
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
│  main.py)    │ ──────────── JSON ───────────────▶ │  OpenRB-150 │ → XL430 × 6 (팔)                                    │             │ → XL330 × 2 (그리퍼)
└──────────────┘                                    └─────────────┘
      ▲
      │ Arducam USB
  카메라
```

**Jetson에서 나가는 신호 두 가지:**
1. `/dev/ttyUSB0` → ESP32: 바퀴 속도 명령 `{"T":1,"L":...,"R":...}` (Waveshare 포맷, Python이 직접 계산)
2. `/dev/ttyACM0` → OpenRB: 팔·그리퍼 명령 (pick / idle)

---

## 환경 세팅

```bash
pip install ultralytics opencv-python pyserial
```

---

## Jetson USB 권한 설정 (매번 필요)

USB 케이블을 꽂거나 Jetson이 재부팅될 때마다 아래 명령을 실행해야 함.  
안 하면 Python에서 시리얼 포트 열기 실패.

```bash
sudo chmod 666 /dev/ttyACM0   # ESP32 (UGV02 바퀴) — CH343 드라이버 → ACM으로 잡힘
sudo chmod 666 /dev/ttyACM1   # OpenRB (팔·그리퍼)
```

포트 번호 확인:
```bash
ls /dev/ttyACM*
# ESP32(UGV)가 ttyACM0, OpenRB가 ttyACM1 으로 잡히는 것이 기본
# 꽂는 순서에 따라 바뀔 수 있으니 main.py 상단 ESP32_PORT / OPENRB_PORT 확인
```

---

## 실행 방법

```bash
# Jetson 최초 세팅 시 (순서대로)
python vision/src/calibration.py    # 1. 카메라 캘리브레이션 (1회)
python vision/src/trt_export.py  # 2. TensorRT 변환 (1회)
python vision/src/main.py   # 3. 메인 실행

# 이후 실행은 항상
python vision/src/main.py
```

---

## 폴더 구조

```
MERO_AI_ROBOT/
├── vision/                        # 비전팀 (Jetson Python)
│   ├── src/
│   │   ├── main.py                # 메인 실행 (트래킹 + 통신)
│   │   ├── calibration.py         # 픽셀→mm 캘리브레이션 (1회)
│   │   ├── trt_export.py          # TensorRT 변환 (Jetson 1회)
│   │   └── video_to_frames.py     # 동영상 → 프레임 추출
│   ├── train/
│   │   └── train.ipynb            # Colab 학습 노트북
│   └── model/
│       ├── best.pt                # 학습 가중치 (d6/d8/d12/d20 전체)
│       ├── best.engine            # TensorRT 파일 (Jetson 변환 후)
│       └── calibration.json       # 캘리브레이션 결과 (1회 실행 후)
├── robot/                         # 로봇팀 (OpenRB Arduino — 팔·그리퍼만)
│   ├── main.ino                   # JSON 수신 + 상태 머신
│   ├── arm.ino                    # XL430 × 6 팔 관절 (구성 확정 후)
│   └── gripper.ino                # XL330 × 2 그리퍼 손가락
├── ros2/                          # ROS2 패키지 (Jetson robot_ws/src/에 배포)
│   └── mobility_pkg/
│       ├── mobility_pkg/
│       │   ├── camera_node.py         # 카메라 → /image_raw 발행
│       │   ├── yolo_vision_node.py    # YOLO 추론 → /detected_objects 발행
│       │   ├── main_decision_node.py  # 면적 기반 이동 판단 → /cmd_vel
│       │   ├── ugv_controller_node.py # /cmd_vel → ESP32 시리얼
│       │   └── gripper_node.py        # /gripper_cmd → OpenRB 시리얼 (미구현)
│       └── launch/
│           └── robot_bringup.launch.py
└── progress.md
```

---

## 파일별 역할 (vision/)

| 파일 | 역할 |
|------|------|
| `vision/src/main.py` | 메인 실행 파일. 탐지·트래킹·타겟선정·ESP32/OpenRB 전송 전부 담당 |
| `vision/src/calibration.py` | 픽셀 좌표 → 실제 mm 변환 비율 측정. `vision/model/calibration.json` 생성 |
| `vision/src/trt_export.py` | `best.pt` → `best.engine` TensorRT 변환 (Jetson에서만 실행) |
| `vision/src/video_to_frames.py` | 동영상에서 프레임 추출해서 데이터셋 생성 |
| `vision/train/train.ipynb` | Google Colab 학습 노트북 |
| `vision/model/best.pt` | 학습된 모델 가중치 (현재 d8만 학습된 임시본) |

---

## 파일별 역할 (robot/)

| 파일 | 역할 |
|------|------|
| `robot/main.ino` | Jetson pick 명령 수신 → 팔·그리퍼 상태 머신 실행 |
| `robot/arm.ino` | XL430 × 6 팔 관절 제어 (구성 확정 대기 중, 스텁 상태) |
| `robot/gripper.ino` | XL330 × 2 그리퍼 손가락 제어. Dynamixel2Arduino 사용 |

> 바퀴 제어(ESP32)는 `vision/src/main.py`의 `control_wheels()`가 직접 담당.

### 상태 머신 흐름

바퀴 이동은 Python이 담당, OpenRB는 팔·그리퍼 시퀀스만 실행:

```
[Python] 탐지 → 타겟 선택 → control_wheels()로 ESP32 직접 제어
                           ↓ (타겟 30mm 이내 도달 시)
                     OpenRB에 pick 명령 전송
                           ↓
[OpenRB] IDLE → PICK (팔 내리기 + 그리퍼 닫기)
                  ↓
               DROP (드롭존으로 팔 이동 + 그리퍼 열기)
                  ↓
               RETURN (팔 홈 복귀) → IDLE
```

### 로봇팀 TODO

| 파일 | 항목 | 내용 |
|------|------|------|
| `main.py` | `ARRIVE_THRESHOLD_MM` | 실물 테스트 후 도착 판정 거리 조정 (현재 30mm) |
| `main.py` | `DROP_ZONES` (추후) | 드롭존 이동이 필요하면 Python에서 cls별 좌표 관리 |
| `gripper.ino` | `FINGER_OPEN_DEG` / `FINGER_CLOSE_DEG` | 실물 테스트 후 실제 각도 측정·수정 |
| `arm.ino` | 전체 구현 | 팔 관절 구성 확정 후 역기구학 + 동작 시퀀스 작성 |

### 필요 라이브러리 (Arduino IDE 라이브러리 매니저)

- **ArduinoJson** (Benoit Blanchon) — JSON 파싱 (`main.ino`)
- **Dynamixel2Arduino** (ROBOTIS) — Dynamixel 제어 (`gripper.ino`, `arm.ino`)

---

## 통신 프로토콜

### 1. Jetson → ESP32 (바퀴 제어)

**연결**: `/dev/ttyACM0`, 115200 baud, JSON per line  
> ⚠️ Jetson에서 CH343 USB 드라이버는 ttyUSB가 아닌 **ttyACM**으로 잡힘

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

**연결**: `/dev/ttyACM1`, 115200 baud, JSON per line

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
| d12 | 136장 | d12_2.mp4가 실제로는 d20이었음 → d20으로 이동 완료 |
| d20 | 185장 | |
| apple | 0장 | 미수집 |
| banana | 0장 | 미수집 |
| orange | 0장 | 미수집 |
| pineapple | 0장 | 미수집 |

> ✅ 대회 환경에서 재촬영 완료 (2026-06-25)  
> ✅ `best.pt` d6/d8/d12/d20 전체 클래스 학습 완료 (YOLOv8s, imgsz=640)

---

## 학습 파이프라인

```
1. Roboflow 라벨링 (바운딩박스)
      ↓
2. vision/train/train.ipynb (Google Colab)
      ↓
3. vision/model/best.pt 저장
      ↓
4. python vision/src/trt_export.py (Jetson)
      ↓
5. vision/model/best.engine → main.py에서 자동 사용
```

Colab 노트북 실행 전 필요한 것:
- Roboflow API 키
- Google Drive 마운트

---

## 2026-06-25 작업 내역

- **shape 전체 클래스 이미지 촬영 완료** — d6/d8/d12/d20 대회 환경에서 재촬영
- **Roboflow 라벨링 + Colab 재학습 → best.pt 갱신** — 전체 shape 클래스 학습 완료
- **Jetson 실기기 연결 및 구동 확인**
  - 핫스팟(172.20.10.5)으로 SSH 접속 성공
  - Arducam USB 카메라 동작 확인 (`/dev/video0`)
  - UGV02 바퀴 실제 구동 확인 (ESP32 `/dev/ttyACM0`, CH343 드라이버)
  - YOLO best.pt 모델 로드 및 실시간 탐지 동작 확인
- **ROS2 mobility_pkg 5개 노드 전체 실행 확인**
  - camera, yolo_vision, ugv_controller, main_decision, gripper 노드 launch 성공
  - yolo_vision_node에 best.pt 경로(`~/MERO_ROBOT_15/vision/model/best.pt`) 적용
- **ROS2 방식 채택 결정** — `vision/src/main.py` 단독 방식 대신 ROS2 노드 구조로 진행
- **main.py FPS 카운터 추가** — 1초마다 `[FPS] XX.X` 터미널 출력 + 화면 오버레이
- **전원 구성 결정**
  - 젯슨: 보조배터리(USB-C PD, 15V) — 5.5×2.1mm 배럴잭 확인 필요
  - XL430 팔: UGV02 내장 12V 배터리 → OpenRB
  - XL330 그리퍼: 5V 별도 공급 (벅컨버터 또는 5V USB)
- **포트 정리** — ESP32: `/dev/ttyACM0`, OpenRB: `/dev/ttyACM1` (CH343 → ACM 확인)
- **mobility_pkg 리포 추가** — `ros2/mobility_pkg/` 경로에 보관

---

## 완료된 작업 ✅

- [x] YOLOv8 실시간 트래킹 구현 (`model.track(persist=True)`)
- [x] 타겟 선택 로직 (신뢰도 최고 물체 1개 자동 선정)
- [x] 화면 시각화 (바운딩박스 + TARGET 노란 강조 + FPS 표시)
- [x] 카메라 캘리브레이션 코드 (`vision/src/calibration.py`) — 헤드리스 모드 지원
- [x] TensorRT 변환 스크립트 (`vision/src/trt_export.py`)
- [x] Colab 학습 노트북 (`vision/train/train.ipynb`)
- [x] d6/d8/d12/d20 이미지 촬영 및 파일명 정리 (총 581장)
- [x] Roboflow 라벨링 완료 (d6/d8/d12/d20)
- [x] **YOLOv8s 전체 클래스 학습** — `vision/model/best.pt` (22.5MB)
- [x] **듀얼 시리얼 통신 구현** (`vision/src/main.py`)
  - ESP32 `/dev/ttyACM0` — 바퀴 직접 제어 (Waveshare JSON)
  - OpenRB `/dev/ttyACM1` — 팔·그리퍼 pick/idle 명령 전송
- [x] **헤드리스 모드** — SSH 환경에서 DISPLAY 없이 실행 가능
- [x] **Jetson 실기기 테스트 완료** (2026-06-25)
  - SSH 접속: 핫스팟 172.20.10.5
  - Arducam USB 카메라 동작 확인 (`/dev/video0`, index 0)
  - UGV02 바퀴 동작 확인 (ESP32: `/dev/ttyACM0`, CH343 드라이버)
  - YOLO 모델 로드 및 탐지 확인
- [x] **OpenRB 메인 제어 코드** (`robot/main.ino`)
  - Jetson JSON 수신·파싱 (ArduinoJson)
  - 상태 머신 (IDLE → PICK → DROP → RETURN)
- [x] **그리퍼 코드** (`robot/gripper.ino`)
  - XL330 × 2 위치 제어 (Protocol 2.0, 57600 baud)
- [x] **팔 코드 스텁** (`robot/arm.ino`)
  - 함수 인터페이스 정의 완료

### 전원 구성 확정
- 젯슨: **보조배터리 → USB-C PD (15V, 5.5×2.1mm 확인 필요)**  
- XL430 팔: **UGV02 내장 12V 배터리 → OpenRB**  
- XL330 그리퍼: **5V 별도 공급** (벅컨버터 또는 보조배터리 USB 5V)  
- ⚠️ XL430은 최대 14.8V → 15V 직결 금지

---

## 미결 설계 이슈

### 보관함 이동 알고리즘 (미결정)

픽업 완료 후 보관함(좌측 하단 고정)까지 가는 방법 3가지 검토 중:

| 방법 | 설명 | 장점 | 단점 |
|------|------|------|------|
| **1. 깃대 인식** | YOLO로 보관함 옆 깃대 탐지 → 추적 이동 | 현재 구조에 자연스럽게 연결, robust | 깃대 데이터 촬영 + 재학습 필요, 실물 미공개 |
| **2. 고정 경로** | 픽업 후 왼쪽 회전 → 일정 시간 전진 → 드롭 | 즉시 구현 가능, 코드 단순 | 픽업 위치마다 오차 큼 |
| **3. 벽 따라가기** | 왼쪽 벽 향해 이동 → 코너까지 → 드롭 | 위치 무관하게 안정적 | 벽 인식 로직 추가 필요 |

**현재 방향**: 깃대 실물 공개 전까지 방법 2(고정 경로)로 구현 → 깃대 공개 후 방법 1으로 업그레이드

---

## 남은 작업 ⬜

### ROS2 (현재 채택된 방식)

| 우선순위 | 작업 | 비고 |
|----------|------|------|
| 🔴 높음 | `gripper_node.py` 구현 | `/gripper_cmd` 구독 → OpenRB 시리얼 전송 |
| 🔴 높음 | `main_decision_node.py` 수정 | 목표 도달 시 `/gripper_cmd` 발행 추가 |
| 🔴 높음 | Jetson robot_ws 업데이트 + `colcon build` | 위 수정 후 배포 |
| 🔴 높음 | OpenRB + Dynamixel 연결 및 동작 테스트 | 전원 구성 확정 후 |
| 🔴 높음 | `arm.ino` 구현 | 팔 관절 구성 확정 후 역기구학 작성 |
| 🔴 높음 | 전원 배선 완성 | 보조배터리(젯슨) + UGV배터리(팔) + 5V(그리퍼) |
| 🔴 높음 | 과일 데이터 촬영 (4종) | 현재 0장 |
| 🔴 높음 | 과일 클래스 라벨링 + 재학습 | 촬영 후 Roboflow → Colab |
| 🟡 중간 | 캘리브레이션 실행 | 카메라 높이·위치 확정 후 |
| 🟡 중간 | TensorRT 변환 (`best.pt` → `best.engine`) | Jetson에서 실행 |
| 🟡 중간 | `gripper.ino` 각도 실측 | `FINGER_OPEN_DEG` / `FINGER_CLOSE_DEG` 수정 |
| 🟡 중간 | 드롭존 좌표 실측 | robot 코드 수정 |
| 🟡 중간 | end-to-end 통합 테스트 | 탐지 → 이동 → pick → drop |
| 🟢 낮음 | 과일 클래스 Roboflow 라벨링 + 재학습 | 데이터 촬영 후 |
| 🟢 낮음 | FPS 확인 | TensorRT 변환으로 향상 예상 |

---

## 인수인계 시 체크리스트

이어서 작업하는 사람이 확인할 것:

1. `vision/model/best.pt` GitHub에 포함 (현재 d8만 학습된 임시본)
2. Roboflow 프로젝트 접근 권한 확인
3. Colab 노트북 실행 전 Roboflow API 키 입력
4. Jetson 연결 포트 확인:
   - `ls /dev/tty*` 실행
   - ESP32: `ttyUSB0` → `main.py`의 `ESP32_PORT` 맞춰 수정
   - OpenRB: `ttyACM0` → `OPENRB_PORT` 맞춰 수정
5. OpenRB Arduino 업로드 시 보드: **OpenRB-150** 선택
6. Dynamixel Wizard로 서보 ID 및 Baudrate 사전 설정 (57600 baud)
7. XL430 전원: OpenRB 초록 단자에 12V 배터리 연결 (두꺼운 전선 필수)
8. 캘리브레이션은 카메라 설치 높이 확정 후 1회 실행
