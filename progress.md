# MERO AI ROBOT — Progress

> 최종 업데이트: 2026-07-15  
> 비전 담당: 조강희

---

## 프로젝트 개요

카메라로 물체를 실시간 탐지·분류·트래킹 후 집게로 집어서 목표 지점으로 이동하는 pick-and-place 대회.  
이 리포는 **비전팀 + 로봇팀 코드** 모두 포함.

**하드웨어**
- 보드: NVIDIA Jetson Orin Nano
- 로봇 플랫폼: Waveshare UGV02 (내장 컨트롤러: ESP32 — 바퀴 제어)
- 팔·그리퍼 컨트롤러: ROBOTIS OpenRB-150 (Dynamixel 제어)
- 다이나믹셀: XL430 × 5, 모두 12V, Baudrate 1000000
  - ID 1: 그리퍼 (랙-피니언, 양 손가락 구동)
  - ID 2: 팔 관절 모터 × 2 (같은 ID — 한쪽 Drive Mode Reverse 설정)
  - ID 3: 바스켓 힌지 모터 × 2 (같은 ID — 동일 방식)
- 카메라: ArduCAM 2.3MP AR0234 글로벌 셔터 USB 3.0

**대회 태스크**
- shape-based: d6, d8, d12, d20 (다면체 주사위)
- image-based: apple, banana, orange, pineapple

---

## 로봇 테스트 당일 — 단계별 진행 순서

> 처음 테스트하는 사람 기준. 순서대로 따라하면 됨.

### STEP 1. 하드웨어 연결 (물리)

1. **젯슨 전원**: 파워뱅크(모루이 MT-65) → USB-C PD 케이블(15V) → 젯슨 배럴잭
2. **그리퍼 전원**: UGV 내장 12V 배터리 → OpenRB-150 초록 단자 (두꺼운 전선)
3. **XC330 연결**: XC330 Dynamixel 케이블 → OpenRB-150 Dynamixel 포트
4. **USB 연결**: 젯슨 ↔ ESP32(UGV), 젯슨 ↔ OpenRB-150 (USB-A to USB-B)
5. **카메라**: ArduCAM USB → 젯슨 USB 3.0 포트

### STEP 2. Dynamixel Wizard (PC, 처음 1회만)

> 목적: XL430의 ID와 통신 속도가 맞는지 확인

1. PC에 Dynamixel Wizard 2.0 설치 (ROBOTIS 공식 사이트)
2. OpenRB-150을 PC에 USB 연결
3. Scan → XL430들이 아래 ID와 Baudrate로 보이는지 확인:
   - **ID=1, Baudrate=1000000** (그리퍼)
   - **ID=2, Baudrate=1000000** (팔 관절 × 2 — 한쪽은 Drive Mode: Reverse 설정)
   - **ID=3, Baudrate=1000000** (바스켓 힌지 × 2 — 동일)
4. 다르면 Wizard에서 ID/Baudrate 변경 후 저장

### STEP 3. Arduino 코드 업로드 (PC, 처음 1회만)

> 목적: OpenRB-150에 그리퍼 제어 코드 올리기

1. Arduino IDE 실행
2. `robot/main.ino` 열기 (같은 폴더에 `gripper.ino`, `arm.ino` 있어야 함)
3. **보드 선택**: Tools → Board → OpenRB-150
4. **포트 선택**: Tools → Port → OpenRB-150 잡힌 COM 포트
5. 업로드 (→ 버튼)
6. 라이브러리 오류 시: Library Manager에서 `ArduinoJson`, `Dynamixel2Arduino` 설치

### STEP 4. 젯슨 SSH 접속 (MobaXterm)

> 핫스팟을 쓰면 접속자마다 IP가 달라짐. 아래 순서로 IP 먼저 확인.

#### 4-1. IP 확인 방법

**방법 A — 아이폰 핫스팟 사용 시 (가장 빠름)**
1. 아이폰 설정 → 개인용 핫스팟 → 연결된 기기 목록에서 "aiwinners" IP 확인

**방법 B — 젯슨에 모니터/키보드 직접 연결**
```bash
hostname -I   # 첫 번째 IP가 핫스팟 IP
```

**방법 C — PC에서 네트워크 스캔**
```bash
# 아이폰 핫스팟 기본 대역: 172.20.10.x
# Windows PowerShell 또는 MobaXterm 터미널에서:
nmap -sn 172.20.10.0/24
# "aiwinners" 또는 "NVIDIA" 이름으로 찾기
```

#### 4-2. MobaXterm SSH 연결

1. MobaXterm 실행 → **Session** → **SSH**
2. Remote host: `172.20.10.x` (위에서 확인한 IP)
3. Username: `aiwinners`
4. Port: 22
5. OK → 비밀번호 입력: `mero1234`

#### 4-3. 카메라 스트림 보기 (브라우저)

main.py 실행 후 PC 브라우저에서:
```
http://172.20.10.x:8080
```
> X는 젯슨 IP 마지막 자리

### STEP 5. USB 권한 열기 (매번 필요)

```bash
# 젯슨 터미널에서
sudo chmod 666 /dev/ttyACM0   # ESP32 (UGV 바퀴)
sudo chmod 666 /dev/ttyACM1   # OpenRB (그리퍼)

# 포트 확인 (꽂는 순서에 따라 0/1 바뀔 수 있음)
ls /dev/ttyACM*
```

### STEP 6. 메인 코드 실행

```bash
# 젯슨 터미널에서
cd ~/MERO_AI_ROBOT
python vision/src/main.py            # 기본 실행 (모든 클래스 탐지)
python vision/src/main.py --cls d8   # d8만 픽업
python vision/src/main.py --timer    # 3분 타이머 화면 표시
```

> 카메라 화면이 뜨고 탐지 박스가 보이면 정상.  
> 포트 오류 시: `vision/src/main.py` 상단 `ESP32_PORT` / `OPENRB_PORT` 값 확인

### STEP 7. 실측값 현황 (2026-07-15 기준)

| 항목 | 파일 | 확정값 | 상태 |
|------|------|--------|------|
| `FINGER_OPEN_DEG` | `robot/gripper.ino` | 265° | ✅ 완료 |
| `FINGER_CLOSE_DEG` | `robot/gripper.ino` | 110° | ✅ 완료 |
| `GRIP_LOAD_THRESHOLD` | `robot/gripper.ino` | 200 (20%) | ✅ 완료 |
| `AREA_THRESHOLD` | `vision/src/main.py` | 28000 | ⬜ 실측 필요 |
| `AREA_SLOW_THRESHOLD` | `vision/src/main.py` | 20000 | ⬜ 실측 필요 |
| `CENTER_MARGIN_PX` | `vision/src/main.py` | 120px | ✅ 완료 |
| `ENCODER_TICKS_PER_M` | `vision/src/encoder_test.py` | **105.2** | ✅ 완료 |
| `ARM_DOWN_RAW` | `robot/arm.ino` | 0 (placeholder) | ⬜ Wizard로 실측 |
| `ARM_UP_RAW` | `robot/arm.ino` | 1706 (placeholder) | ⬜ Wizard로 실측 |
| `CONTAINER_CLOSED_RAW` | `robot/arm.ino` | 0 (placeholder) | ⬜ Wizard로 실측 |
| `CONTAINER_OPEN_RAW` | `robot/arm.ino` | 1024 (placeholder) | ⬜ Wizard로 실측 |
| `FLAG_AREA_THRESHOLD` | `vision/src/main.py` | 60000 | ⬜ 3m 거리에서 실측 |
| `FLAG_AREA_SLOW_THRESHOLD` | `vision/src/main.py` | 30000 | ⬜ 동일 |
| `STORAGE_BACKUP_SECS` | `vision/src/main.py` | 0.8s | ⬜ 미실측 |

### IMU 특성 (2026-07-08 실측)

- **T=126 없음** — IMU 데이터 모두 T=1001에 포함 (gx/gy/gz/mx/my/mz)
- **지자기(mx/my) 사용 불가** — 모터 전류 간섭으로 회전 중 값 뒤틀림
- **자이로(gz) 적분 방식** — GZ_SCALE=16.5, 관성 오버슈트 ~17° 있음
- **ESP32 watchdog** — 약 3초 명령 없으면 모터 자동 정지 → 0.1s 주기 재전송 필수

### STEP 8. 캘리브레이션 (선택, 카메라 위치 확정 후)

```bash
# 1. 사진 촬영 (SSH 환경)
python vision/src/calibration.py --capture
# → vision/model/calib_frame.jpg 생성됨

# 2. PC에서 calib_frame.jpg 열어서 기준 물체 양 끝 픽셀 좌표 확인

# 3. 좌표 입력해서 비율 계산
python vision/src/calibration.py --calc x1 y1 x2 y2 실제거리mm
# 예: python vision/src/calibration.py --calc 120 300 540 300 210
# → vision/model/calibration.json 생성 → main.py 자동 적용
```

---

## 구매 내역

### 구매 내역

| 품목 | 제품명 | 용도 | 금액 |
|------|--------|------|------|
| SSD | SK하이닉스 Gold P31 M.2 NVMe 1TB | 젯슨 저장장치 | 125,636원 |
| 카메라 | ArduCAM 2.3MP AR0234 글로벌 셔터 USB 3.0 [B0495C] | 물체 탐지 | 276,540원 |
| 모빌리티 | Waveshare 6x4 Off-Road UGV (Extension Rails, ESP32) 6WD | 로봇 본체 | 262,190원 |
| 배터리 | 삼성 INR18650-30Q 3.6V 3000mAh | UGV 배터리 (설치 완료) | 48,000원 |
| 다이나믹셀 | XL430 × 6 + XC330 × 2 | 팔 관절 + 그리퍼 | 190,000원 |
| 필라멘트 | BambuLab PLA Basic 1kg (흰색) | 팔·구조물 3D프린팅 | 22,000원 |
| 파워뱅크 | 모루이 MT-65 65W 20000mAh | 젯슨 전원 | 34,800원 |
| DC 전원 케이블 | USB C타입 To DC 전원케이블 15V/45W (케이엠에스파트너) | 파워뱅크 → 젯슨 배럴잭 | 24,500원 |
| OpenRB-150 | ROBOTIS OpenRB-150 (로봇데스크 종합몰) | Dynamixel 컨트롤러 (젯슨↔팔·그리퍼) | 31,000원 |
| | | **합계** | **1,014,666원** |

---

## 시스템 아키텍처

```
┌──────────────┐     /dev/ttyACM0 (CH343→ACM)      ┌─────────────┐
│   Jetson     │ ──── {"T":1,"L":spd,"R":spd} ────▶ │    ESP32    │ → 바퀴 모터
│  Orin Nano   │                                    └─────────────┘
│  (vision/    │
│  src/main.py)│     /dev/ttyACM1                   ┌─────────────────────────────────┐
│              │ ──── {"cmd":"grip"/"dump"} ────────▶ │  OpenRB-150                     │
│              │ ◀─── {"status":"gripped"/           │  → ID1: XL430 그리퍼 (랙-피니언) │
│              │       "grip_failed"/"dumped"} ─────  │  → ID2: XL430 팔 관절 × 2       │
└──────────────┘                                    │  → ID3: XL430 바스켓 힌지 × 2   │
      ▲▲                                            └─────────────────────────────────┘
      ││ ArduCAM USB
  전방 카메라 (/dev/video0, CAMERA_INDEX_OBJ=0)  — 물체 탐지
  후방 카메라 (/dev/video2, CAMERA_INDEX_FLAG=2) — 태극기 탐지 (GO_TO_STORAGE)
```

**Jetson에서 나가는 신호 두 가지:**
1. `/dev/ttyACM0` → ESP32: 바퀴 속도 명령 `{"T":1,"L":...,"R":...}` (CH343 드라이버 → ACM)
2. `/dev/ttyACM1` → OpenRB: 명령 (`grip` / `dump` / `idle`)

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
python vision/src/calibration.py      # 1. 카메라 캘리브레이션 (1회, 선택)
python vision/src/trt_export.py       # 2. TensorRT 변환 (1회, 선택)
python vision/src/main.py             # 3. 메인 실행 (캘리브 없어도 동작)

# 경기 당일 — 타겟 클래스 지정 (오전 공지 후)
python vision/src/main.py --cls d8    # 예: d8만 픽업
python vision/src/main.py --cls apple # 예: apple만 픽업
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
├── robot/                         # 로봇팀 (OpenRB Arduino — 그리퍼만)
│   ├── main.ino                   # JSON 수신 + 상태 머신
│   └── gripper.ino                # XC330 × 1 그리퍼 (랙-피니언)
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
| `vision/model/best.pt` | 학습된 모델 가중치 (d6/d8/d12/d20 전체 학습 완료, YOLOv8s) |

---

## 파일별 역할 (robot/)

| 파일 | 역할 |
|------|------|
| `robot/main.ino` | Jetson grip/dump 명령 수신 → 상태 머신 실행, Dynamixel 인스턴스 선언 |
| `robot/gripper.ino` | ID1 XL430 그리퍼 제어 (랙-피니언). PRESENT_LOAD 기반 집기 감지 |
| `robot/arm.ino` | ID2 XL430 팔 관절 + ID3 XL430 바스켓 힌지 제어 |

> 바퀴 제어(ESP32)는 `vision/src/main.py`의 `control_wheels()`가 직접 담당.

### 상태 머신 흐름

```
[Python main.py]                          [OpenRB main.ino]

SEARCHING                                 IDLE
  탐지 + 이동 (바퀴 제어)
  ↓ 도달 (bbox 면적 ≥ AREA_THRESHOLD)
  grip 명령 전송 ──────────────────────▶ GRIPPING
GRIPPING                                    그리퍼 닫기 (PRESENT_LOAD 감지)
  바퀴 정지, 신호 대기                 ◀── {"status":"gripped"}     → LIFTING (팔 올리기 → 바스켓 투하 → 팔 내림 → IDLE)
  ├─ gripped → SEARCHING (반복 수집)   ◀── {"status":"grip_failed"} → IDLE
  ├─ grip_failed → SEARCHING (복귀)
  └─ timeout(15s) → SEARCHING (복귀)

  ※ 목표 클래스 전부 max_count 달성 시 → GO_TO_STORAGE

GO_TO_STORAGE                             IDLE
  phase 0: 제자리 회전하며 태극기 탐색 (후방 카메라 + flag.pt)
  phase 1: 태극기 보이면 후진하며 접근 → FLAG_AREA_THRESHOLD 도달 시
  dump 명령 전송 ──────────────────────▶ DUMPING (바스켓 힌지 열어 내용물 쏟기)
DROPPING                                 ◀── {"status":"dumped"} → IDLE
  바퀴 정지, dumped 신호 대기
  ├─ dumped → SEARCHING (복귀)
  └─ timeout(30s) → SEARCHING (복귀)
```

### 로봇팀 TODO

| 파일 | 항목 | 내용 |
|------|------|------|
| `main.py` | `AREA_THRESHOLD` | 실물 테스트 후 도착 면적 임계값 조정 (현재 28000) |
| `arm.ino` | `ARM_DOWN_RAW` / `ARM_UP_RAW` | Dynamixel Wizard Present Position 읽어 실측 |
| `arm.ino` | `CONTAINER_CLOSED_RAW` / `CONTAINER_OPEN_RAW` | 동일 방법 실측 |
| `main.py` | `FLAG_AREA_THRESHOLD` | 3m 거리에서 태극기 bbox 면적 실측 |
| `main.py` | `STORAGE_BACKUP_SECS` | 집은 자리에서 후진 후 회전 공간 확인 |

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

### 2. Jetson → OpenRB (팔·그리퍼·바스켓 제어)

**연결**: `/dev/ttyACM1`, 115200 baud, JSON per line

```json
{"cmd": "grip", "cls": "d8"}  ← 집기 (IDLE→GRIPPING→LIFTING→IDLE, 바스켓에 투하)
{"cmd": "dump"}                ← 바스켓 힌지 열어 내용물 쏟기 (IDLE→DUMPING→IDLE)
{"cmd": "idle"}                ← 대기
```

**OpenRB → Jetson 응답:**
```json
{"status": "gripped"}      ← 집기+바스켓 투하 완료 (Python SEARCHING 복귀)
{"status": "grip_failed"}  ← 집기 실패 — Load 미달 (Python SEARCHING 복귀)
{"status": "dumped"}       ← 바스켓 비우기 완료 (Python SEARCHING 복귀)
```

### 3. OpenRB → Dynamixel (팔·그리퍼·바스켓 직접)

Dynamixel2Arduino 라이브러리 사용. Protocol 2.0, Baudrate 1000000.  
OpenRB 내장 Dynamixel 포트 (`Serial1`) 사용 — 방향핀 별도 불필요.

| 서보 | ID | 모델 | 전원 | 역할 |
|------|-----|------|------|------|
| 그리퍼 (랙-피니언, 양 손가락) | 1 | XL430 | 12V | gripper.ino |
| 팔 관절 × 2 (같은 ID) | 2 | XL430 | 12V | arm.ino |
| 바스켓 힌지 × 2 (같은 ID) | 3 | XL430 | 12V | arm.ino |

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
| d6 | 85장 | |
| d8 | 150장 | |
| d12 | 136장 | |
| d20 | 185장 | |
| apple | 163장 | ⚠️ 실제 과일 영상 추출 — 대회용 아님. 과일 이미지 붙인 **흰색 큐브** 촬영 후 교체 필요 |
| banana | 146장 | ⚠️ 동일 |
| orange | 0장 | 미수집 |
| pineapple | 0장 | 미수집 |

> ✅ 대회 환경에서 재촬영 완료 (2026-06-25)  
> ✅ `best.pt` d6/d8/d12/d20 전체 클래스 학습 완료 (YOLOv8s, imgsz=640)  
> ⚠️ `vision/DATASET/` 는 .gitignore 제외 — 깃에 올라가지 않음

---

## 학습 파이프라인

**기본 (수동 라벨링):**
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

**대안 — SAM2 자동 라벨링 (과일큐브 권장):**
```
1. 과일큐브 영상 촬영 (30초~1분짜리)
      ↓
2. Colab에서 SAM2 실행
   - 1프레임에서 큐브 클릭 한 번
   - → 나머지 전 프레임 자동 마스크 전파
      ↓
3. YOLO 포맷으로 annotation export
      ↓
4. train.ipynb에서 YOLOv8s fine-tuning (기존 방식 동일)
      ↓
5. vision/model/best.pt 교체
```

> 수동 Roboflow 라벨링 대비 시간 대폭 절감. 모델 자체는 YOLOv8s 유지 (Jetson TensorRT 호환).

**성능 비교 방법 (old vs new best.pt):**
```bash
# 같은 validation 이미지로 두 모델 mAP 비교
yolo val model=vision/model/best_old.pt data=data.yaml   # 기존
yolo val model=vision/model/best.pt     data=data.yaml   # 신규
# → mAP@50 수치 비교. 필드 테스트(대회 조명/배경)가 더 중요
```

Colab 노트북 실행 전 필요한 것:
- Roboflow API 키
- Google Drive 마운트

---

## 2026-07-15 작업 내역

- **전체 코드 디버깅 완료** (OpenRB 3파일 + vision 3파일)
  - `GOAL_PWM` → `ControlTableItem::PWM_LIMIT` (address 36): GOAL_PWM은 PWM Control Mode 전용 레지스터로 Position Control Mode에서 동작 안 함 — `robot/arm.ino`, `robot/gripper.ino`
  - `using namespace ControlTableItem` 명시적 namespace: Arduino IDE가 .ino 파일을 알파벳순(arm→gripper→main) 병합하므로 main.ino의 namespace 선언이 arm/gripper에 미적용 — `ControlTableItem::PWM_LIMIT`으로 명시 수정
  - `strlcpy` → `strncpy` + 수동 null terminator: STM32/newlib-nano 환경에서 `strlcpy` 미지원 — `robot/main.ino`
  - GO_TO_STORAGE 태극기 놓침 시 정지 누락: phase 1→0 복귀 시 `control_wheels(None)` 없어 로봇이 계속 후진 — `vision/src/main.py`
  - `_last_print_t` 미업데이트: 타겟 없을 때 탐지 print 매 루프 스팸 — `vision/src/main.py`
  - `calibration.py` CAMERA_INDEX `1→0`: main.py CAMERA_INDEX_OBJ=0과 불일치
  - `launcher.py` 하드코딩 경로 제거: `/home/aiwinners/MERO_ROBOT_15/...` → `__file__` 기반 동적 경로, `import os` 추가

- **하드웨어 확정** (XL430 × 5, ID 1/2/3)
  - ID1=그리퍼, ID2=팔 관절×2, ID3=바스켓 힌지×2
  - 두 모터 같은 ID 방법: Dynamixel Wizard에서 한쪽 Drive Mode Reverse 설정

- **설계 결정 확정**
  - 거리 판단: 픽셀 면적 모드 (캘리브레이션 미사용)
  - GO_TO_STORAGE: 후방 카메라 + flag.pt 태극기 탐지 방식
  - 경기 시작: MobaXterm 명령 미리 입력 후 Enter

---

## 2026-07-08 작업 내역

- **보관함 이동 로직 개선** (`vision/src/main.py`)
  - IMU 기반 방향 회전: 시작 시 yaw 기록 → GO_TO_STORAGE에서 `storage_yaw(=initial_yaw-90°)`까지 IMU로 회전
  - 엔코더 기반 직진 거리 측정: T=1001 `lp`/`rp` 필드 파싱 → `ENCODER_TICKS_PER_M` 기준 4.5m 도달 시 정지
  - IMU/엔코더 미획득 시 고정 시간 fallback 자동 적용
- **실측 필요 항목** (아래 표 참고)

---

## 2026-07-06 작업 내역

- **launcher.py 추가** (`vision/src/launcher.py`) — 물리 버튼 3개 + SSD1306 OLED로 독립 운용
  - 버튼A: shape 순환 (d6→d8→d12→d20)
  - 버튼B: fruit 순환 (apple→banana→orange→pineapple)
  - 버튼C: 시작/정지 (main.py subprocess 실행)
  - OLED 128×64: 선택 클래스 + 상태(READY/RUNNING/DONE) 표시
  - GPIO 없을 때 키보드(s/f/Enter)로 테스트 가능
- **launcher 부품 구매 목록**
  - SSD1306 OLED 0.96" 4핀 I2C (GND/VCC/SCL/SDA)
  - 4핀 택트 스위치 × 3
  - 미니 브레드보드 35×46mm
  - 점퍼선 암수 × 10개
- **OpenRB 배터리 구매** — 폴리트로닉스 PT-B2200N-SP35 (11.1V 3S LiPo, 2200mAh, XT60)
  - XT60 → OpenRB 터미널 블록 연결 필요

---

## 2026-07-05 작업 내역

- **하드웨어 정정**: 그리퍼 모터 XC330 → **XL430** (실물 확인), Baudrate 57600 → **1000000**
- **그리퍼 실측 완료**
  - `FINGER_OPEN_DEG` = 265° (raw 2700 측정)
  - `FINGER_CLOSE_DEG` = 110° (실측)
  - `GRIP_LOAD_THRESHOLD` = 200 (PRESENT_LOAD 기반, ~20%)
- **gripper.ino 전류→Load 방식으로 변경** — `PRESENT_LOAD` 기반 집기 판별
- **main.ino** — `using namespace ControlTableItem` 추가, baudrate 수정
- **카메라 해상도 자동 감지** (`vision/src/main.py`) — calibration.json 없을 때 FRAME_W 640 고정 버그 수정 (실제 960×600)
- **Jetson 실기기 연결 테스트**
  - SSH: `aiwinners@172.20.10.5` (핫스팟)
  - ArduCAM: `/dev/video0` (960×600), ESP32: `/dev/ttyACM0`, OpenRB: `/dev/ttyACM1`
  - YOLO 탐지 + 바퀴 이동 동작 확인
  - 그리퍼 집기 동작 확인

---

## 2026-06-30 작업 내역

- **시리얼 포트 자동 감지 추가** (`vision/src/main.py`)
  - `/dev/serial/by-id/` USB ID 기반 자동 매칭 (CH343 → ESP32, OpenRB → 그리퍼)
  - 케이블 꽂는 순서 바뀌어도 자동으로 올바른 포트 사용
  - 탐지 실패 시 기본값(`/dev/ttyACM0`, `/dev/ttyACM1`) fallback

- **경기 타이머 추가** (`vision/src/main.py`)
  - `--timer` 플래그로 화면 중앙 상단에 3분 카운트다운 표시
  - 60초 미만 주황, 30초 미만 빨강으로 색상 변화
  - 예: `python vision/src/main.py --cls d8 --timer`

- **룰북 검토 완료**
  - 세트1 4개(10점×4=40점) + 세트2 3개(20점×3=60점) = 100점 달성 조건 확인
  - `--cls d8 apple` 처럼 두 클래스 동시 지정으로 100점 도전 가능
  - 과일큐브 = d6와 외형 동일 → 모델이 과일 이미지로 구분해야 함

- **과일 데이터 현황 파악**
  - apple(163장)/banana(146장) 영상 프레임 추출 완료 (`vision/DATASET/image-based/`)
  - ⚠️ 실제 과일 영상 — 대회 물체(과일 이미지 붙인 흰색 큐브)와 다름 → 큐브 촬영으로 교체 필요

---

## 2026-06-27 작업 내역

- **전류 기반 집기 성공 감지 추가** (`robot/gripper.ino`, `robot/main.ino`, `vision/src/main.py`)
  - `gripperClose()` → `bool gripperClose()`: 닫힌 후 XC330 전류 읽어 임계값 비교
  - 전류 ≥ `GRIP_CURRENT_THRESHOLD`(현재 30mA) → 잡음, 미달 → 미스
  - 집기 실패 시: 그리퍼 열고 `armHome()` → `{"status":"grip_failed"}` 전송 → Python SEARCHING 복귀
  - `GRIP_CURRENT_THRESHOLD` 실측 조정 필요 (빈 손 vs 물체 잡기 전류 차이 측정)

- **안전장치 2종 추가** (`vision/src/main.py`)
  - `GO_TO_STORAGE` 전체 타임아웃 15초: 이동 중 로봇 stuck 시 SEARCHING 자동 복귀
  - 카메라 프레임 실패 처리: 1회 실패 시 즉시 종료 → 연속 10회 실패 시 종료로 변경 (글리치 내성)

---

## 2026-06-26 작업 내역

- **프레임워크 최종 결정: Python 단독 방식** (`vision/src/main.py`)
  - ROS2 검토 후 러닝커브 부담으로 단독 Python 방식으로 결정
  - `ros2/` 폴더는 레퍼런스로 보관, 메인 코드는 `vision/src/main.py`

- **`main.py` 전면 재설계 완료**
  - `--cls` 인수 추가: 경기 당일 타겟 클래스 지정 (`python main.py --cls d8`)
  - **4단계 상태 머신 구현**: SEARCHING → GRIPPING → GO_TO_STORAGE → DROPPING
  - **calibration 없이도 동작**: bbox 면적 기반 픽셀 모드 추가
    - `calibration.json` 있으면 mm 기반, 없으면 자동으로 면적 기반 전환
    - 도달 판단: 면적 ≥ 40000 (픽셀모드) 또는 거리 < 30mm (mm모드)
  - 보관함 고정 경로 이동 구현: ① 좌회전 N초 → ② 직진 N초 (실측 조정 필요)
  - grip / drop 명령 분리: gripped/done 완료 신호 각각 수신

- **`robot/main.ino` 재설계**
  - 상태 머신: IDLE → GRIPPING → HOLDING → DROPPING → RETURNING
  - grip/drop 명령 분리 처리 (기존 단일 pick 명령에서 변경)
  - Dynamixel 인스턴스(`dxl`)를 main.ino에서 선언해 arm/gripper 공유

- **`robot/gripper.ino` 수정**
  - extern dxl 참조로 변경 (중복 정의 제거)
  - 헤더 오타 수정: XL430 → XC330

- **`robot/arm.ino` 구조 정리**
  - `armTransport()` 추가: 물체 든 채 이동 자세 (주행 중 팔 접기)
  - 함수명 정리: `armPickUp` / `armTransport` / `armDrop` / `armHome`
  - 관절 ID 상수 정의 (ID 3~8, 실측 후 수정)

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
  - XC330 그리퍼: 12V (UGV 배터리 공유) (벅컨버터 또는 5V USB)
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
  - 상태 머신 (IDLE → GRIPPING → HOLDING → DROPPING → IDLE)
  - grip/drop 명령 분리, gripped/grip_failed/done 응답 신호 전송
- [x] **그리퍼 코드** (`robot/gripper.ino`)
  - XC330 × 1 위치 제어 (Protocol 2.0, 57600 baud), 랙-피니언 단일 모터
  - 전류 기반 집기 감지 (gripperClose() bool 반환)
- [x] **`vision/src/main.py` 완성** (2026-06-26)
  - 4단계 상태 머신 (SEARCHING/GRIPPING/GO_TO_STORAGE/DROPPING)
  - calibration 유무 자동 감지 (mm 모드 / 픽셀 면적 모드)
  - `--cls` 인수로 경기 당일 타겟 클래스 지정
  - 보관함 고정 경로 이동 구현 (후진→좌회전→직진, 시간 기반)
- [x] **전류 기반 집기 감지** (2026-06-27)
  - `gripper.ino`: `gripperClose()` bool 반환, PRESENT_CURRENT 읽어 임계값 비교
  - `main.ino`: 집기 실패 시 `grip_failed` 신호 전송, IDLE 복귀
  - `main.py`: `grip_failed` 수신 처리 → SEARCHING 복귀
- [x] **안전장치** (2026-06-27)
  - GO_TO_STORAGE 전체 타임아웃 (15초, stuck 방지)
  - 카메라 프레임 연속 실패 10회 시 종료 (글리치 내성)

### 전원 구성 확정
- 젯슨: **보조배터리 → USB-C PD (15V, 5.5×2.1mm 확인 필요)**  
- XL430 팔: **UGV02 내장 12V 배터리 → OpenRB**  
- XC330 그리퍼: **5V 별도 공급** (벅컨버터 또는 보조배터리 USB 5V)  
- ⚠️ XL430은 최대 14.8V → 15V 직결 금지

---

## 설계 확정 사항

### GO_TO_STORAGE: 태극기 기반 후진 접근 (2026-07-15 확정)

- **후방 카메라** (CAMERA_INDEX_FLAG=2) + **flag.pt** 모델로 태극기 탐지
- phase 0: 제자리 회전하며 태극기 탐색
- phase 1: 태극기 보이면 후진하며 접근 → FLAG_AREA_THRESHOLD 도달 시 dump
- **바닥 검정 선은 무시** — 구역 구분선일 뿐 경로 추적 불필요
- flag.pt 미학습 시 GO_TO_STORAGE 진입 즉시 SEARCHING 복귀

### 거리 판단: 픽셀 모드 (2026-07-15 확정)

- **캘리브레이션 미사용** — 카메라가 대각선 설치라 단일 mm_per_pixel로 정확한 매핑 불가
- **bbox 면적(area) 기반** 도달 판단 → AREA_THRESHOLD 실측 조정

### 경기 시작

MobaXterm에서 명령 미리 입력 후 신호 오면 Enter:
```bash
python vision/src/main.py --cls d8 --timer
```

---

## 남은 작업 ⬜

### 실측 (하드웨어 필요)

| 우선순위 | 항목 | 현재값 | 측정 방법 | 수정 위치 |
|----------|------|--------|-----------|-----------|
| ✅ 완료 | `FINGER_OPEN_DEG` | **265°** | 실측 완료 | `gripper.ino` |
| ✅ 완료 | `FINGER_CLOSE_DEG` | **110°** | 실측 완료 | `gripper.ino` |
| ✅ 완료 | `GRIP_LOAD_THRESHOLD` | **200** (Load 20%) | 실측 완료 | `gripper.ino` |
| ✅ 완료 | `ENCODER_TICKS_PER_M` | **105.2** | 실측 완료 | `main.py` |
| 🔴 높음 | `ARM_DOWN_RAW` | 0 (placeholder) | Dynamixel Wizard Present Position 확인 | `arm.ino` |
| 🔴 높음 | `ARM_UP_RAW` | 1706 (placeholder) | 동일 | `arm.ino` |
| 🔴 높음 | `CONTAINER_CLOSED_RAW` | 0 (placeholder) | 동일 | `arm.ino` |
| 🔴 높음 | `CONTAINER_OPEN_RAW` | 1024 (placeholder) | 동일 | `arm.ino` |
| 🔴 높음 | `AREA_THRESHOLD` | 28000 | 물체 바로 앞에서 터미널 area= 값 확인 | `main.py` |
| 🔴 높음 | `FLAG_AREA_THRESHOLD` | 60000 | 보관함 3m 거리에서 태극기 bbox 면적 측정 | `main.py` |
| 🔴 높음 | `FLAG_AREA_SLOW_THRESHOLD` | 30000 | 동일 | `main.py` |
| 🟡 중간 | `CAMERA_INDEX_OBJ` | 0 | `/dev/video*` 번호 실제 확인 | `main.py` |
| 🟡 중간 | `CAMERA_INDEX_FLAG` | 2 | 동일 | `main.py` |
| 🟡 중간 | `STORAGE_BACKUP_SECS` | 0.8s | 집은 자리에서 후진 후 공간 확인 | `main.py` |
| 🟡 중간 | 팔/바스켓 delay 값 | armUp 1200ms 등 | 실제 모터 속도와 맞는지 확인 후 조정 | `arm.ino` |

### 데이터 / 학습

| 우선순위 | 작업 | 방법 |
|----------|------|------|
| 🔴 높음 | 태극기(flag.pt) 학습 | 태극기 사진 20~30장 촬영 → Roboflow Smart Polygon 라벨링 → Colab 학습 → `vision/model/flag.pt` |
| 🔴 높음 | 과일큐브 촬영 | 흰색 큐브에 과일 이미지 부착 후 ArduCAM으로 촬영 (실제 과일 X) — apple/banana 교체 + orange/pineapple 신규 |
| 🔴 높음 | 과일 클래스 라벨링 + 재학습 | **Roboflow Smart Polygon (SAM 기반) 권장** → `train.ipynb` (Colab) → `best.pt` 교체 |
| 🟡 중간 | TensorRT 변환 | Jetson에서: `python vision/src/trt_export.py` → `best.engine` 생성 (FPS 향상) |

### 테스트

| 우선순위 | 작업 | 확인 항목 |
|----------|------|-----------|
| 🔴 높음 | end-to-end 통합 테스트 | 탐지 → 이동 → grip → 팔 올리기 → 바스켓 투하 → SEARCHING 반복 → GO_TO_STORAGE → dump 전체 사이클 |
| 🟡 중간 | 클래스 오인식 대응 | d8↔d12 flickering 발생 시 N-frame majority voting 구현 검토 |

---

## 인수인계 시 체크리스트

이어서 작업하는 사람이 확인할 것:

1. `vision/model/best.pt` GitHub에 포함 ✅ (d6/d8/d12/d20 전체 학습 완료)
2. `vision/model/flag.pt` — **미학습** ⬜ (태극기 20~30장 촬영 후 학습 필요)
3. Roboflow 프로젝트 접근 권한 확인
4. Colab 노트북 실행 전 Roboflow API 키 입력
5. Jetson 연결 포트 확인:
   - `ls /dev/ttyACM*` 실행
   - ESP32: `ttyACM0` → `main.py`의 `ESP32_PORT` 확인 (CH343 드라이버, ttyUSB 아님)
   - OpenRB: `ttyACM1` → `OPENRB_PORT` 확인 (꽂는 순서에 따라 바뀔 수 있음)
   - 카메라: `/dev/video0` (전방), `/dev/video2` (후방) — 실제 번호 확인 후 `CAMERA_INDEX_OBJ`, `CAMERA_INDEX_FLAG` 수정
6. OpenRB Arduino 업로드 시 보드: **OpenRB-150** 선택 (`main.ino` + `gripper.ino` + `arm.ino` 같은 폴더)
7. Dynamixel Wizard로 서보 ID 및 Baudrate 사전 설정:
   - ID1 (그리퍼), ID2 (팔 관절×2), ID3 (바스켓 힌지×2), 모두 **Baudrate=1000000**
   - ID2/ID3: 한쪽 모터 Drive Mode → Reverse 설정 필수
8. XL430 전원: OpenRB 초록 단자에 12V 배터리 연결 (두꺼운 전선 필수)
9. `arm.ino` placeholder 값 (ARM_DOWN_RAW 등) 실측 후 수정 필수
10. `AREA_THRESHOLD`, `FLAG_AREA_THRESHOLD` 실물 테스트로 측정 후 `main.py` 수정
