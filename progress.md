# MERO AI ROBOT — Progress

> 최종 업데이트: 2026-07-21  
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
2. **그리퍼·팔·컨테이너 전원**: OpenRB 전용 별도 배터리(폴리트로닉스 PT-B2200N-SP35, 11.1V 3S LiPo, 2200mAh, XT60) → OpenRB-150 초록 단자 (두꺼운 전선) — UGV 내장 배터리 아님
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
2. `robot/robot.ino` 열기 (같은 폴더에 `gripper.ino`, `arm.ino`, `safety.ino` 있어야 함)
   > ⚠️ 파일명이 `main.ino`가 아니라 `robot.ino`인 이유: Arduino는 스케치 폴더명(`robot`)과 진입 파일명이 같아야 컴파일됨
3. **보드 선택**: Tools → Board → OpenRB-150
4. **포트 선택**: Tools → Port → OpenRB-150 잡힌 COM 포트
5. 업로드 (→ 버튼)
6. 라이브러리 오류 시: Library Manager에서 `ArduinoJson`, `Dynamixel2Arduino` 설치

> `robot/test_sequence/`는 카메라·바퀴 없이 그리퍼→팔→컨테이너 동작만 1회 자동 검증하는 독립 테스트 스케치 (같은 방식으로 업로드해서 사용)

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
python vision/src/main.py                    # 기본 실행 (모든 클래스 탐지)
python vision/src/main.py --cls d8            # d8만 픽업
python vision/src/main.py --timer             # 3분 타이머 화면 표시
python vision/src/main.py --cls d12 --align-only       # 정렬 테스트: 회전(좌우)→전후(상하)→2초 직진→grip 후 1회성 종료
python vision/src/main.py --cls d12 --align-fwd-first   # 정렬 테스트: 순서 반대 (전후 먼저→회전 나중)
python vision/src/main.py --cls d12 --no-wheels          # 바퀴 명령 안 보냄 (탐지/그리퍼만 테스트)
python vision/src/main.py --cls d12 --test               # 도달하면 grip 1회 전송 후 즉시 종료
```

> 카메라 화면이 뜨고 탐지 박스가 보이면 정상.  
> 포트/카메라는 전부 **이름 기반 자동 감지**라 USB 꽂는 순서 안 타도 됨 (2026-07-21 확정, 아래 참고).  
> 자동 감지 실패 시에만 `main.py` 상단 `ESP32_PORT`/`OPENRB_PORT`/`CAMERA_INDEX_OBJ`/`CAMERA_INDEX_FLAG` 값 확인.

### STEP 7. 실측값 현황 (2026-07-17 기준)

| 항목 | 파일 | 확정값 | 상태 |
|------|------|--------|------|
| `FINGER_OPEN_RAW` | `robot/gripper.ino` | **2600** (raw, 2026-07-21 2400→2600 확대) | ✅ 완료 (기계적 한계 실물 재확인 권장) |
| `FINGER_CLOSE_RAW` | `robot/gripper.ino` | **1150** (raw) | ✅ 완료 |
| `GRIP_LOAD_THRESHOLD` | `robot/gripper.ino` | 200 (20%) | ✅ 완료 |
| `GRIPPER_SPEED` | `robot/gripper.ino` | 50 (Profile Velocity) | ✅ 완료 |
| `ARM_DOWN_RAW` | `robot/arm.ino` | **1480** (raw) | ✅ 완료 |
| `ARM_UP_RAW` | `robot/arm.ino` | **2850** (raw) | ✅ 완료 |
| `CONT_CLOSED_RAW` | `robot/arm.ino` | **2100** (raw) | ✅ 완료 |
| `CONT_OPEN_RAW` | `robot/arm.ino` | **1000** (raw) | ✅ 완료 |
| `ARM_SPEED` / `CONTAINER_SPEED` | `robot/arm.ino` | 40 / 40 (Profile Velocity) | ✅ 완료 |
| `AREA_GRIP_THRESHOLD` | `vision/src/main.py` | 30000 (2026-07-21 확정, 기존 `AREA_THRESHOLD` 대체) | 🟡 실전 거리에서 재검증 권장 |
| `AREA_SLOW_THRESHOLD` | `vision/src/main.py` | 20000 | ⬜ 실측 필요 |
| `CENTER_MARGIN_PX` / `CENTER_MARGIN_Y_PX` | `vision/src/main.py` | 42px / 35px (2026-07-21 튜닝) | 🟡 실전 거리에서 재검증 권장 |
| `CENTER_OFFSET_X_PX` / `CENTER_OFFSET_Y_PX` | `vision/src/main.py` | 0 / 220 (2026-07-21 튜닝, 양수Y=아래) | 🟡 카메라 재장착 시 재조정 필요 |
| `FINAL_APPROACH_SECS` | `vision/src/main.py` | 1.7초 (정지→직진 시간, 2026-07-21 튜닝) | 🟡 실전 거리에서 재검증 권장 |
| `FORWARD_TRIM` | `vision/src/main.py` | 0.025 (직진 시 우측 쏠림 보정, 2026-07-21 추가) | 🟡 실물 재확인 필요 — 계속 오른쪽으로 쏠리면 기계적(바퀴/모터) 문제일 수 있음 |
| `ENCODER_TICKS_PER_M` | `vision/src/encoder_test.py` | **105.2** | ✅ 완료 |
| `FLAG_AREA_THRESHOLD` | `vision/src/main.py` | 60000 | ⬜ 3m 거리에서 실측 (현재 `GO_TO_STORAGE` 트리거 자체가 비활성 상태) |
| `FLAG_AREA_SLOW_THRESHOLD` | `vision/src/main.py` | 30000 | ⬜ 동일 |
| `STORAGE_BACKUP_SECS` | `vision/src/main.py` | 0.8s | ⬜ 미실측 |
| `CAMERA_INDEX_OBJ` / `CAMERA_INDEX_FLAG` | `vision/src/main.py` | 이름 기반 자동 감지 (2026-07-21) | ✅ 완료 — `arducam`/`nv76`·`cm400` 키워드로 `/sys/class/video4linux` 조회 |

> ⚠️ 팔/컨테이너는 물리 모터 2개가 같은 ID(팔=2, 컨테이너=3)를 공유하는 구조 — 한쪽은 Dynamixel Wizard에서 미리 DRIVE_MODE=Reverse로 구워둠. 코드에서는 절대 DRIVE_MODE를 다시 쓰지 않음 (같은 ID로 두 번 쓰면 둘 다 같은 값이 되어 reverse 구분이 깨짐)

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
      ││ USB (카메라 2대)
  전방 카메라 (Arducam)      — 물체 탐지
  후방 카메라 (NV76-CM400A) — 태극기 탐지 (GO_TO_STORAGE)
  ※ 2026-07-21부터 인덱스 하드코딩 대신 이름 기반 자동 감지
    (/sys/class/video4linux/videoN/name 에서 "arducam"/"nv76"·"cm400" 매칭)
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

# 테스트/디버깅용 플래그 (2026-07-21 추가, 대회 당일엔 안 씀)
python vision/src/main.py --cls d12 --align-only        # 회전(좌우)→전후(상하)→직진→grip, 1회성
python vision/src/main.py --cls d12 --align-fwd-first    # 순서 반대: 전후 먼저→회전 나중, 1회성
python vision/src/main.py --cls d12 --no-wheels           # 바퀴 명령 억제 (탐지/그리퍼만 확인)
python vision/src/main.py --cls d12 --test                # 도달 시 grip 1회 전송 후 즉시 종료

# 데이터 촬영 (record.py)
python vision/src/record.py --cls apple --sec 10          # 10초 녹화 후 자동 프레임 추출
python vision/src/record.py --cls mixed --shutter          # Enter로 한 장씩 촬영 (여러 물체 섞어찍기)
python vision/src/record.py --cls flag --cam 0 --shutter   # 후면(태극기) 카메라로 한 장씩 촬영
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
├── robot/                         # 로봇팀 (OpenRB Arduino — 그리퍼+팔+컨테이너)
│   ├── robot.ino                  # JSON 수신 + 상태 머신 (진입파일, 폴더명과 일치시켜야 컴파일됨)
│   ├── gripper.ino                # ID1 XL430 그리퍼 (랙-피니언)
│   ├── arm.ino                    # ID2 팔 + ID3 컨테이너 (각각 물리모터 2개, 같은 ID 공유)
│   ├── safety.ino                 # Dynamixel overload/hardware error 감시 + 자동 복구
│   └── test_sequence/             # 카메라·바퀴 없이 그리퍼→팔→컨테이너 1회 자동 테스트
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

## 2026-07-20 작업 내역

- **SEARCHING grip 트리거 방식 변경** (`vision/src/main.py`) — 기존 3프레임 연속 confirm 방식(`CONFIRM_FRAMES`)을 제거하고, area+중앙정렬 조건을 최초 충족하면 **정지 → 1초간 정면 직진(`FINAL_APPROACH_SECS`) → grip 전송** 방식으로 변경. 새 상태 변수 `final_approach`/`final_approach_start`/`final_approach_cls` 추가
- **임계값 조정** — `AREA_THRESHOLD`(28000) → `AREA_GRIP_THRESHOLD`(35000)로 이름 변경 + 값 상향. `CENTER_MARGIN_PX`(120→60), `CENTER_MARGIN_Y_PX`(100→50) 절반으로 축소해 중앙정렬 기준을 더 엄격하게 변경
- **TODO 주석 추가** (`vision/src/main.py`, 카메라 초기화부) — "YUY2(비압축) 포맷이 USB 대역폭 초과로 `select()` timeout 유발 — MJPG 전환 예정" 미해결 이슈만 표시, 코드 변경은 아직 없음

> ⚠️ `AREA_GRIP_THRESHOLD`(35000), `FINAL_APPROACH_SECS`(1.0s), `CENTER_MARGIN_PX`/`CENTER_MARGIN_Y_PX`(60/50) 모두 미실측값 — 실물 테스트로 재조정 필요.

---

## 파일별 역할 (vision/)

| 파일 | 역할 |
|------|------|
| `vision/src/main.py` | 메인 실행 파일. 탐지·트래킹·타겟선정·ESP32/OpenRB 전송 전부 담당 |
| `vision/src/calibration.py` | 픽셀 좌표 → 실제 mm 변환 비율 측정. `vision/model/calibration.json` 생성 |
| `vision/src/trt_export.py` | `best.pt` → `best.engine` TensorRT 변환 (Jetson에서만 실행) |
| `vision/src/video_to_frames.py` | 동영상에서 프레임 추출해서 데이터셋 생성 |
| `vision/train/train.ipynb` | Google Colab 학습 노트북 (로컬 디스크 저장 방식, Drive 미의존) |
| `vision/model/best.pt` | 학습된 모델 가중치 (d6/d8/d12/d20, YOLOv8s, mAP50 0.994) |
| `vision/src/capture.py` | 스페이스바로 사진 저장하는 데이터셋 수집 스크립트 |
| `vision/src/record.py` | 클래스별 영상 녹화(`--sec`)+프레임 자동추출, 또는 `--shutter`로 Enter 눌러 한 장씩 촬영. `DATASET/{shape-based,image-based}/{cls}/`에 자동 저장 |

---

## 파일별 역할 (robot/)

| 파일 | 역할 |
|------|------|
| `robot/robot.ino` | Jetson grip/dump 명령 수신 → 상태 머신 실행, Dynamixel 인스턴스 선언 |
| `robot/gripper.ino` | ID1 XL430 그리퍼 제어 (랙-피니언). load confirm+squeeze+hold 방식으로 집기 판정 |
| `robot/arm.ino` | ID2 팔(물리모터 2개, 같은 ID) + ID3 컨테이너(동일 구조) 제어 |
| `robot/safety.ino` | Hardware Error 감시, overload 자동 reboot 복구, fatal fault 분리 |

> 바퀴 제어(ESP32)는 `vision/src/main.py`의 `control_wheels()`가 직접 담당.

### 상태 머신 흐름 (2026-07-20 기준 — 단순화된 상태)

```
[Python main.py]                          [OpenRB robot.ino]

SEARCHING                                 IDLE
  탐지 + 이동 (바퀴 제어)
  ↓ 도달 (bbox 면적 ≥ AREA_GRIP_THRESHOLD(35000) + 중앙정렬)
  정지 → 1초 직진 접근 (FINAL_APPROACH_SECS)
  grip 명령 전송 ──────────────────────▶ GRIPPING
    --test 모드면 여기서 프로그램 종료          그리퍼 닫기 (load confirm+squeeze+hold)
GRIPPING                                    ※ 집기 성공/실패 상관없이 항상 LIFTING 진행
  바퀴 정지, 신호 대기                        (safety.ino 개입 시에만 중단)
  ├─ gripped → SEARCHING (반복)         ◀── {"status":"gripped"}
  ├─ grip_failed → SEARCHING (반복)     ◀── {"status":"grip_failed"} (safety fault 시에만 발생)
  └─ timeout(15s) → SEARCHING (반복)    ◀── {"status":"motor_fault"/"motor_recovered"/"motion_aborted"}

  ※ 수집 개수 카운터 없음 — 무한 SEARCHING↔GRIPPING 반복
```

> ⚠️ `GO_TO_STORAGE`/`DROPPING` 상태 코드는 `main.py`에 남아있지만 트리거(수집 개수 카운터)가 제거되어 **현재 진입 불가** (죽은 코드). 대회에 실제로 쓰려면 "목표 개수 채우면 보관함 이동" 로직을 다시 붙여야 함. 아래 다이어그램은 그 로직이 살아있었을 때 기준으로 참고용으로 남겨둠:

```
GO_TO_STORAGE                             IDLE
  phase 0: 제자리 회전하며 태극기 탐색 (후방 카메라 + flag.pt)
  phase 1: 태극기 보이면 후진하며 접근 → FLAG_AREA_THRESHOLD 도달 시
  dump 명령 전송 ──────────────────────▶ DUMPING (컨테이너 열어 내용물 쏟기)
DROPPING                                 ◀── {"status":"dumped"} → IDLE
  바퀴 정지, dumped 신호 대기
  ├─ dumped → SEARCHING (복귀)
  └─ timeout(60s) → SEARCHING (복귀)
```

### 로봇팀 TODO

| 파일 | 항목 | 내용 |
|------|------|------|
| `main.py` | `AREA_GRIP_THRESHOLD` | 실물 테스트 후 도착 면적 임계값 조정 (현재 35000, 2026-07-20 상향) |
| `main.py` | `FINAL_APPROACH_SECS` | 직진 접근 시간 실측 조정 (현재 1.0s, 2026-07-20 신규) |
| `arm.ino` | `ARM_DOWN_RAW` / `ARM_UP_RAW` | ✅ 완료 (1480 / 2850) |
| `arm.ino` | `CONT_CLOSED_RAW` / `CONT_OPEN_RAW` | ✅ 완료 (2100 / 1000) |
| `main.py` | `FLAG_AREA_THRESHOLD` | 3m 거리에서 태극기 bbox 면적 실측 (현재 `GO_TO_STORAGE` 트리거 자체가 비활성) |
| `main.py` | `STORAGE_BACKUP_SECS` | 집은 자리에서 후진 후 회전 공간 확인 |
| `main.py` | 수집 개수 카운터 / `GO_TO_STORAGE` 트리거 | 2026-07-17 제거됨 — 대회에 쓰려면 재구현 필요 |

### 필요 라이브러리 (Arduino IDE 라이브러리 매니저)

- **ArduinoJson** (Benoit Blanchon) — JSON 파싱 (`robot.ino`)
- **Dynamixel2Arduino** (ROBOTIS) — Dynamixel 제어 (`gripper.ino`, `arm.ino`, `safety.ino`)

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
{"cmd": "grip", "cls": "d8"}  ← 집기 (IDLE→GRIPPING→LIFTING→IDLE, 컨테이너에 투하)
{"cmd": "dump"}                ← 컨테이너 열어 내용물 쏟기 (IDLE→DUMPING→IDLE)
{"cmd": "idle"}                ← 대기
{"cmd": "reset_fault"}         ← safety.ino가 fatal fault로 멈춘 뒤 사람이 확인 후 재개할 때
```

**OpenRB → Jetson 응답:**
```json
{"status": "gripped"}         ← 집기 시도+컨테이너 투하 완료 (성공/실패 무관, Python SEARCHING 복귀)
{"status": "grip_failed"}     ← safety fault로 grip 중단됨 (Python SEARCHING 복귀)
{"status": "dumped"}          ← 컨테이너 비우기 완료 (Python SEARCHING 복귀)
{"status": "motor_fault"}     ← 과열/전압/충격/엔코더 등 자동복구 불가 — reset_fault 필요
{"status": "motor_recovered"} ← overload 자동복구 성공, 현재 동작은 중단하고 IDLE 복귀
{"status": "motion_aborted"}  ← 위 두 경우 외 safety 개입으로 동작 중단
{"status": "fault_reset"}     ← reset_fault 처리 완료
```

> ⚠️ 2026-07-17 변경: 그리퍼 load 임계값 미달(집기 실패)이어도 팔은 항상 올려서 시퀀스를 끝까지 진행함 — `grip_failed`는 이제 safety.ino가 개입한 경우에만 발생.

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

## 2026-07-21 작업 내역

> 실물 로봇으로 정렬→집기 흐름 실제 튜닝한 날. 하드웨어 트러블슈팅(USB 인식 불안정) 시간이 오래 걸림.

### `vision/src/main.py` — SEARCHING 상태머신 재작성

- **grip 트리거를 area+중앙정렬 기반 "정지→직진→grip" 방식으로 변경** — 기존 3프레임 연속 confirm(`CONFIRM_FRAMES`) 로직 제거. `target["area"] >= AREA_GRIP_THRESHOLD`(현재 30000)이면서 화면 중앙의 작은 박스(`CENTER_MARGIN_PX`×`CENTER_MARGIN_Y_PX` = 42×35px, `CENTER_OFFSET_X_PX`/`CENTER_OFFSET_Y_PX` = 0/220 위치) 안에 들어오면 즉시 정지 → `FINAL_APPROACH_SECS`(2.0초) 직진 → grip 전송 → `GRIPPING` 전환
- **`FORWARD_TRIM`(0.025) 추가** — 직진 시 로봇이 오른쪽으로 쏠리는 걸 확인해서, 블라인드 직진 구간(`final_approach`/`align_final_forward`)에 오른쪽 바퀴를 살짝 더 빠르게 주는 보정 추가. 계속 쏠리면 소프트웨어 트림보다 바퀴/모터 자체의 기계적 문제일 가능성 높음
- **`--align-only` 플래그 추가** — 방향 검증 전용 테스트 모드. 1단계: 전진 없이 **제자리 회전만**으로 좌우(cx) 정렬 → 2단계: 회전 없이 **직진/후진만**으로 상하(cy) 정렬 → 정렬 끝나면 1초 대기 후 2초 직진 → grip 전송하고 **프로그램 종료** (1회성, 반복 안 함). 회전 중 상하가 틀어지거나 전후 이동 중 좌우가 틀어지면 해당 단계로 복귀
- **`--align-fwd-first` 플래그 추가** — `--align-only`와 순서만 반대 (전진/후진 먼저 → 회전 나중). 상태 변수(`fb_phase`, `fb_final_forward` 등)를 `--align-only`용과 완전히 분리해서 서로 간섭 없음. 어느 순서가 더 안정적인지 실물로 비교하기 위한 용도
- **`--no-wheels` 플래그 추가** — ESP32가 연결돼 있어도 `control_wheels()`가 아무 명령도 안 보내게 함 (탐지/그리퍼만 테스트하고 싶을 때, 실수로 로봇이 움직이는 것 방지)
- **카메라 포맷 YUY2 → MJPG 전환** — `select() timeout` 경고(USB 대역폭 초과) 완화 목적. 해상도/FPS는 1920x1200@50fps 유지 (1280x720@30fps로 낮춰봤다가 다시 원복 — 팀 판단으로 고해상도 유지 결정)
- **카메라 인덱스도 이름 기반 자동 감지로 변경** — 기존 `_find_port()`(시리얼용)와 동일한 패턴으로 `_find_camera_index()` 추가. `/sys/class/video4linux/videoN/name`을 읽어서 `"arducam"`(물체캠), `"nv76"`/`"cm400"`(태극기캠, 신규 웹캠 NV76-CM400A) 키워드로 매칭 → USB 꽂는 순서 바뀌어도 `/dev/videoN` 번호 안 흔들림. 오늘 하루에도 카메라 인덱스가 몇 번씩 바뀌어서 (물체캠이 태극기캠 인덱스로 잡히는 등) 헤맨 끝에 이 방식으로 정리

### `robot/gripper.ino`

- **`FINGER_OPEN_RAW` 2400 → 2600** — 그리퍼를 더 넓게 벌리도록 확대. 기계적 한계(하드스탑)에 안 걸리는지 실물로 계속 확인 필요

### `vision/src/record.py` — 데이터 수집 도구 개선

- **`--shutter` 모드 추가** — 영상 녹화 대신 Enter 누를 때마다 사진 한 장씩 촬영 (기존 `next_frame_num()` 재사용해서 이어찍기 가능). 예: `python vision/src/record.py --cls flag --cam 0 --shutter`
- **`mixed` 클래스 추가** — 여러 과일을 한 프레임에 섞어 찍을 때 `DATASET/image-based/mixed/`에 저장되도록 `get_save_dir()`에 특수 케이스 추가

### 하드웨어 — 오늘 겪은 문제들과 원인

- **UGV 배터리 전압이 0.58V로 표시된 사건** — 실제로는 배터리 커넥터 접촉불량. 재체결로 해결. (`v` 필드는 ESP32가 `loadVoltage_V * 100`을 정수로 보내는 값이라 `main.py`의 `/100.0` 파싱 자체는 맞았음 — Waveshare `ugv_base_ros` 펌웨어 소스로 확인)
- **ESP32 하트비트 워치독** — UGV 펌웨어가 3초(`HEART_BEAT_DELAY`) 안에 새 속도 명령이 안 오면 자동으로 바퀴를 멈춤. 수동 시리얼 테스트 스크립트로 오래 움직이려면 0.5초 간격으로 계속 재전송해야 함 (한 번만 보내고 `sleep`하면 3초 만에 멈춤)
- **OpenRB USB 인식 불안정** — 업로드 직후 리셋 타이밍에 `by-id` 심볼릭 링크가 아직 안 생겨서 포트 자동감지 실패 → OpenRB 명령이 엉뚱하게 ESP32로 감. 또 한 번은 `lsusb` 자체가 멈추고 OpenRB가 완전히 USB 버스에서 사라지는 증상 발생 — **젯슨 재부팅으로 해결**. USB 케이블 재연결/리셋 버튼 더블클릭만으로는 안 됐음
- **ESP32/OpenRB 포트 번호(`ttyACM0`/`ttyACM1`)가 재연결마다 계속 뒤바뀜** — `main.py`는 이미 `by-id` 이름 기반 자동감지라 문제없지만, 수동 테스트 스크립트에 포트를 하드코딩해서 여러 번 삽질함. 앞으로 수동 스크립트도 `ls -l /dev/serial/by-id/`로 먼저 확인하고 쓸 것
- **시리얼 포트 권한(`Permission denied`)이 재부팅/재연결마다 반복** — `sudo chmod 666 /dev/ttyACMx`을 매번 해야 했음. 영구 해결책: `sudo usermod -aG dialout aiwinners` 실행 후 재로그인하면 이후 chmod 불필요 (오늘 세션에서 제안만 하고 실제 적용 여부는 미확인 — 다음에 확인 필요)
- **새 후면(태극기) 카메라 `NV76-CM400A` 웹캠 추가 연결** — 기존 Arducam(물체캠, USB3, `lsusb -t` 확인 결과 정상적으로 5000M 라인에 물려있음)과 별개로 후면용으로 새로 장착. 오토포커스가 계속 초점을 바꿔서 화면이 흐려지는 문제 있었음 → `v4l2-ctl -d /dev/video0 --set-ctrl=focus_automatic_continuous=0` + `--set-ctrl=focus_absolute=<값>`으로 오토포커스 끄고 고정 초점 설정 가능 (범위 0~1023, 최적값은 촬영 거리 보고 실측 필요 — 아직 미확정)
- **정체불명의 USB 장치 `XIFT NV76-CM400A` (VID:PID `6210:ec03`)** — 알고 보니 이게 그 신규 웹캠이었음. 이름 없는 모델이라 `lsusb`가 이상하게 표시한 것

### `vision/src/main.py` — PATH_NAV/PATH_RETURN 그리드 경로 주행 + 정밀 정렬 승격

- **`--align-only` vs `--align-fwd-first` 비교 결과** — 실물 테스트로 **전진/후진(상하 cy) 먼저 → 회전(좌우 cx) 나중**(`--align-fwd-first` 순서)이 맞는 것으로 확인. 이 순서를 `precise_align`이라는 이름으로 SEARCHING의 실제 grip 로직에 기본 반영함. 두 플래그 자체는 참고용으로 남겨둠 (제거하지 않음)
- **SEARCHING에 정밀 정렬 단계 추가** — `_is_at_target()`을 area 임계 단독 체크로 단순화(중심 정렬은 더 이상 여기서 안 봄) → area 도달 시 `precise_align=True` 진입 → 전진/후진으로 cy 정렬 → 회전으로 cx 정렬 → 완료되면 기존 `final_approach`(직진 접근+grip) 단계로 이어짐
- **`PATH_NAV`/`PATH_RETURN` 상태 신규 추가** (`--path-test` 플래그) — 가로 7×세로 6칸(50cm 간격) 그리드 기반 고정 경로 주행: 시작 1m 직진 → 우회전 → 직진(벽까지) → 좌회전 → 직진(지점까지) → 좌회전 → 직진(벽까지) → 좌회전 → 완료 시 `GO_TO_STORAGE`로 자동 전환
  - 직진1/3(벽 탐색 구간) 중에만 목표 물건 감지 시 `SEARCHING`으로 잠깐 빠져서 집고, 집는 동안 이동/회전한 시간을 누적해뒀다가 grip 후 `PATH_RETURN` 상태에서 그 시간만큼 반대로 후진+반대 회전해서 원래 있던 자리로 복귀한 뒤 경로 재개
  - **회전/직진 시간 실측 반영**: `PATH_SECS_PER_METER = 4.0`(1m당 4초), `PATH_TURN_90_SECS = 1.5`(제자리 90도 회전 1.5초) — 나머지 구간 시간은 이 값들로 계산됨
  - 벽 감지(`is_near_wall()`)는 아직 미구현(항상 False) — YOLO 대신 **바닥-벽 경계선의 화면상 높이로 거리 추정하는 방식**을 검토 중 (다음 세션에서 이어서 논의)
- **`select_target()` 우선순위 변경** — area 최대인 것 우선, area가 비슷한(`AREA_SIMILAR_TOLERANCE`=3000 이내) 후보가 여럿이면 화면 오른쪽(cx 큰 것) 우선 선택

> ⚠️ **다음 세션에서 확인할 것**: `FORWARD_TRIM` 값이 충분한지(계속 오른쪽으로 쏠리면 하드웨어 점검), `dialout` 그룹 등록 여부, 그리퍼 2600 raw가 기계적 한계 안 걸리는지, 새 웹캠 `focus_absolute` 최적값, `PATH_NAV` 좌/우회전 부호(`PATH_LEFT_L/R`, `PATH_RIGHT_L/R`)가 실제 방향과 맞는지, `PATH_TO_POINT_SECS`/`PATH_START_FORWARD_SECS` 실측 재검증, 벽 감지 방식(바닥-벽 경계선 높이 기반) 구현 여부.

---

## 2026-07-17 작업 내역

- **`robot/main.ino` → `robot.ino` 이름 변경** — Arduino 툴체인은 스케치 폴더명과 진입 파일명이 같아야 컴파일됨 (`robot/` 폴더인데 `main.ino`라 그동안 실제로 빌드 불가능한 상태였음). 이제 `arduino-cli compile robot/`로 정상 컴파일·업로드 확인 완료
- **`safety.ino` 추가** — Dynamixel Hardware Error Status 감시, overload 발생 시 자동 reboot 복구. 과열/전압/엔코더/전기충격은 fatal fault로 분리해 사람이 `{"cmd":"reset_fault"}` 보내야 복귀하도록 함. `{"status":"motor_fault"/"motor_recovered"/"motion_aborted"}` 신규 응답 추가
- **그리퍼 로직 정교화** (`gripper.ino`) — load 감지 시 즉시 멈추지 않고 confirm count(3프레임 연속) + 살짝 더 조이기(squeeze) + 그 자리 hold 방식으로 변경
- **집기 실패해도 팔은 항상 올리도록 변경** — 기존엔 load 임계값 미달 시 팔을 안 올리고 바로 `grip_failed` 리턴했는데, 테스트 편의를 위해 성공/실패 상관없이 항상 LIFTING(팔 올림→투하→팔 내림)까지 진행하도록 변경. `safety.ino`가 개입하는 심각한 하드웨어 오류일 때만 중단됨
- **모터 위치/속도값 재실측 반영** — 그리퍼(OPEN=2400/CLOSE=1150), 팔(DOWN=1480/UP=2850), 컨테이너(CLOSED=2100/OPEN=1000), 전부 raw(0~4095) 단위로 통일 (degree 변환 없이 직접 사용). `PROFILE_VELOCITY` 속도값도 추가 (그리퍼50/팔·컨테이너40)
- **집기 후 팔 올리기 전 0.5초, 그리퍼 연 뒤 팔 내리기 전 0.8초 딜레이 추가** — 물체가 실제로 안정적으로 잡히고/떨어질 시간 확보
- **테스트 스케치 정리** — `test_step`/`test_sequence`/`test_container`를 각자 독립 폴더로 분리 (한 폴더에 있으면 Arduino가 전부 하나로 묶어 컴파일해서 `setup()`/`loop()` 중복 정의 충돌났음). 이후 `test_step`/`test_container`/진단용 `test_diag`는 정리 완료해서 제거, `test_sequence`만 유지
- **`vision/src/main.py` — 수집 개수 카운터 제거** — `pickup_counts`, `max_count()`, 클래스별 목표 개수 체크, 화면 스코어 표시, 전체 수집 완료 시 `GO_TO_STORAGE` 자동 전환 로직을 전부 제거. 지금은 물체 하나 감지→접근→grip→다시 탐색을 무한 반복하는 단순한 루프. `GO_TO_STORAGE`/`DROPPING` 상태 코드 자체는 남아있지만 트리거가 없어 현재는 도달 불가 (죽은 코드, 나중에 다시 연결 필요)
- **`--test` 1회성 테스트 모드 추가** (`main.py`) — 타겟 도달 확인되면 grip 명령만 보내고 바로 프로그램 종료 (반복 없이 단발 테스트용)
- **MJPEG 스트림 서버 버그 수정** — `HTTPServer` → `ThreadingHTTPServer`로 교체. 기존엔 동시 접속 1개만 처리 가능해서 브라우저 새로고침/재접속 시 스트림이 멈춘 것처럼 보이는 문제가 있었음
- **재학습 완료 및 `best.pt` 교체** — Roboflow(`merong-gurme` 프로젝트) 데이터셋으로 YOLOv8s 재학습, **mAP50 0.994** (d6/d8/d12/d20 전체 클래스). `train.ipynb`도 Google Drive FUSE 마운트가 반복적으로 불안정(`FileExistsError`/`NotADirectoryError`)해서 **Drive 의존 제거하고 Colab 로컬 디스크(`/content/runs_local`)에 저장하는 방식으로 변경**
- **`vision/src/capture.py` 추가** — 데이터셋 수집용 스페이스바 캡처 스크립트
- **원격 접속 환경 정리**
  - Jetson SSH 키 등록 완료 (비밀번호 없이 접속 가능) — `~/.ssh/authorized_keys`에 공개키 추가. Windows OpenSSH는 개인키 파일 권한이 본인 계정 외에도 열려있으면 조용히 키를 무시하고 비번으로 넘어가니, `icacls`로 권한 제한 필요했음 (VS Code Remote-SSH에서 비번 계속 뜨던 원인)
  - VS Code Remote-SSH 접속 방법 정리 (`aiwinners@<jetson IP>`)
  - 핫스팟 두 개(iPhone/Android) Jetson에 동시 등록 가능 확인 (`nmcli device wifi connect`) — NetworkManager가 저장된 프로필 중 주변에 있는 걸 자동으로 잡음
  - SSH/VS Code는 **같은 네트워크 안에서만** 동작 (172.20.10.x 등은 사설 IP라 인터넷 건너 원격 접속 불가) — 물리적으로 다른 곳에서 접속하려면 Tailscale 같은 VPN 필요(미설정)

> ⚠️ **주의**: 위 변경 중 `main.py`의 카운터 제거 + `GO_TO_STORAGE` 비활성화는 "일단 grip 단독 동작 테스트"를 위한 임시 단순화임. 대회에 쓰려면 목표 개수 채운 뒤 보관함으로 이동하는 로직을 다시 붙여야 함.

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

### 전원 구성 확정 (2026-07-17 기준)
- 젯슨: **보조배터리 → USB-C PD (15V, 5.5×2.1mm 확인 필요)**  
- XL430 그리퍼+팔+컨테이너 전부: **OpenRB 전용 별도 배터리** (폴리트로닉스 PT-B2200N-SP35, 11.1V 3S LiPo, 2200mAh, XT60) → OpenRB — UGV 내장 배터리 아님  
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
| ✅ 완료 | `FINGER_OPEN_RAW` / `FINGER_CLOSE_RAW` | **2400 / 1150** (raw) | 실측 완료 | `gripper.ino` |
| ✅ 완료 | `GRIP_LOAD_THRESHOLD` | **200** (Load 20%) | 실측 완료 | `gripper.ino` |
| ✅ 완료 | `ENCODER_TICKS_PER_M` | **105.2** | 실측 완료 | `main.py` |
| ✅ 완료 | `ARM_DOWN_RAW` / `ARM_UP_RAW` | **1480 / 2850** (raw) | 실측 완료 | `arm.ino` |
| ✅ 완료 | `CONT_CLOSED_RAW` / `CONT_OPEN_RAW` | **2100 / 1000** (raw) | 실측 완료 | `arm.ino` |
| ✅ 완료 | 모터 속도(Profile Velocity) | 그리퍼50 / 팔40 / 컨테이너40 | 실측+테스트로 조정 완료 | `gripper.ino`, `arm.ino` |
| ✅ 완료 | `AREA_GRIP_THRESHOLD` | 30000 (2026-07-21, 기존 `AREA_THRESHOLD` 대체) | 실전 거리에서 재검증 권장 | `main.py` |
| ✅ 완료 | 카메라 포맷 YUY2→MJPG 전환 | 완료 (2026-07-21) | USB 대역폭 초과 `select()` timeout 문제 해결 | `main.py` |
| 🔴 높음 | 수집 개수 카운터 / `GO_TO_STORAGE` 트리거 | 제거됨 | 대회에 쓰려면 재구현 필요 (2026-07-17 참고) | `main.py` |
| 🔴 높음 | `FLAG_AREA_THRESHOLD` | 60000 | 보관함 3m 거리에서 태극기 bbox 면적 측정 | `main.py` |
| 🔴 높음 | `FLAG_AREA_SLOW_THRESHOLD` | 30000 | 동일 | `main.py` |
| ✅ 완료 | `CAMERA_INDEX_OBJ` / `CAMERA_INDEX_FLAG` | 이름 기반 자동 감지 (2026-07-21) | `/dev/video*` 번호 신경 안 써도 됨 | `main.py` |
| 🟡 중간 | 새 웹캠(NV76-CM400A) `focus_absolute` | 미확정 | `v4l2-ctl -d /dev/video0 --set-ctrl=focus_absolute=<값>`으로 촬영거리 맞춰 실측 | v4l2-ctl (하드웨어 설정, 코드 아님) |
| 🟡 중간 | `STORAGE_BACKUP_SECS` | 0.8s | 집은 자리에서 후진 후 공간 확인 | `main.py` |

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

1. `vision/model/best.pt` GitHub에 포함 ✅ (d6/d8/d12/d20 전체 학습 완료, mAP50 0.994)
2. `vision/model/flag.pt` — **미학습** ⬜ (태극기 20~30장 촬영 후 학습 필요)
3. Roboflow 프로젝트 접근 권한 확인 (workspace: `s-workspace-qdwfc`, project: `merong-gurme`)
4. Colab 노트북 실행 전 Roboflow API 키 입력 — Google Drive는 FUSE 불안정 이슈로 더 이상 안 씀, 결과는 로컬(`/content/runs_local`)에 저장됨
5. Jetson 연결 포트/카메라 확인 (2026-07-21부터 시리얼·카메라 전부 **이름 기반 자동 감지**라 보통 신경 안 써도 됨):
   - 시리얼: `ls -l /dev/serial/by-id/`로 ESP32(`1a86`/`ch343`)·OpenRB(`openrb`/`robotis`) 확인 가능. 자동 감지 실패 시에만 `main.py`의 `ESP32_PORT`/`OPENRB_PORT` fallback 값 확인
   - 카메라: `v4l2-ctl --list-devices`로 Arducam(물체캠)·NV76-CM400A(태극기캠) 확인 가능. 자동 감지 실패 시에만 `CAMERA_INDEX_OBJ`/`CAMERA_INDEX_FLAG` fallback 값 확인
   - USB 인식이 아예 안 될 때(`lsusb`에 장치 자체가 안 보임/응답 없음): 케이블 재연결이나 리셋 버튼으로 안 풀리면 **젯슨 재부팅**이 제일 빠른 해결책이었음 (2026-07-21 경험)
6. OpenRB Arduino 업로드 시 보드: **OpenRB-150** 선택 (`robot.ino` + `gripper.ino` + `arm.ino` + `safety.ino` 같은 폴더, 파일명이 `main.ino`가 아니라 `robot.ino`인 것에 주의 — 폴더명과 일치해야 컴파일됨)
7. Dynamixel Wizard로 서보 ID 및 Baudrate 사전 설정:
   - ID1 (그리퍼), ID2 (팔, 물리모터 2개 공유), ID3 (컨테이너, 물리모터 2개 공유), 모두 **Baudrate=1000000**
   - ID2/ID3: 한쪽 모터 Drive Mode → Reverse Wizard에서 미리 설정 (코드에서는 절대 재설정 안 함 — 같은 ID 공유라 둘 다 같은 값으로 덮어써짐)
8. XL430 전원: OpenRB 초록 단자에 12V 배터리 연결 (두꺼운 전선 필수)
9. 위치/속도값 전부 실측 완료 (그리퍼 2400/1150, 팔 1480/2850, 컨테이너 2100/1000, 속도 50/40/40) — `gripper.ino`/`arm.ino` 참고
10. `AREA_THRESHOLD`, `FLAG_AREA_THRESHOLD` 실물 테스트로 측정 후 `main.py` 수정
11. **수집 개수 카운터 / `GO_TO_STORAGE` 자동 전환 로직이 제거된 상태** ⬜ — 지금 `main.py`는 물체 하나 감지→grip→반복만 하는 무한루프. 대회에 쓰려면 "목표 개수 채운 뒤 보관함 이동" 로직을 다시 붙여야 함 (2026-07-17 작업 내역 참고)
