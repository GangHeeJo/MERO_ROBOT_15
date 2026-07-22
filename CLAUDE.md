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
  - ID 4: 카메라 회전 서보 — `robot/camera.ino`로 `robot.ino` 본체에 통합 완료. 전원 켤 때 카메라가 보던 위치를 "정면"으로 저장해두고, `cam_backward`(180도 회전)/`cam_forward`(정면 복귀) 명령으로 제어
- 카메라: ArduCAM 2.3MP AR0234 글로벌 셔터 USB 3.0 — 전면 1대만 사용 (2026-07-22부터 통합 모델로 단일 카메라 구조로 리팩토링, 후면 태극기 전용 카메라는 폐기). `GO_TO_STORAGE` 진입 시 ID4 서보로 카메라 자체를 180도 돌려서 같은 카메라로 후방(보관함 방향)을 봄 — 카메라 회전축이 지면과 수평(팬 회전)이라 영상 상하 반전은 없음. USB 대역폭 타이밍 노이즈로 `select() timeout` 경고가 간헐적으로 뜨는데(1920×1200@50fps 고해상도 유지 중), `FRAME_FAIL_LIMIT`(`main.py`, 연속 100회)까지는 자동 복구 시도 — 그 이상 멈추면 `cap.read()` 자체가 블로킹된 것으로 보고 USB 재연결 필요

**대회 태스크**
- shape-based: d6, d8, d12, d20
- image-based: apple, banana, orange, pineapple

## 폴더 구조

```
MERO_AI_ROBOT/
├── vision/                        # 비전팀 (Jetson Python)
│   ├── src/
│   │   ├── main.py                # 메인 실행 (탐지·트래킹·타겟선정 + ESP32/OpenRB 통신, 카메라 1대 + 통합모델 구조)
│   │   ├── calibration.py         # 카메라 캘리브레이션 (픽셀→mm, 헤드리스 --capture/--calc 지원, 1회 실행)
│   │   ├── capture.py             # 스페이스바로 사진 저장하는 데이터셋 수집 스크립트
│   │   ├── record.py              # 클래스별 영상 녹화(--sec) 또는 --shutter(Enter로 한 장씩) 데이터 수집
│   │   ├── flag_test.py           # best.pt로 flag 탐지 품질만 필터링해 확인 — 실행 후 브라우저 :8081로 스트림 확인
│   │   ├── encoder_test.py        # 바퀴 엔코더 ENCODER_TICKS_PER_M 실측용
│   │   ├── imu_test.py            # IMU 값 확인용
│   │   ├── wheels_test.py         # ESP32 바퀴 단독 구동 테스트 (w/a/s/d, f <초> 직진, r <초> 회전, L R 속도 직접)
│   │   ├── camera_test.py         # 카메라만 단독 가동 — 연결/해상도/FPS 확인, 브라우저 :8082 스트림
│   │   ├── cam_servo_test.py      # OpenRB 카메라 서보(ID4)만 단독 테스트 — b(후방)/f(정면)/t(왕복), 재업로드 불필요
│   │   ├── basket_test.py         # OpenRB 바스켓(ID3)만 단독 테스트 — o(열기, 유지)/c(닫기)
│   │   ├── launcher.py            # 물리 버튼 3개 + OLED로 카메라·젯슨 없이 클래스 선택/실행하는 독립 런처
│   │   ├── trt_export.py          # TensorRT 변환 스크립트 (Jetson 전용)
│   │   └── video_to_frames.py
│   ├── train/
│   │   └── train.ipynb            # Colab 학습 노트북
│   ├── model/
│   │   ├── best.pt                # 학습된 가중치 — 도형(d6/d8/d12/d20)+과일(apple/banana/orange/pineapple)+flag 통합 9클래스
│   │   ├── flag.pt                # flag 단독 모델 (flag_test.py 전용, best.pt 통합 이후 사용 여부 재확인 필요)
│   │   ├── best.engine            # TensorRT 변환 파일 (Jetson 변환 후 생성)
│   │   └── calibration.json       # 캘리브레이션 결과 (calibration.py 실행 후 생성)
│   └── DATASET/
│       ├── shape-based/           # d6·d8·d12·d20
│       └── image-based/           # apple·banana(실사진, 대회용 큐브로 교체 필요)·orange·pineapple(미수집)
├── robot/                         # 로봇팀 (OpenRB Arduino — 그리퍼+팔+컨테이너+카메라서보)
│   ├── robot.ino                  # JSON 수신 + 상태 머신 (진입파일 — 폴더명 robot과 일치해야 컴파일됨, main.ino 아님 주의)
│   ├── gripper.ino                # ID1 XL430 그리퍼 (랙-피니언)
│   ├── arm.ino                    # ID2 팔 + ID3 컨테이너 (각각 물리모터 2개, 같은 ID 공유)
│   ├── camera.ino                 # ID4 카메라 회전 서보 — cam_backward/cam_forward
│   ├── safety.ino                 # Dynamixel overload/hardware error 감시 + 자동 복구
│   ├── test_sequence/             # 카메라·바퀴 없이 그리퍼→팔→컨테이너 1회 자동 테스트 (독립 스케치)
│   ├── test_arm_updown/           # 팔(ID2) 단독 상하 이동 테스트 — 고정 delay 대신 실제 도달 위치 폴링
│   ├── test_container/            # 컨테이너(ID3) 단독 개폐 테스트
│   └── test_camera_servo/         # 카메라 회전 서보(ID4) 단독 180도 왕복 테스트
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

# 메인 실행 (캘리브레이션 없어도 동작, --cls 없이 실행하면 로드 후 콘솔에서 클래스 입력 대기)
python vision/src/main.py

# 경기 당일 — 타겟 클래스 지정 (복수 지정 가능)
python vision/src/main.py --cls d8
python vision/src/main.py --cls d8 apple

# 디버깅/테스트 플래그 (대회 당일엔 안 씀)
python vision/src/main.py --cls d12 --timer        # 화면에 3분 카운트다운 표시
python vision/src/main.py --cls d12 --test          # 도달 시 grip 1회 전송 후 즉시 종료
python vision/src/main.py --cls d12 --align-only    # 정렬 테스트: 회전(좌우)→전후(상하)→2초 직진→grip, 1회성
python vision/src/main.py --cls d12 --no-wheels     # 바퀴 명령 억제 (탐지/그리퍼만 확인)

# 카메라 연결 확인 / flag 탐지 품질 확인 — 실행 후 브라우저에서 http://<젯슨IP>:8081 접속
python vision/src/flag_test.py

# 데이터 수집
python vision/src/capture.py --cls apple            # 스페이스바로 사진 저장
python vision/src/record.py --cls apple --sec 10    # 10초 녹화 후 자동 프레임 추출
python vision/src/record.py --cls flag --shutter    # Enter로 한 장씩 촬영

# 선택: 캘리브레이션 (1회, 카메라 높이 확정 후, 헤드리스 SSH 지원)
python vision/src/calibration.py --capture                       # 사진만 저장, PC에서 좌표 확인
python vision/src/calibration.py --calc x1 y1 x2 y2 실제거리mm   # 좌표로 비율 계산

# 선택: TensorRT 변환 (1회, Jetson에서만)
python vision/src/trt_export.py
```

## 학습 파이프라인

```
Roboflow 라벨링 → train.ipynb(Colab) → best.pt → trt_export.py → best.engine
```

`best.pt`는 도형 4종 + 과일 4종 + flag를 합친 9클래스 통합 모델(2026-07-22 병합). flag 단독 데이터셋은 반례 부족으로 오탐이 심해서 통합 모델로 재학습함. `select_target()`이 flag 클래스는 grip 후보에서 항상 제외.

## 통신 구조

```
Jetson main.py
  ├─→ /dev/ttyACM0 → ESP32 (UGV 바퀴)   {"T":1, "L":speed, "R":speed}
  └─→ /dev/ttyACM1 → OpenRB-150          {"cmd":"grip"/"dump"/"idle"/"start"/
                                           "gripper_open"/"gripper_close"/"reset_fault"/"arm_up"/
                                           "cam_backward"/"cam_forward"/
                                           "basket_open"/"basket_close", ...}
  └─← /dev/ttyACM1 ← OpenRB-150          {"status":"gripped"/"grip_failed"/"dumped"/
                                           "gripper_opened"/"gripper_closed"/"started"/"arm_up_done"/
                                           "cam_backward_done"/"cam_forward_done"/
                                           "basket_opened"/"basket_closed"/
                                           "motor_fault"/"motor_recovered"/"motion_aborted"/"fault_reset"}
```

`gripper_open`/`gripper_close`는 안전정책이 아니라 집기 메커니즘 자체에 필요 — IDLE 기본값이 "닫힘"이라 미리 열어두지 않으면 집을 공간이 없음. 타겟 발견(정밀 정렬 진입) 시 `gripper_open`, 타겟 놓치면 `gripper_close` 전송. `start`는 경기 시작 시 규정 크기용으로 올려둔 팔을 내리는 명령(전원 켜지면 팔은 기본적으로 올림 상태로 대기). `arm_up`은 디버깅용 — 팔을 수동으로 시작 위치(올림)로 복귀. `cam_backward`/`cam_forward`는 ID4 카메라 서보 제어 — `GO_TO_STORAGE` 진입 시(경기당 1회) `cam_backward`를 보내 카메라가 후방(보관함 방향)을 보게 함. `basket_open`/`basket_close`는 임시 디버깅용 — `dump`와 달리 자동으로 안 닫히고 열린 채로 유지되어 바스켓 안을 직접 확인하거나 수동으로 비울 때 사용 (`vision/src/basket_test.py`로 단독 테스트 가능).

## 상태 머신

> 2026-07-22: `GO_TO_STORAGE`/`DROPPING`이 시간 기반 트리거(`PICK_PHASE_SECS`)로 재구현되어 더 이상 죽은 코드가 아님. 경기 시작(Enter) 시점에 `match_start_time`을 기록하고, 매 프레임 상태머신 분기 진입 전에 `SEARCHING`/`GRIPPING`/`POST_GRIP_SCAN` 셋 중 어느 상태에 있든 `PICK_PHASE_SECS`(150초=2분30초) 지나면 즉시 중단하고 `GO_TO_STORAGE`로 강제 전환됨 — SEARCHING에서만 체크하면 grip 타임아웃(15초)+스캔(4초)으로 남은 30초를 거의 다 까먹을 수 있어서 세 상태 모두 체크.

**Python (main.py) — 현재 실제 동작:**
```
(SEARCHING/GRIPPING/POST_GRIP_SCAN 공통) PICK_PHASE_SECS(150초) 경과 시 즉시 중단
    → gripper_close(열려있었다면) → cam_backward 전송 → GO_TO_STORAGE

SEARCHING → 타겟 발견 시 gripper_open 전송, 정밀 정렬(전후→회전) 진행
         → area(bbox 면적) ≥ AREA_GRIP_THRESHOLD 이고 화면 중앙 작은 박스 안에 들어오면
           정지 → FINAL_APPROACH_SECS(1.7초) 직진 → grip 전송 → GRIPPING
         → 정렬 중 타겟 놓치면 gripper_close 전송 후 재탐색
GRIPPING → (gripped/grip_failed/timeout 무엇이든) → POST_GRIP_SCAN
POST_GRIP_SCAN → 이동 없이 제자리 회전(POST_GRIP_SCAN_SECS=4초)하며 주변 재탐색
              → 타겟 발견하거나 시간 다 차면 → SEARCHING (이후 정밀 정렬/탐색은 기존 로직 그대로)
GO_TO_STORAGE → phase 0: 제자리 회전하며 flag 탐색 (같은 카메라/프레임에서 cls=='flag'만 필터, 별도 추론 없음)
              → phase 1: flag 보이면 후진 접근(카메라가 후방을 보는 상태라 "뒤가 앞"처럼 취급, 조향 부호 반전)
                → FLAG_AREA_THRESHOLD 도달 시 정지 → dump 전송 → DROPPING
                → flag 놓치면 phase 0으로 복귀
              → STORAGE_TIMEOUT_SECS(60초) 넘으면 SEARCHING으로 강제 복귀
DROPPING → dumped 수신 또는 DROP_TIMEOUT_SECS(15초) 타임아웃 → SEARCHING 복귀
```
타겟 없을 때 탐색 이동은 "가장 먼 물체 1개" 대신 **밀집도 가중 중심**(주변 물체가 몰려있는 방향)으로 조향 (`CLUSTER_RADIUS_PX` 재사용). `MAX_ROTATE_SECS`(1.0초) 넘게 회전해도 물체가 2개 안 모이면 `SEARCH_FORWARD_BURST_SECS`(2.0초) 직진 후 회전 재개. (`--test` 플래그: grip 전송 후 결과 기다리지 않고 바로 프로그램 종료)

`select_target()`은 밀집도가 아니라 **area(화면 중앙에 가까운 정도) 단독 기준**으로 후보 하나를 고른다 — 원래 밀집도(cluster_score)를 1순위로 썼었는데, 그 값이 다른 물체 검출 여부에 따라 프레임마다 흔들리기 쉬워서 타겟 후보가 여러 개일 때 정밀 정렬 도중 다른 물체로 선택이 튀는 문제가 있어 단순화함(2026-07-22). 정밀 정렬(`precise_align`) 진입 시 그 물체의 track id를 `last_target_id`에 락 걸고, 이후로는 `select_target()`을 다시 안 부르고 그 id만 `detected`에서 찾아 추적 — 놓치면 `TARGET_MISS_GRACE_FRAMES`(3프레임)까지는 정지 대기 후 재등장 기다리고, 그래도 안 잡히면 재탐색으로 복귀.

**OpenRB (robot.ino) — 상태: IDLE / GRIPPING / LIFTING / DUMPING**
```
IDLE(그리퍼 닫힘) → (grip 수신) → GRIPPING → (집기 시도, 성공/실패 무관 항상 진행, 2초 대기) → LIFTING
    (팔 올림 → 그리퍼 열어 투하 → 2초 대기 → 그리퍼 다시 닫기(대기상태) → 팔 내림) → IDLE (gripped 전송)
IDLE → (dump 수신) → DUMPING → (컨테이너 열고 500ms 후 닫기) → IDLE (dumped 전송)
IDLE → (gripper_open/gripper_close/start/arm_up/cam_backward/cam_forward/basket_open/basket_close 수신) → 해당 동작만 수행, 상태 변화 없음

safety.ino가 overload/hardware error를 감지하면 어느 상태에서든 즉시 IDLE로 복귀
(fatal fault면 사람이 reset_fault 보낼 때까지 명령 거부)
```

## 전원 구성

- 젯슨: 보조배터리 USB-C PD → 배럴잭 (내경 실측 필요)
- XL430 그리퍼+팔+컨테이너 전부: OpenRB 전용 별도 배터리(11.1V 3S LiPo, XT60) → OpenRB — UGV 내장 배터리 아님
