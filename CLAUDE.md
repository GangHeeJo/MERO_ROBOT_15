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
  - ID 4: 카메라 회전 서보 — 추가 검토 중, `robot/test_camera_servo/`에서 단독 테스트만 된 상태, `robot.ino` 본체엔 아직 미통합
- 카메라: ArduCAM 2.3MP AR0234 글로벌 셔터 USB 3.0 — 전면 1대만 사용 (2026-07-22부터 통합 모델로 단일 카메라 구조로 리팩토링, 후면 태극기 전용 카메라는 폐기)

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
│   │   ├── flag_test.py           # flag 탐지 품질 확인용 — 실행 후 브라우저 :8081로 스트림 확인
│   │   ├── encoder_test.py        # 바퀴 엔코더 ENCODER_TICKS_PER_M 실측용
│   │   ├── imu_test.py            # IMU 값 확인용
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
├── robot/                         # 로봇팀 (OpenRB Arduino — 그리퍼+팔+컨테이너)
│   ├── robot.ino                  # JSON 수신 + 상태 머신 (진입파일 — 폴더명 robot과 일치해야 컴파일됨, main.ino 아님 주의)
│   ├── gripper.ino                # ID1 XL430 그리퍼 (랙-피니언)
│   ├── arm.ino                    # ID2 팔 + ID3 컨테이너 (각각 물리모터 2개, 같은 ID 공유)
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
                                           "gripper_open"/"gripper_close"/"reset_fault"/"arm_up", ...}
  └─← /dev/ttyACM1 ← OpenRB-150          {"status":"gripped"/"grip_failed"/"dumped"/
                                           "gripper_opened"/"gripper_closed"/"started"/"arm_up_done"/
                                           "motor_fault"/"motor_recovered"/"motion_aborted"/"fault_reset"}
```

`gripper_open`/`gripper_close`는 안전정책이 아니라 집기 메커니즘 자체에 필요 — IDLE 기본값이 "닫힘"이라 미리 열어두지 않으면 집을 공간이 없음. 타겟 발견(정밀 정렬 진입) 시 `gripper_open`, 타겟 놓치면 `gripper_close` 전송. `start`는 경기 시작 시 규정 크기용으로 올려둔 팔을 내리는 명령(전원 켜지면 팔은 기본적으로 올림 상태로 대기). `arm_up`은 디버깅용 — 팔을 수동으로 시작 위치(올림)로 복귀.

## 상태 머신

> ⚠️ 2026-07-17 기준: `main.py`에서 수집 개수 카운터와 `GO_TO_STORAGE` 자동 전환 트리거를 제거해서, 지금은 SEARCHING↔GRIPPING만 무한 반복하는 단순한 루프임. `GO_TO_STORAGE`/`DROPPING`은 Python(`main.py`) 쪽에만 남아있는 상태 이름이고 진입할 방법이 없음 (죽은 코드, robot.ino에는 애초에 대응 상태 없음). 대회에 쓰려면 재구현 필요 — 자세한 내용은 `progress.md`의 2026-07-17 작업 내역 참고.

**Python (main.py) — 현재 실제 동작 (2026-07-21 재작성):**
```
SEARCHING → 타겟 발견 시 gripper_open 전송, 정밀 정렬(회전→전후) 진행
         → area(bbox 면적) ≥ AREA_GRIP_THRESHOLD 이고 화면 중앙 작은 박스 안에 들어오면
           정지 → FINAL_APPROACH_SECS(현재 1.7초, `main.py` 상수 확인) 직진 → grip 전송 → GRIPPING
         → 정렬 중 타겟 놓치면 gripper_close 전송 후 재탐색
GRIPPING → (gripped/grip_failed/timeout 무엇이든) → POST_GRIP_SCAN
POST_GRIP_SCAN → 이동 없이 제자리 회전(POST_GRIP_SCAN_SECS=4초, 실측 필요)하며 주변 재탐색
              → 타겟 발견하거나 시간 다 차면 → SEARCHING (이후 정밀 정렬/탐색은 기존 로직 그대로)
```
타겟 없을 때 탐색 이동은 2026-07-22부터 "가장 먼 물체 1개" 대신 **밀집도 가중 중심**(주변 물체가 몰려있는 방향)으로 조향 (`main.py` `select_target()`과 동일한 클러스터 반경 재사용). 3프레임 연속 confirm 방식은 폐기됨. (`--test` 플래그: grip 전송 후 결과 기다리지 않고 바로 프로그램 종료)

**OpenRB (robot.ino) — 상태: IDLE / GRIPPING / LIFTING / DUMPING**
```
IDLE(그리퍼 닫힘) → (grip 수신) → GRIPPING → (집기 시도, 성공/실패 무관 항상 진행, 2초 대기) → LIFTING
    (팔 올림 → 그리퍼 열어 투하 → 2초 대기 → 그리퍼 다시 닫기(대기상태) → 팔 내림) → IDLE (gripped 전송)
IDLE → (dump 수신) → DUMPING → (컨테이너 열고 500ms 후 닫기) → IDLE (dumped 전송)
IDLE → (gripper_open/gripper_close/start/arm_up 수신) → 해당 동작만 수행, 상태 변화 없음

safety.ino가 overload/hardware error를 감지하면 어느 상태에서든 즉시 IDLE로 복귀
(fatal fault면 사람이 reset_fault 보낼 때까지 명령 거부)
```

## 알려진 이슈 (2026-07-22 세션 중단 시점 기준)

> ⚠️ **팔(ID2) 모터가 명령에 응답만 하고 실제로 안 움직이는 문제 — 미해결**
> `{"cmd":"arm_up"}`/`{"cmd":"start"}` 모두 정상 응답(`arm_up_done`/`started`)이 오는데 팔이 물리적으로 안 움직임. 그리퍼(ID1) fault는 `reset_fault`로 리셋했지만 팔은 계속 무반응. 배선 문제는 아닌 것으로 확인됨 — safety.ino가 조용히 개입 중이거나 팔이 별도 fault 상태일 가능성. 다음 확인 순서: `ls /dev/ttyACM*`로 포트 확인 → `robot/test_arm_updown/`을 `arduino-cli monitor`로 모니터링해서 실제 위치 로그 확인. 상세: `progress.md`의 2026-07-22 작업 내역 최하단 참고.

## 전원 구성

- 젯슨: 보조배터리 USB-C PD → 배럴잭 (내경 실측 필요)
- XL430 그리퍼+팔+컨테이너 전부: OpenRB 전용 별도 배터리(11.1V 3S LiPo, XT60) → OpenRB — UGV 내장 배터리 아님
