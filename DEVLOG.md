# MERO AI ROBOT — 개발 로그

---

## 2026-06-20

### 프로젝트 구조 파악
- 대회 목적 확인: 실시간 비전으로 물체 분류 → 집게로 집어서 목표 지점으로 이동 (pick-and-place)
- 역할 분담: 본인 = 비전 담당
- 기술 스택 확정
  - 보드: Jetson Orin Nano
  - 로봇: Waveshare UGV02
  - 카메라: Arducam (USB 방식 확인)
  - 모델: YOLOv8

### 대회 태스크 2가지 확인
- **shape-based**: d6, d8, d12, d20 (다면체 주사위 분류)
- **image-based**: apple, banana, orange, pineapple (과일 분류)

### 데이터셋 현황 분석

| 클래스 | 장수 | 영상 수 | 촬영 날짜 |
|--------|------|---------|-----------|
| d6     | 85장 | 5개     | 06-18 하루만 |
| d8     | 150장| 8개     | 06-04 + 06-18 |
| d12    | 136장| 4개     | 06-18 하루만 |
| d20    | 185장| 5개     | 06-18 하루만 |
| 과일 4종 | 0장 | -      | 미수집 |

- 해상도 전체 1920x1200 통일
- **문제**: 전부 집에서 촬영 → 대회 환경(바닥, 조명)과 다름 → 재촬영 필요
- d6가 85장으로 가장 적어 클래스 불균형 있음

### 코드 개선 (`arducam_test.py`)
- 하드코딩된 절대 경로 → `os.path.abspath(__file__)` 기준 상대 경로로 변경
- 카메라 fallback(1번 → 0번) 후 체크 없던 것 → 0번도 실패 시 즉시 종료 추가
- `while` 루프 `try/finally`로 감싸기 → Ctrl+C 포함 모든 종료에서 `cap.release()` 보장
- 잘못된 "Stream 모드" 주석 제거
- `model()` → `model.track(persist=True)` 로 변경 → 트래킹 적용
  - 여러 물체에 고유 ID 부여, 프레임 간 ID 유지
  - 바운딩박스 중심좌표 `(cx, cy)` + 클래스명 + 신뢰도 + track_id 콘솔 출력 추가 (로봇 제어 연동용)

### 코드 개선 (`export_image_from_video.py`)
- 폴더 모드에서 `*.mp4`만 검색하던 버그 수정 → `*.mp4`, `*.avi`, `*.mov` 전체 검색

### 학습 파이프라인 구성
- `MERO_train.ipynb` 생성 (Google Colab용)
  - Roboflow API로 데이터 직접 다운로드
  - YOLOv8s 학습 (100 epoch, patience=20)
  - 증강: 밝기/색상/회전(±45°)/크기/모자이크 — 집 환경과 대회 환경 차이 보완 목적
  - 결과 Google Drive 자동 저장
  - mAP 출력 + best.pt 다운로드

---

## 2026-06-20 (2차)

### 데이터셋 정리
- d6 폴더 이미지 파일명 일괄 변경: `WIN_20260618_...` → `d6_1.jpg` ~ `d6_85.jpg` (85개)

### 로봇팀 통신 방식 결정
- UGV02 내장 컨트롤러 = **ESP32 슬레이브 컴퓨터** 확인 (Waveshare 공식 스펙)
- 통신 방식: **JSON over Serial (USB, 115200 baud)**
  - Jetson USB → ESP32 직결, 추가 하드웨어 불필요
  - ESP32가 JSON 커맨드 수신을 공식 지원하는 구조

### 신규 파일 추가

#### `src/arducam_test.py` 대폭 업그레이드
- **ESP32 시리얼 통신** 추가 (pyserial)
  - ESP32 미연결 시 자동으로 카메라 단독 모드 전환 (에러 없음)
  - 매 프레임 탐지 결과를 JSON으로 전송
- **타겟 선택 로직** 추가 (`select_target()`)
  - 여러 물체 중 신뢰도 가장 높은 것 1개를 `TARGET`으로 선정
  - 화면에 노란 박스 + `TARGET` 텍스트로 강조 표시
- **카메라 캘리브레이션 연동** 추가
  - `calibration.json` 있으면 픽셀 좌표를 mm 좌표로 자동 변환
  - 없으면 픽셀 좌표만 사용 (기능은 정상 동작)
- ESP32 전송 JSON 포맷:
  ```json
  {
    "objects": [{"id":1, "cls":"d8", "cx":342.5, "cy":218.3, "mx":12.3, "my":-5.1, "conf":0.91}],
    "target":  {"id":1, "cls":"d8", "cx":342.5, "cy":218.3, "mx":12.3, "my":-5.1, "conf":0.91}
  }
  ```

#### `src/calibration.py` 신규 생성
- 마우스 2번 클릭 + 실제 거리(mm) 입력 → mm/pixel 비율 계산
- `model/calibration.json` 저장 → arducam_test.py가 자동 로드

#### `src/export_engine.py` 신규 생성
- `best.pt` → `best.engine` TensorRT 변환 래퍼 스크립트
- Jetson에서 1회 실행, 변환 후 arducam_test.py 경로만 변경하면 됨

### 앞으로 할 일

- [ ] 과일 데이터 수집 (apple, banana, orange, pineapple)
- [ ] 대회 바닥 환경에서 shape-based 재촬영 (d6 우선)
- [ ] Roboflow 라벨링 (shape-based 556장)
- [ ] Colab에서 YOLOv8 학습 → best.pt 갱신
- [ ] 모델 성능 확인 (mAP, confusion matrix)
- [x] 카메라 캘리브레이션 코드 작성 (`src/calibration.py`)
- [x] 로봇팀 통신 인터페이스 코드 작성 (JSON over Serial)
- [ ] Jetson 도착 후: TensorRT 변환 + 포트 확인 + 실물 테스트
- [ ] ESP32 수신 코드 로봇팀과 연동 테스트
- [ ] 멀티스레딩 (비전 / 제어 분리)
