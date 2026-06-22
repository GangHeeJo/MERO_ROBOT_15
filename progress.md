# MERO AI ROBOT — Progress

> 최종 업데이트: 2026-06-21  
> 비전 담당: 조강희

---

## 프로젝트 개요

카메라로 물체를 실시간 탐지·분류·트래킹 후 집게로 집어서 목표 지점으로 이동하는 pick-and-place 대회.  
이 리포는 **비전 파트**만 포함. 로봇 제어 코드는 로봇팀 별도 관리.

**하드웨어**
- 보드: NVIDIA Jetson Orin Nano
- 로봇: Waveshare UGV02 (내장 컨트롤러: ESP32)
- 카메라: Arducam USB

**대회 태스크**
- shape-based: d6, d8, d12, d20 (다면체 주사위)
- image-based: apple, banana, orange, pineapple

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
├── vision/       # 비전팀
├── robot/        # 로봇팀 ESP32 코드
└── progress.md
```

## 파일별 역할 (vision/)

| 파일 | 역할 |
|------|------|
| `vision/src/arducam_test.py` | 메인 실행 파일. 탐지·트래킹·타겟선정·ESP32 전송 전부 담당 |
| `vision/src/calibration.py` | 픽셀 좌표 → 실제 mm 변환 비율 측정. `vision/model/calibration.json` 생성 |
| `vision/src/export_engine.py` | `best.pt` → `best.engine` TensorRT 변환 (Jetson에서만 실행) |
| `vision/src/export_image_from_video.py` | 동영상에서 프레임 추출해서 데이터셋 생성 |
| `vision/train/MERO_train.ipynb` | Google Colab 학습 노트북 |
| `vision/model/best.pt` | 학습된 모델 가중치 (현재 d8만 학습된 임시본) |

---

## 로봇팀 연동 (ESP32 통신)

**연결**: Jetson USB → ESP32, 115200 baud, JSON per line

매 프레임 아래 형식으로 전송:

```json
{
  "objects": [
    {"id": 1, "cls": "d8", "cx": 342.5, "cy": 218.3, "mx": 12.3, "my": -5.1, "conf": 0.91}
  ],
  "target": {"id": 1, "cls": "d8", "cx": 342.5, "cy": 218.3, "mx": 12.3, "my": -5.1, "conf": 0.91}
}
```

| 필드 | 설명 |
|------|------|
| `objects` | 이번 프레임 탐지된 전체 물체 목록 |
| `target` | 집게가 집을 대상 1개 (신뢰도 최고). 없으면 null |
| `id` | 트래킹 ID (프레임 간 유지) |
| `cls` | 물체 종류 (d6 / d8 / d12 / d20 / apple 등) |
| `cx`, `cy` | 픽셀 좌표 |
| `mx`, `my` | 카메라 중심 기준 실제 거리 mm (캘리브레이션 후 포함) |

탐지 없으면: `{"objects": [], "target": null}`

**ESP32 포트 확인 (Jetson 터미널):**
```bash
ls /dev/tty*   # ttyUSB0 또는 ttyACM0 확인
```
확인 후 `arducam_test.py` 상단 `SERIAL_PORT` 값 수정.

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
2. train/MERO_train.ipynb (Google Colab)
      ↓
3. model/best.pt 저장
      ↓
4. python src/export_engine.py (Jetson)
      ↓
5. model/best.engine → arducam_test.py에서 사용
```

Colab 노트북 실행 전 필요한 것:
- Roboflow API 키
- Google Drive 마운트

---

## 완료된 작업 ✅

- [x] YOLOv8 실시간 트래킹 구현 (`model.track(persist=True)`)
- [x] 타겟 선택 로직 (신뢰도 최고 물체 1개 자동 선정)
- [x] 화면 시각화 (바운딩박스 + TARGET 노란 강조)
- [x] ESP32 시리얼 통신 코드 (JSON over USB Serial)
- [x] 카메라 캘리브레이션 코드 (`src/calibration.py`)
- [x] TensorRT 변환 스크립트 (`src/export_engine.py`)
- [x] Colab 학습 노트북 (`train/MERO_train.ipynb`)
- [x] d6 이미지 파일명 정리 (d6_1 ~ d6_85)
- [x] pyserial 설치 및 통신 테스트

---

## 남은 작업 ⬜

| 우선순위 | 작업 | 비고 |
|----------|------|------|
| 🔴 높음 | 과일 데이터 촬영 (4종) | 현재 0장 |
| 🔴 높음 | 대회 환경에서 재촬영 | d6 우선 |
| 🔴 높음 | Roboflow 라벨링 (556장) | |
| 🟡 중간 | Colab 전체 클래스 학습 | 라벨링 완료 후 |
| 🟡 중간 | 모델 성능 확인 (mAP, confusion matrix) | |
| 🟢 낮음 | Jetson 도착 후 TensorRT 변환 | |
| 🟢 낮음 | 캘리브레이션 실행 (1회) | 코드 완료 |
| 🟢 낮음 | ESP32 실물 연동 테스트 | |
| 🟢 낮음 | 멀티스레딩 (비전 / 제어 분리) | |

---

## 인수인계 시 체크리스트

이어서 작업하는 사람이 확인할 것:

1. `model/best.pt` 파일 별도 수령 (git에 미포함)
2. Roboflow 프로젝트 접근 권한 확인
3. Colab 노트북 실행 전 Roboflow API 키 입력
4. Jetson 도착 시 `ls /dev/tty*` 로 ESP32 포트 확인 후 `SERIAL_PORT` 수정
5. 캘리브레이션은 카메라 설치 높이 확정 후 1회 실행
