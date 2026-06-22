# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 프로젝트

pick-and-place 로봇 대회. 카메라로 물체 분류·트래킹 → 집게로 집어서 목표 지점 이동.
비전 담당 파트만 이 리포에 있고 로봇 제어 코드는 없음.

**하드웨어**: Jetson Orin Nano / Waveshare UGV02 / Arducam USB

**대회 태스크**
- shape-based: d6, d8, d12, d20
- image-based: apple, banana, orange, pineapple

## 폴더 구조

```
MERO_AI_ROBOT/
├── src/
│   ├── arducam_test.py            # 실시간 트래킹 메인 (ESP32 통신 포함)
│   ├── calibration.py             # 카메라 캘리브레이션 (픽셀→mm, 1회 실행)
│   ├── export_engine.py           # TensorRT 변환 스크립트 (Jetson 전용)
│   └── export_image_from_video.py # 동영상 → 프레임 추출
├── train/
│   └── MERO_train.ipynb           # Colab 학습 노트북
├── model/
│   ├── best.pt                    # 학습된 가중치
│   ├── best.engine                # TensorRT 변환 파일 (Jetson 변환 후 생성)
│   └── calibration.json           # 캘리브레이션 결과 (calibration.py 실행 후 생성)
├── DATASET/
│   ├── shape-based/               # d6·d8·d12·d20
│   └── image-based/               # 과일 (미수집)
├── CLAUDE.md
└── DEVLOG.md
```

## 실행

```bash
# 메인 실행 (트래킹 + ESP32 통신)
python src/arducam_test.py

# Jetson 셋업 시 순서
python src/calibration.py    # 1. 캘리브레이션 (1회)
python src/export_engine.py  # 2. TensorRT 변환 (1회, Jetson에서만)
python src/arducam_test.py   # 3. 메인 실행
```

## 학습 파이프라인

```
Roboflow 라벨링 → MERO_train.ipynb(Colab) → best.pt → export_engine.py → best.engine
```

## 로봇팀 연동 (ESP32 통신)

Jetson → ESP32: **USB Serial, 115200 baud, JSON per line**

```json
{
  "objects": [{"id":1, "cls":"d8", "cx":342.5, "cy":218.3, "mx":12.3, "my":-5.1, "conf":0.91}],
  "target":  {"id":1, "cls":"d8", "cx":342.5, "cy":218.3, "mx":12.3, "my":-5.1, "conf":0.91}
}
```

- `objects`: 탐지된 전체 물체 목록
- `target`: 집게가 집을 대상 1개 (신뢰도 최고)
- `mx`, `my`: calibration.json 있을 때만 포함 (mm, 이미지 중심 기준)
- 탐지 없으면: `{"objects": [], "target": null}`
