"""
Jetson TensorRT 변환 스크립트
─────────────────────────────
실행: python src/trt_export.py  ← 반드시 Jetson에서 실행

목적:
  학습된 best.pt (PyTorch 가중치)를 Jetson 전용 추론 엔진인
  TensorRT 포맷(best.engine)으로 변환.
  변환 후 추론 속도가 3~5배 빨라짐.

변환 후 main.py 에서 모델 경로만 바꾸면 됨:
  MODEL_PATH = os.path.join(BASE_DIR, "model", "best.engine")

주의:
  - Windows/Mac 에서는 실행 불가 (TensorRT는 NVIDIA GPU 전용)
  - 변환에 수 분 소요됨
  - best.pt 가 model/ 폴더에 있어야 함
"""

import os
from ultralytics import YOLO

BASE_DIR    = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PT_PATH     = os.path.join(BASE_DIR, "model", "best.pt")
ENGINE_PATH = os.path.join(BASE_DIR, "model", "best.engine")

# best.pt 존재 여부 확인
if not os.path.exists(PT_PATH):
    print(f"❌ 모델 파일 없음: {PT_PATH}")
    print("   Colab 학습 후 best.pt 를 model/ 폴더에 넣어주세요.")
    exit()

# 이미 변환된 파일이 있으면 덮어쓸지 확인
if os.path.exists(ENGINE_PATH):
    ans = input(f"⚠️ {ENGINE_PATH} 이미 존재합니다. 덮어쓰겠습니까? (y/n): ")
    if ans.lower() != 'y':
        print("취소됨.")
        exit()

print(f"변환 시작: {PT_PATH}")
print("시간이 수 분 걸릴 수 있습니다... (Jetson 성능에 따라 다름)")

model = YOLO(PT_PATH)

# half=True : FP16 반정밀도 사용 → 속도 향상, 정밀도 손실 미미
model.export(format="engine", half=True)

print(f"\n✅ 변환 완료: {ENGINE_PATH}")
print("\n다음 단계:")
print("  main.py 상단에서 아래 줄의 주석을 해제하세요:")
print("  # MODEL_PATH = os.path.join(BASE_DIR, 'model', 'best.engine')")
