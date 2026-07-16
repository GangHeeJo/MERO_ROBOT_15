#!/usr/bin/env python3
"""
capture.py — 실시간 프리뷰 + 스페이스바로 사진 저장

사용:
  python vision/src/capture.py --cls apple
  python vision/src/capture.py --cls d8 --cam 2
  python vision/src/capture.py --cls flag --width 1920 --height 1200

조작:
  SPACE  : 사진 저장
  Q / ESC: 종료

저장 위치:
  vision/DATASET/shape-based/{cls}/   -> d6, d8, d12, d20
  vision/DATASET/image-based/{cls}/   -> apple, banana, orange, pineapple
  vision/DATASET/{cls}/               -> 기타
"""

import cv2
import os
import re
import argparse

BASE_DIR    = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATASET_DIR = os.path.join(BASE_DIR, "DATASET")

SHAPE_CLASSES = {'d6', 'd8', 'd12', 'd20'}
FRUIT_CLASSES = {'apple', 'banana', 'orange', 'pineapple'}


def get_save_dir(cls):
    if cls in SHAPE_CLASSES:
        return os.path.join(DATASET_DIR, "shape-based", cls)
    elif cls in FRUIT_CLASSES:
        return os.path.join(DATASET_DIR, "image-based", cls)
    else:
        return os.path.join(DATASET_DIR, cls)


def next_frame_num(save_dir, cls):
    pattern = re.compile(rf"^{re.escape(cls)}_(\d+)\.jpg$")
    nums = [int(m.group(1)) for f in os.listdir(save_dir) if (m := pattern.match(f))]
    return max(nums) + 1 if nums else 0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--cls',    required=True)
    parser.add_argument('--cam',    type=int, default=0)
    parser.add_argument('--width',  type=int, default=1920)
    parser.add_argument('--height', type=int, default=1200)
    args = parser.parse_args()

    save_dir = get_save_dir(args.cls)
    os.makedirs(save_dir, exist_ok=True)

    cap = cv2.VideoCapture(args.cam)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"카메라 {args.cam}  해상도 {w}x{h}")
    print(f"저장 경로: {save_dir}")
    print("SPACE=저장  Q/ESC=종료")

    for _ in range(5):
        cap.read()

    count = next_frame_num(save_dir, args.cls)
    cv2.namedWindow("Capture", cv2.WINDOW_NORMAL)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        disp = frame.copy()
        cv2.putText(disp, f"cls:{args.cls}  saved:{count}  SPACE=capture",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow("Capture", disp)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == 27:
            break
        elif key == ord(' '):
            path = os.path.join(save_dir, f"{args.cls}_{count:05d}.jpg")
            cv2.imwrite(path, frame)
            print(f"저장: {path}")
            count += 1

    cap.release()
    cv2.destroyAllWindows()
    print(f"종료 — 총 {count}장")


if __name__ == '__main__':
    main()
