#!/usr/bin/env python3
"""
record.py — 클래스별 영상 녹화 + 프레임 자동 추출

사용:
  python vision/src/record.py --cls apple             # 30초 녹화, 10프레임당 1장
  python vision/src/record.py --cls flag  --sec 20    # 20초
  python vision/src/record.py --cls d8   --interval 5 # 5프레임당 1장 (더 많은 이미지)
  python vision/src/record.py --cls apple --cam 2     # 카메라 인덱스 지정

저장 위치:
  vision/DATASET/shape-based/{cls}/   → d6, d8, d12, d20
  vision/DATASET/image-based/{cls}/   → apple, banana, orange, pineapple
  vision/DATASET/{cls}/               → flag, 기타
"""

import cv2
import os
import re
import time
import argparse
from datetime import datetime

BASE_DIR    = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATASET_DIR = os.path.join(BASE_DIR, "DATASET")

SHAPE_CLASSES = {'d6', 'd8', 'd12', 'd20'}
FRUIT_CLASSES = {'apple', 'banana', 'orange', 'pineapple'}


def get_save_dir(cls):
    if cls in SHAPE_CLASSES:
        return os.path.join(DATASET_DIR, "shape-based", cls)
    elif cls in FRUIT_CLASSES or cls == "mixed":
        return os.path.join(DATASET_DIR, "image-based", cls)
    else:
        return os.path.join(DATASET_DIR, cls)


def next_frame_num(save_dir, cls):
    """기존 jpg 파일 중 최대 번호 + 1 반환 (겹침 방지)"""
    pattern = re.compile(rf"^{re.escape(cls)}_(\d+)\.jpg$")
    nums = [int(m.group(1)) for f in os.listdir(save_dir) if (m := pattern.match(f))]
    return max(nums) + 1 if nums else 0


def extract_frames(video_path, save_dir, cls, interval):
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"[extract] 영상 열기 실패: {video_path}")
        return

    start_num = next_frame_num(save_dir, cls)
    frame_idx = 0
    saved = 0

    print(f"[extract] {interval}프레임당 1장 추출 중...")

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        if frame_idx % interval == 0:
            save_path = os.path.join(save_dir, f"{cls}_{start_num + saved:05d}.jpg")
            cv2.imwrite(save_path, frame)
            saved += 1
        frame_idx += 1

    cap.release()
    print(f"[extract] {saved}장 추출 완료 → {save_dir}")


def record(cls, cam_idx, seconds, interval, args_w=None, args_h=None):
    save_dir = get_save_dir(cls)
    os.makedirs(save_dir, exist_ok=True)

    cap = cv2.VideoCapture(cam_idx)
    if not cap.isOpened():
        print(f"카메라 {cam_idx} 열기 실패")
        return

    if args_w and args_h:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  args_w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args_h)

    w    = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h    = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps  = cap.get(cv2.CAP_PROP_FPS) or 30.0
    print(f"         해상도: {w}x{h}")

    # 카메라 워밍업
    for _ in range(5):
        cap.read()

    timestamp  = datetime.now().strftime("%Y%m%d_%H%M%S")
    video_path = os.path.join(save_dir, f"{cls}_{timestamp}.mp4")

    writer = cv2.VideoWriter(video_path, cv2.VideoWriter_fourcc(*'mp4v'), fps, (w, h))

    has_display = bool(os.environ.get("DISPLAY", ""))
    if has_display:
        cv2.namedWindow("Recording  (q=중단)", cv2.WINDOW_NORMAL)

    print(f"\n[record] 클래스:{cls}  카메라:{cam_idx}  {seconds}초 녹화 시작")
    print(f"         {video_path}")
    if not has_display:
        print("         헤드리스 모드 — Ctrl+C로 중단")

    start       = time.time()
    frame_count = 0
    tick        = max(1, int(fps))

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("프레임 읽기 실패")
                break

            elapsed   = time.time() - start
            remaining = seconds - elapsed
            if remaining <= 0:
                break

            writer.write(frame)
            frame_count += 1

            if frame_count % tick == 0:
                filled = int(20 * elapsed / seconds)
                bar    = '█' * filled + '░' * (20 - filled)
                print(f"\r  [{bar}] {elapsed:.0f}s/{seconds}s  {frame_count}f", end='', flush=True)

            if has_display:
                disp = frame.copy()
                cv2.putText(disp, f"REC {elapsed:.1f}s / {seconds}s",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                cv2.imshow("Recording  (q=중단)", disp)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print("\n중단됨")
                    break

    except KeyboardInterrupt:
        print("\n중단됨")

    print(f"\n[record] 완료: {frame_count}프레임 저장")
    cap.release()
    writer.release()
    if has_display:
        cv2.destroyAllWindows()

    extract_frames(video_path, save_dir, cls, interval)


def shutter(cls, cam_idx, args_w=None, args_h=None):
    """Enter 누를 때마다 사진 한 장씩 촬영 (Ctrl+C로 종료)."""
    save_dir = get_save_dir(cls)
    os.makedirs(save_dir, exist_ok=True)

    cap = cv2.VideoCapture(cam_idx)
    if not cap.isOpened():
        print(f"카메라 {cam_idx} 열기 실패")
        return

    if args_w and args_h:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  args_w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args_h)

    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"         해상도: {w}x{h}")

    # 카메라 워밍업
    for _ in range(5):
        cap.read()

    start_num = next_frame_num(save_dir, cls)
    frame_num = start_num
    print(f"\n[shutter] 클래스:{cls}  카메라:{cam_idx}  저장위치:{save_dir}")
    print("[shutter] Enter를 누르면 한 장씩 촬영, Ctrl+C로 종료")

    try:
        while True:
            input(f"  [{frame_num}번째] Enter로 촬영...")
            ret, frame = cap.read()
            if not ret:
                print("프레임 읽기 실패")
                continue
            save_path = os.path.join(save_dir, f"{cls}_{frame_num:05d}.jpg")
            cv2.imwrite(save_path, frame)
            print(f"  ✅ 저장: {save_path}")
            frame_num += 1
    except KeyboardInterrupt:
        print(f"\n[shutter] 종료 — 총 {frame_num - start_num}장 촬영")

    cap.release()


def main():
    parser = argparse.ArgumentParser(description="클래스별 영상 녹화 + 프레임 추출")
    parser.add_argument('--cls',      required=True, help="클래스명 (d6/d8/d12/d20/apple/banana/orange/pineapple/flag)")
    parser.add_argument('--sec',      type=int, default=30, help="녹화 시간 초 (기본 30)")
    parser.add_argument('--interval', type=int, default=10, help="프레임 추출 간격 (기본 10, 낮을수록 이미지 많음)")
    parser.add_argument('--cam',      type=int, default=0,    help="카메라 인덱스 (기본 0)")
    parser.add_argument('--width',    type=int, default=1920, help="녹화 해상도 너비 (기본 1920)")
    parser.add_argument('--height',   type=int, default=1200, help="녹화 해상도 높이 (기본 1200)")
    parser.add_argument('--shutter',  action='store_true', help="Enter 누를 때마다 사진 한 장씩 촬영 (영상 녹화 대신)")
    args = parser.parse_args()

    if args.shutter:
        shutter(args.cls, args.cam, args.width, args.height)
    else:
        record(args.cls, args.cam, args.sec, args.interval, args.width, args.height)


if __name__ == '__main__':
    main()
