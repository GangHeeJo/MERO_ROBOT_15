import cv2
import os
import glob

print("="*60)
print("🎥 동영상 프레임 자동 추출기 (누적 저장 방식) 🎥")
print("="*60)

# ──────────────────────────────────────────────
# 1. 사용자 입력
# ──────────────────────────────────────────────
# strip('"\' ')  : 파일 탐색기에서 경로를 복사하면 양쪽에 따옴표가 붙는 경우가 있어 자동 제거
video_input    = input("🎬 1. 동영상 파일(.mp4/.avi/.mov) 경로 또는 폴더 경로를 입력하세요:\n👉 ").strip('"\' ')
save_dir       = input("\n💾 2. 쪼개진 이미지를 저장할 폴더 경로를 입력하세요:\n👉 ").strip('"\' ')
interval_input = input("\n⏱️ 3. 몇 프레임당 1장을 추출할까요? (숫자만 입력, 예: 10):\n👉 ").strip()

# 프레임 간격 유효성 검사 (양의 정수여야 함)
try:
    FRAME_INTERVAL = int(interval_input)
    if FRAME_INTERVAL <= 0:
        raise ValueError
except ValueError:
    print("\n⚠️ 프레임 간격은 양의 정수로 입력해야 합니다. 기본값인 10으로 설정합니다.")
    FRAME_INTERVAL = 10

# ──────────────────────────────────────────────
# 2. 저장 폴더 생성
# ──────────────────────────────────────────────
# exist_ok=True : 이미 폴더가 있어도 에러 없이 그대로 사용
os.makedirs(save_dir, exist_ok=True)

# ──────────────────────────────────────────────
# 3. 입력 경로가 단일 파일인지 폴더인지 판별
# ──────────────────────────────────────────────
SUPPORTED_EXTS = ('.mp4', '.avi', '.mov')  # 지원하는 동영상 확장자 목록

video_files = []
if os.path.isfile(video_input):
    # 단일 파일로 지정한 경우 — 지원 확장자인지 확인 후 추가
    if video_input.lower().endswith(SUPPORTED_EXTS):
        video_files.append(video_input)
    else:
        print(f"\n❌ 에러: 지원하지 않는 파일 형식입니다. ({', '.join(SUPPORTED_EXTS)} 만 가능)")
        exit()

elif os.path.isdir(video_input):
    # 폴더로 지정한 경우 — 지원하는 모든 확장자의 파일을 검색
    # 수정 전: *.mp4만 검색해서 .avi/.mov 파일이 누락됐었음
    for ext in ("*.mp4", "*.avi", "*.mov"):
        video_files.extend(glob.glob(os.path.join(video_input, ext)))

else:
    print("\n❌ 에러: 유효하지 않은 경로입니다. 파일이나 폴더가 실제로 존재하는지 확인해주세요.")
    exit()

if not video_files:
    print("\n⚠️ 처리할 동영상 파일을 찾지 못했습니다. 프로그램을 종료합니다.")
    exit()

print(f"\n📡 총 {len(video_files)}개의 동영상 파일을 찾았습니다. 추출을 시작합니다...\n")

total_saved_images = 0

# ──────────────────────────────────────────────
# 4. 프레임 추출 및 누적 저장
# ──────────────────────────────────────────────
for video_path in video_files:
    # 파일명에서 확장자 제거 (예: d6.mp4 → d6, WIN_xxx.avi → WIN_xxx)
    video_name = os.path.splitext(os.path.basename(video_path))[0]

    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"❌ 동영상을 열 수 없습니다: {video_path}")
        continue

    frame_count = 0  # cap이 실제로 읽은 프레임 번호와 항상 동기화되는 카운터
    saved_count = 0  # 이번 동영상에서 새로 저장한 이미지 수

    while True:
        ret, frame = cap.read()
        if not ret:
            # 동영상 끝 또는 읽기 실패 → 다음 파일로
            break

        # FRAME_INTERVAL 간격의 프레임만 추출 (예: 10이면 0, 10, 20, 30... 번째 프레임)
        if frame_count % FRAME_INTERVAL == 0:
            # 저장 파일명 형식: {동영상이름}_f{프레임번호5자리}.jpg
            # 예) d6_f00120.jpg
            save_name = f"{video_name}_f{frame_count:05d}.jpg"
            save_path = os.path.join(save_dir, save_name)

            if os.path.exists(save_path):
                # 이미 추출된 프레임이면 덮어쓰지 않고 건너뜀 (재실행해도 안전)
                frame_count += 1
                continue

            # 이미지 저장 (JPEG 형식)
            cv2.imwrite(save_path, frame)
            saved_count += 1
            total_saved_images += 1

        frame_count += 1  # 매 프레임마다 카운터 증가 (cap.read()와 1:1 동기화)

    cap.release()

    if saved_count > 0:
        print(f"✅ [{video_name}] {saved_count}장의 이미지를 새로 추출했습니다.")
    else:
        print(f"ℹ️ [{video_name}] 이미 모두 처리되었거나 새로 추가할 프레임이 없습니다.")

print(f"\n🎉 모든 작업이 끝났습니다! 새로 추가된 {total_saved_images}장의 이미지가 '{save_dir}' 폴더에 저장되었습니다.")
