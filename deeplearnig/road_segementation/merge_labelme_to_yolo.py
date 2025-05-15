import os
import json
import random
import shutil
from pathlib import Path
import cv2

# 경로 설정
input_dir = Path("/home/vit/dev_ws/project/driving_frame_1200_1569")  # JSON + JPG가 있는 폴더
output_img_dir = Path("/home/vit/dev_ws/project/road_dataset_yolo/images")
output_lbl_dir = Path("/home/vit/dev_ws/project/road_dataset_yolo/labels")

# train/val 폴더 생성
for subset in ["train", "val"]:
    (output_img_dir / subset).mkdir(parents=True, exist_ok=True)
    (output_lbl_dir / subset).mkdir(parents=True, exist_ok=True)

# 모든 JSON 파일 기준으로 섞고 나누기
json_files = sorted(input_dir.glob("*.json"))
random.seed(42)
random.shuffle(json_files)

train_ratio = 0.8
split_idx = int(len(json_files) * train_ratio)
train_files = json_files[:split_idx]
val_files = json_files[split_idx:]

def convert_and_copy(json_list, subset):
    for json_file in json_list:
        # .png 또는 .jpg 이미지 파일 탐색
        img_file = next(
            (input_dir / f"{json_file.stem}{ext}" for ext in [".png", ".jpg"] if (input_dir / f"{json_file.stem}{ext}").exists()),
            None
        )
        if img_file is None:
            print(f"[경고] 이미지 누락: {json_file.stem}.jpg 또는 .png")
            continue

        # 이미지 크기 읽기
        img = cv2.imread(str(img_file))
        if img is None:
            print(f"[오류] 이미지 로드 실패: {img_file}")
            continue
        h, w = img.shape[:2]

        # JSON 로드 및 YOLO 형식 변환
        with open(json_file, "r") as f:
            data = json.load(f)

        yolo_lines = []
        for shape in data["shapes"]:
            try:
                class_id = int(shape["label"])  # 정수 라벨만 처리
            except ValueError:
                print(f"[무시됨] '{shape['label']}' → 정수 변환 실패")
                continue

            points = shape["points"]
            normalized_points = []
            for x, y in points:
                normalized_points.extend([x / w, y / h])
            line = f"{class_id} " + " ".join(f"{p:.6f}" for p in normalized_points)
            yolo_lines.append(line)

        # 저장 경로
        dst_img_path = output_img_dir / subset / img_file.name
        dst_lbl_path = output_lbl_dir / subset / f"{json_file.stem}.txt"

        shutil.copy(img_file, dst_img_path)
        with open(dst_lbl_path, "w") as f:
            f.write("\n".join(yolo_lines))

# 변환 실행
convert_and_copy(train_files, "train")
convert_and_copy(val_files, "val")

print("✅ YOLO 형식으로 변환 완료 (PNG 또는 JPG 포함)")
