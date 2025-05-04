import os
import json
import glob
from sklearn.model_selection import train_test_split
import shutil

# 경로 설정
input_dir = '/home/vit/dev_ws/project/road_dataset'
output_dir = '/home/vit/dev_ws/project/road_dataset_yolo'
images_output = os.path.join(output_dir, 'images')
labels_output = os.path.join(output_dir, 'labels')
classes = ['white_solid', 'yellow_center', 'white_dashed']  # 0, 1, 2

# 디렉토리 생성
for split in ['train', 'val']:
    os.makedirs(os.path.join(images_output, split), exist_ok=True)
    os.makedirs(os.path.join(labels_output, split), exist_ok=True)

# 모든 이미지와 json 파일 정리
all_jsons = sorted(glob.glob(os.path.join(input_dir, '*.json')))
all_images = [j.replace('.json', '.jpg') for j in all_jsons]

# train/val 분할
train_jsons, val_jsons = train_test_split(all_jsons, test_size=0.2, random_state=42)

def convert(json_path, split):
    with open(json_path, 'r') as f:
        data = json.load(f)

    img_name = os.path.basename(json_path).replace('.json', '.jpg')
    txt_name = img_name.replace('.jpg', '.txt')

    img_output_path = os.path.join(images_output, split, img_name)
    txt_output_path = os.path.join(labels_output, split, txt_name)

    # 이미지 복사
    original_img_path = os.path.join(input_dir, img_name)
    shutil.copy2(original_img_path, img_output_path)

    with open(txt_output_path, 'w') as out:
        for shape in data['shapes']:
            label = shape['label']
            class_id = int(label)
            points = shape['points']

            # 바운딩 박스 계산 (YOLO는 center_x, center_y, width, height / 정규화)
            x_coords = [p[0] for p in points]
            y_coords = [p[1] for p in points]
            x_min, x_max = min(x_coords), max(x_coords)
            y_min, y_max = min(y_coords), max(y_coords)
            x_center = (x_min + x_max) / 2.0 / data['imageWidth']
            y_center = (y_min + y_max) / 2.0 / data['imageHeight']
            width = (x_max - x_min) / data['imageWidth']
            height = (y_max - y_min) / data['imageHeight']

            out.write(f"{class_id} {x_center:.6f} {y_center:.6f} {width:.6f} {height:.6f}\n")

# 실행
for json_file in train_jsons:
    convert(json_file, 'train')

for json_file in val_jsons:
    convert(json_file, 'val')

print("✅ 변환 완료")
