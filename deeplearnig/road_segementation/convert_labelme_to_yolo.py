import os
import json
import cv2
from sklearn.model_selection import train_test_split
from tqdm import tqdm

# 경로 설정
input_dir = '/home/vit/dev_ws/project/road_dataset'
output_dir = '/home/vit/dev_ws/project/road_dataset_yolo'

# YOLO 형식 저장 경로
images_dir = os.path.join(output_dir, 'images')
labels_dir = os.path.join(output_dir, 'labels')
os.makedirs(images_dir, exist_ok=True)
os.makedirs(labels_dir, exist_ok=True)

# JSON 파일 수집
json_list = [f for f in os.listdir(input_dir) if f.endswith('.json')]

# 변환 함수
def convert_to_yolo(json_path, image_path, save_image_path, save_label_path):
    with open(json_path, 'r') as f:
        data = json.load(f)

    img = cv2.imread(image_path)
    h, w = img.shape[:2]
    cv2.imwrite(save_image_path, img)

    with open(save_label_path, 'w') as out:
        for shape in data['shapes']:
            label = shape['label']
            points = shape['points']

            # 최소 사각형 구하기
            x_coords = [p[0] for p in points]
            y_coords = [p[1] for p in points]
            xmin, xmax = min(x_coords), max(x_coords)
            ymin, ymax = min(y_coords), max(y_coords)

            # 중심, 너비, 높이 → YOLO 형식으로 정규화
            x_center = (xmin + xmax) / 2 / w
            y_center = (ymin + ymax) / 2 / h
            box_width = (xmax - xmin) / w
            box_height = (ymax - ymin) / h

            out.write(f"{label} {x_center:.6f} {y_center:.6f} {box_width:.6f} {box_height:.6f}\n")

# 전체 변환
image_paths = []
label_paths = []

for json_file in tqdm(json_list):
    json_path = os.path.join(input_dir, json_file)
    img_file = json_file.replace('.json', '.jpg')
    img_path = os.path.join(input_dir, img_file)

    if not os.path.exists(img_path):
        continue

    save_img_path = os.path.join(images_dir, img_file)
    save_lbl_path = os.path.join(labels_dir, img_file.replace('.jpg', '.txt'))

    convert_to_yolo(json_path, img_path, save_img_path, save_lbl_path)
    image_paths.append(save_img_path)
    label_paths.append(save_lbl_path)

# 학습/검증 셋 분리
train_imgs, val_imgs = train_test_split(image_paths, test_size=0.2, random_state=42)

for subset, img_list in zip(['train', 'val'], [train_imgs, val_imgs]):
    for img_path in img_list:
        label_path = img_path.replace('/images/', '/labels/').replace('.jpg', '.txt')

        subset_img_dir = os.path.join(output_dir, 'images', subset)
        subset_lbl_dir = os.path.join(output_dir, 'labels', subset)
        os.makedirs(subset_img_dir, exist_ok=True)
        os.makedirs(subset_lbl_dir, exist_ok=True)

        os.rename(img_path, os.path.join(subset_img_dir, os.path.basename(img_path)))
        os.rename(label_path, os.path.join(subset_lbl_dir, os.path.basename(label_path)))

print("✅ 변환 완료! YOLO 학습을 시작할 준비가 되었습니다.")
