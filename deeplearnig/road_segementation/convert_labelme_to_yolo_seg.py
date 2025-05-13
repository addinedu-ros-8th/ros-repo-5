import os
import json
import cv2
from sklearn.model_selection import train_test_split
from tqdm import tqdm
import shutil
# 입력 및 출력 경로 설정 (raw string 사용으로 이스케이프 오류 방지)
input_dir = r'C:\dataset\road_dataset'
output_dir = r'C:\dataset\road_dataset_yolo'
# 하위 폴더 생성
images_dir = os.path.join(output_dir, 'images')
labels_dir = os.path.join(output_dir, 'labels')
os.makedirs(images_dir, exist_ok=True)
os.makedirs(labels_dir, exist_ok=True)
# JSON 파일 목록
json_files = [f for f in os.listdir(input_dir) if f.endswith('.json')]
# 변환 함수
def convert_labelme_json_to_yolo_seg(json_path, image_path, save_image_path, save_label_path):
    with open(json_path, 'r') as f:
        data = json.load(f)
    image = cv2.imread(image_path)
    if image is None:
        print(f":경고: 이미지 로드 실패: {image_path}")
        return False
    h, w = image.shape[:2]
    # 이미지 저장
    cv2.imwrite(save_image_path, image)
    with open(save_label_path, 'w') as out:
        for shape in data['shapes']:
            label = int(shape['label'])  # 예: '0', '1', '2'
            points = shape['points']
            norm_points = []
            for x, y in points:
                norm_x = x / w
                norm_y = y / h
                norm_points.extend([f"{norm_x:.6f}", f"{norm_y:.6f}"])
            out.write(f"{label} " + " ".join(norm_points) + "\n")
    return True
# 변환 수행
image_paths = []
for json_file in tqdm(json_files, desc="변환 중"):
    base_name = json_file.replace('.json', '')
    json_path = os.path.join(input_dir, json_file)
    image_path = os.path.join(input_dir, base_name + '.jpg')
    if not os.path.exists(image_path):
        print(f":경고: 이미지 없음: {image_path}")
        continue
    save_img_path = os.path.join(images_dir, base_name + '.jpg')
    save_lbl_path = os.path.join(labels_dir, base_name + '.txt')
    success = convert_labelme_json_to_yolo_seg(json_path, image_path, save_img_path, save_lbl_path)
    if success:
        image_paths.append(save_img_path)
    else:
        print(f":경고: 변환 실패: {json_file}")
# 학습/검증 셋 분할
train_imgs, val_imgs = train_test_split(image_paths, test_size=0.2, random_state=42)
for subset, img_list in zip(['train', 'val'], [train_imgs, val_imgs]):
    subset_img_dir = os.path.join(images_dir, subset)
    subset_lbl_dir = os.path.join(labels_dir, subset)
    os.makedirs(subset_img_dir, exist_ok=True)
    os.makedirs(subset_lbl_dir, exist_ok=True)
    for img_path in img_list:
        label_path = img_path.replace('/images/', '/labels/').replace('\\images\\', '\\labels\\').replace('.jpg', '.txt')
        if not os.path.exists(label_path):
            print(f":경고: 라벨 파일 누락: {label_path}")
            continue
        shutil.move(img_path, os.path.join(subset_img_dir, os.path.basename(img_path)))
        shutil.move(label_path, os.path.join(subset_lbl_dir, os.path.basename(label_path)))
print(":흰색_확인_표시: YOLOv8 segmentation 형식으로 변환 완료!")