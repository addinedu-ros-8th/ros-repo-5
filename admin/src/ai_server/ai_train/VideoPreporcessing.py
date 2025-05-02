import os
import cv2
import glob
import numpy as np
from tqdm import tqdm

class DataPreprocessing:
    def __init__(self, video_dir='/home/pepsi/dev_ws/ros-repo-5/admin/src/ai_server/dataset/video', output_dir='/home/pepsi/dev_ws/ros-repo-5/admin/src/ai_server/dataset/', frame_rate=2):
        self.video_dir = video_dir
        self.output_dir = output_dir
        self.frame_rate = frame_rate

        os.makedirs(os.path.join(self.output_dir, "images"), exist_ok=True)

    def extract_images(self):
        """비디오를 프레임으로 추출하고 캡션을 부여"""
        video_folder = glob.glob(os.path.join(self.video_dir, "*.mp4"))

        for video_path in tqdm(video_folder, total=len(video_folder)):
            
            video_name = "driving_frame"

            cap = cv2.VideoCapture(video_path)

            frame_idx = 0
            saved_count = 1

            while cap.isOpened():
                ret, frame = cap.read()
                if not ret:
                    break

                if frame_idx % self.frame_rate == 0:
                    # HSV로 변환
                    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

                    # 밝기 줄이기 (0.0 ~ 1.0 사이 값)
                    brightness_factor2 = 0.7                    
                    saturation_factor = 1.5
                    hsv[:,:,2] = np.clip(hsv[:,:,2] * brightness_factor2, 0, 255).astype(np.uint8)
                    hsv[:,:,1] = np.clip(hsv[:,:,1] * saturation_factor, 0, 255).astype(np.uint8)

                    result = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

                    image_name = f"{video_name}_{saved_count:04d}.jpg"
                    image_path = os.path.join(self.output_dir, "images", image_name)
                    cv2.imwrite(image_path, result)

                    saved_count += 1

                frame_idx += 1

            cap.release()


if __name__ == "__main__":
    dp = DataPreprocessing()
    dp.extract_images()