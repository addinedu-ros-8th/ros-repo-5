import cv2
import socket
import zlib
import numpy as np
import threading
import os
from datetime import datetime
import pickle

PORT = 6001
MAX_PACKET_SIZE = 65536

# Pickle로부터 카메라 보정 파라미터 불러오기
with open("/home/pepsi/dev_ws/ros-repo-5/admin/src/calib_images/calibration_data.pickle", "rb") as f:
    calib_data = pickle.load(f)

camera_matrix = calib_data["camera_matrix"]
dist_coeff = calib_data["dist_coeff"]

class VideoReceiver:
    def __init__(self, pinky_num, port=PORT):
        self.pinky_num = pinky_num
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', port))
        self.sock.settimeout(2.0)
        self.running = True
        self.frame = None
        self.lock = threading.Lock()

    def run(self):
        try:
            while self.running:
                try:
                    data, addr = self.sock.recvfrom(MAX_PACKET_SIZE)
                    decompressed = zlib.decompress(data)
                    np_data = np.frombuffer(decompressed, dtype=np.uint8)
                    frame = cv2.imdecode(np_data, cv2.IMREAD_COLOR)
                    if frame is not None:
                        with self.lock:
                            self.frame = frame
                except:
                    continue
        finally:
            self.sock.close()

    def get_frame(self):
        with self.lock:
            return self.frame.copy() if self.frame is not None else None

    def stop(self):
        self.running = False

def main():
    pinky1_receiver = VideoReceiver(1, port=6001)
    pinky1_thread = threading.Thread(target=pinky1_receiver.run)
    pinky1_thread.start()

    recording = False
    out = None
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    save_base = '/home/pepsi/dev_ws/ros-repo-5/admin/dataset/video'
    os.makedirs(save_base, exist_ok=True)
    fps = 30.0

    try:
        while True:
            frame = pinky1_receiver.get_frame()
            if frame is not None:
                h, w = frame.shape[:2]
                newcameramtx, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeff, (w, h), 1, (w, h))
                undistorted = cv2.undistort(frame, camera_matrix, dist_coeff, None, newcameramtx)

                if recording and out is not None:
                    out.write(undistorted)

                cv2.imshow('Pinky1', undistorted)

            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC → 종료
                break
            elif key == ord('s'):  # s → 녹화 on/off 전환
                recording = not recording
                if recording:
                    print("🔴 녹화 시작")
                    frame_h, frame_w = frame.shape[:2]
                    save_path = os.path.join(
                        save_base,
                        f'driving_recording_{datetime.now().strftime("%Y%m%d_%H%M%S")}.mp4'
                    )
                    out = cv2.VideoWriter(save_path, fourcc, fps, (frame_w, frame_h))
                    if not out.isOpened():
                        print("❌ VideoWriter 초기화 실패")
                        recording = False
                        out = None
                else:
                    print("🛑 녹화 종료")
                    if out is not None:
                        out.release()
                        out = None

    except KeyboardInterrupt:
        pass
    finally:
        pinky1_receiver.stop()
        pinky1_thread.join()
        if out is not None:
            out.release()
        cv2.destroyAllWindows()
        print("✅ [DONE] 수신 종료 및 정리 완료")

if __name__ == "__main__":
    main()
