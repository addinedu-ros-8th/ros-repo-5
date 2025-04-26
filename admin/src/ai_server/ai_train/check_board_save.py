import cv2
import socket
import zlib
import numpy as np
import threading
import os
from datetime import datetime

PORT = 6000
MAX_PACKET_SIZE = 65536

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
    pinky1_receiver = VideoReceiver(1, port=6000)
    pinky1_thread = threading.Thread(target=pinky1_receiver.run)
    pinky1_thread.start()

    COUNT = 1
    save_base = '/home/pepsi/dev_ws/ros-repo-5/admin/dataset/video'
    os.makedirs(save_base, exist_ok=True)

    try:
        while True:
            frame = pinky1_receiver.get_frame()
            if frame is not None:

                cv2.imshow('Pinky1', frame)

            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC → 종료
                break
            elif key == ord('s'):
                print(save_base + f"check_board{COUNT}.jpg 저장 완료")
                cv2.imwrite(save_base + f"check_board{COUNT}.jpg", frame)
                COUNT += 1

    except KeyboardInterrupt:
        pass
    finally:
        pinky1_receiver.stop()
        pinky1_thread.join()
        cv2.destroyAllWindows()
        print("✅ [DONE] 수신 종료 및 정리 완료")

if __name__ == "__main__":
    main()
