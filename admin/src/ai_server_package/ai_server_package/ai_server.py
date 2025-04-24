import rclpy
from rclpy.node import Node
from ai_server_package_msgs.msg import CmdAndPinkyNum as Cmd
from rclpy.executors import MultiThreadedExecutor

import socket
import zlib
import cv2
import numpy as np
import threading
import os
from datetime import datetime

# 수신 포트 설정
PORT = 9999
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


class commandPublisher(Node):
    def __init__(self, pinky_receiver):
        super().__init__(f"command_publisher{pinky_receiver.pinky_num}")
        self.receiver = pinky_receiver
        self.publisher = self.create_publisher(Cmd, '/drive', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.latest_frame = None

    def timer_callback(self):
        frame = self.receiver.get_frame()
        if frame is None:
            return
        
        self.latest_frame = frame

        msg = Cmd()
        msg.x = 1.0
        msg.y = 2.0
        msg.pinky_num = self.receiver.pinky_num
        self.publisher.publish(msg)


def main():
    rclpy.init()

    pinky1_receiver = VideoReceiver(1, port=6000)
    pinky1_thread = threading.Thread(target=pinky1_receiver.run)
    pinky1_thread.start()

    pinky2_receiver = VideoReceiver(2, port=6001)
    pinky2_thread = threading.Thread(target=pinky2_receiver.run)
    pinky2_thread.start()

     # 🧠 ROS 노드 실행
    node1 = commandPublisher(pinky1_receiver)
    node2 = commandPublisher(pinky2_receiver)

    executor = MultiThreadedExecutor()
    executor.add_node(node1)
    executor.add_node(node2)

    try:
        while rclpy.ok():
            if node1.latest_frame is not None:
                cv2.imshow("Pinky1", node1.latest_frame)
            if node2.latest_frame is not None:
                cv2.imshow("Pinky2", node2.latest_frame)

            if cv2.waitKey(1) == 27:
                print("🛑 ESC → 종료 요청")
                rclpy.shutdown()
                break

            rclpy.spin_once(node1, timeout_sec=0.01)
            rclpy.spin_once(node2, timeout_sec=0.01)

    except KeyboardInterrupt:
        pass
    finally:
        node1.destroy_node()
        node2.destroy_node()
        rclpy.shutdown()

        pinky1_receiver.stop()
        pinky2_receiver.stop()
        pinky1_thread.join()
        pinky2_thread.join()

        cv2.destroyAllWindows()
        print("✅ [DONE] 종료 완료")



if __name__ == "__main__":
    main()
