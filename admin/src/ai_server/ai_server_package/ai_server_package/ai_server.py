import rclpy
from rclpy.node import Node
from ai_server_package_msgs.msg import DetectionWithPinky
from rclpy.executors import MultiThreadedExecutor

import socket
import zlib
import cv2
import threading
import numpy as np
from ultralytics import YOLO
import torch
import pickle

# 수신 포트 설정
PORT = 9999
MAX_PACKET_SIZE = 65536

with open("admin/src/ai_server/ai_train/calib_images/calibration_data.pickle", "rb") as f:
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


class commandPublisher(Node):
    def __init__(self, pinky_receiver):
        super().__init__(f"command_publisher{pinky_receiver.pinky_num}")
        self.receiver = pinky_receiver
        self.publisher = self.create_publisher(DetectionWithPinky, '/drive', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        device = "cuda" if torch.cuda.is_available() else "cpu"
        self.yolo_model = YOLO("admin/src/ai_server/ai_train/runs/segment/yolov8_epoch300/weights/best.pt").to(device)

        self.last_offset = 0.0
        self.last_radius = 1000.0  # 직선 기본값
        self.pinky_num = 0
        self.pinky_num = pinky_receiver.pinky_num
        self.latest_frame = None


    def estimate_curvature(self, left_pts, right_pts):
        try:
            if left_pts is not None and right_pts is not None:
                min_len = min(len(left_pts), len(right_pts))
                left_cut = left_pts[:min_len, 0, :] if len(left_pts.shape) == 3 else left_pts[:min_len]
                right_cut = right_pts[:min_len, 0, :] if len(right_pts.shape) == 3 else right_pts[:min_len]
                mid_pts = (left_cut + right_cut) / 2.0
            elif left_pts is not None:
                mid_pts = (left_pts[:, 0, :] if len(left_pts.shape) == 3 else left_pts) + np.array([160.0, 0])
            elif right_pts is not None:
                mid_pts = (right_pts[:, 0, :] if len(right_pts.shape) == 3 else right_pts) - np.array([160.0, 0])
            else:
                return self.last_radius

            if len(mid_pts) < 5:
                return self.last_radius

            poly = np.polyfit(mid_pts[:, 1], mid_pts[:, 0], 2)
            A = poly[0]
            y_eval = np.max(mid_pts[:, 1])
            curvature = ((1 + (2*A*y_eval + poly[1])**2)**1.5) / np.abs(2*A)
            curvature = 0.9 * self.last_radius + 0.1 * curvature
            return curvature
        except Exception as e:
            self.get_logger().warn(f"곡률 계산 실패: {e}")
            return self.last_radius

    def publish_control(self, offset, radius, fallback=False):
        msg = DetectionWithPinky()
        msg.pinky_num = self.pinky_num
        msg.offset = float(offset)
        msg.radius = float(radius)
        self.publisher.publish(msg)

        if fallback:
            self.get_logger().warn(f"[Fallback] 차선 없음 → 이전값 사용 offset={offset:.2f}, radius={radius:.1f}")
        else:
            self.get_logger().info(f"[LaneControl] offset={offset:.2f}, radius={radius:.1f}")

    def timer_callback(self):
        frame = self.receiver.get_frame()
        if frame is None:
            return

        h, w = frame.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeff, (w, h), 1, (w, h))
        undistorted = cv2.undistort(frame, camera_matrix, dist_coeff, None, newcameramtx)

        results = self.yolo_model(undistorted, verbose=False)
        mask = results[0].masks.data.cpu().numpy() if results[0].masks is not None else None

        if mask is None or len(mask) == 0:
            self.publish_control(self.last_offset, self.last_radius, fallback=True)
            return

        # ROI 설정 (상단 20%부터 하단 90%까지)
        roi_y_start = int(h * 0.2)
        roi_y_end = int(h * 0.9)

        left_mask = (mask[0] * 255).astype(np.uint8) if len(mask) > 0 else None
        right_mask = (mask[1] * 255).astype(np.uint8) if len(mask) > 1 else None

        left_pts = cv2.findNonZero(left_mask[roi_y_start:roi_y_end, :]) if left_mask is not None else None
        right_pts = cv2.findNonZero(right_mask[roi_y_start:roi_y_end, :]) if right_mask is not None else None

        # ROI 시작 위치로 포인트 보정
        if left_pts is not None:
            left_pts[:, 0, 1] += roi_y_start
        if right_pts is not None:
            right_pts[:, 0, 1] += roi_y_start

        # 차선 폭 (픽셀 단위, 상황에 맞게 조정)
        lane_width_px = 120  # 180에서 120으로 줄임, 실제 차선 폭에 맞게 조정 필요
        image_center = w / 2

        # 차선 중심과 offset 계산
        if left_pts is not None and right_pts is not None:
            left_x = np.mean(left_pts[:, 0, 0])
            right_x = np.mean(right_pts[:, 0, 0])
            lane_center = (left_x + right_x) / 2
        elif left_pts is not None:
            left_x = np.mean(left_pts[:, 0, 0])
            lane_center = left_x + lane_width_px / 2  # 차선 폭의 절반으로 조정
        elif right_pts is not None:
            right_x = np.mean(right_pts[:, 0, 0])
            lane_center = right_x - lane_width_px / 2  # 차선 폭의 절반으로 조정
        else:
            self.publish_control(self.last_offset, self.last_radius, fallback=True)
            return

        # Offset 계산 (정규화된 값, -1에서 1 사이)
        offset = (image_center - lane_center) / (w / 2)
        radius = self.estimate_curvature(left_pts, right_pts)

        # 시각화 프레임 생성
        vis_frame = frame.copy()
        
        # 감지된 차선 그리기
        for i, m in enumerate(mask):
            binary = (m * 255).astype(np.uint8)
            color = (0, 255, 0) if i == 0 else (255, 0, 0)
            contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(vis_frame, contours, -1, color, 2)

        self.latest_frame = vis_frame
        self.last_offset = offset
        self.last_radius = radius
        self.publish_control(offset, radius)


def main():
    rclpy.init()

    pinky1_receiver = VideoReceiver(1, port=6000)
    pinky1_thread = threading.Thread(target=pinky1_receiver.run)
    pinky1_thread.start()

    pinky2_receiver = VideoReceiver(2, port=6001)
    pinky2_thread = threading.Thread(target=pinky2_receiver.run)
    pinky2_thread.start()

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



if __name__ == "__main__":
    main()
