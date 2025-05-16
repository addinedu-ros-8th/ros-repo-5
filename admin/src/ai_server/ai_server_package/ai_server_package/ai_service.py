import rclpy
from rclpy.node import Node
from ai_server_package_msgs.msg import CommandInfo, DriveInfo
from sensor_msgs.msg import LaserScan
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
    def __init__(self, vehicle_id, port=PORT):
        self.vehicle_id = vehicle_id
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

class VideoSender:
    def __init__(self, UDP_IP, UDP_PORT):
        self.client_address = (UDP_IP, UDP_PORT)
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send_frame(self, frame):
        _, buffer = cv2.imencode('.jpg', frame)
        compressed = zlib.compress(buffer.tobytes())
        self.udp_socket.sendto(compressed, self.client_address)

    def close(self):
        self.udp_socket.close()


class commandPublisher(Node):
    def __init__(self, vehicle_receiver, port=6000):
        super().__init__(f"command_publisher{vehicle_receiver.vehicle_id}")
        self.receiver = vehicle_receiver
        self.vehicle_id = vehicle_receiver.vehicle_id
        self.direction_sub = self.create_subscription(DriveInfo, '/nav_direction', self.direction_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.publisher = self.create_publisher(CommandInfo, '/drive', 10)
        
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        device = "cuda" if torch.cuda.is_available() else "cpu"
        self.yolo_seg_model = YOLO("/home/pepsi/Downloads/yolo_seg.pt").to(device)
        self.yolo_detect_model = YOLO("/home/pepsi/Downloads/yolo_detect.pt").to(device)

        self.direction = 0
        self.prev_offset = 0.0
        self.prev_curvature = 0.0
        self.lidar_ranges = None
        self.angle_increment = None
        self.angle_min = None
        self.base_speed = 0.5
        self.stop_flag = False

        self.video_sender = VideoSender("192.168.0.134", 9999)


    def calculate_curvature(self, x_vals, y_vals):
        if len(x_vals) < 3 or len(y_vals) < 3:
            return None
        fit = np.polyfit(y_vals, x_vals, 2)
        A = fit[0]
        y_eval = np.max(y_vals)
        curvature_radius = ((1 + (2*A*y_eval + fit[1])**2)**1.5) / np.abs(2*A)
        return curvature_radius
    

    def draw_lane_visualization(self, image, left_pts, right_pts):
        if len(left_pts) < 2 or len(right_pts) < 2:
            return image

        polygon = np.concatenate((left_pts, right_pts[::-1]), axis=0).astype(np.int32)
        overlay = image.copy()
        cv2.fillPoly(overlay, [polygon], color=(255, 255, 0))
        image = cv2.addWeighted(overlay, 0.4, image, 0.6, 0)

        cv2.polylines(image, [left_pts.astype(np.int32)], False, (0, 255, 255), 3)
        cv2.polylines(image, [right_pts.astype(np.int32)], False, (255, 255, 255), 3)

        mid_pts = ((left_pts + right_pts[:len(left_pts)]) / 2).astype(np.int32)
        cv2.polylines(image, [mid_pts], False, (0, 255, 0), 2)

        return image
    
    def lane_keeping(self, frame):
        results = self.yolo_seg_model(frame)
        if not results or results[0].masks is None or results[0].boxes is None:
            return results[0].plot(), 0.0, 0.0
        
        masks = results[0].masks.data.cpu().numpy()
        classes = results[0].boxes.cls.cpu().numpy()
        
        h, w = frame.shape[:2]

        right_x, right_y, left_x, left_y = [], [], [], []

        dashed_candidates = []
        for i, cls in enumerate(classes):
            mask = masks[i]
            ys, xs = np.where(mask > 0)
            if len(xs) == 0:
                continue

            x_mean = np.mean(xs)

            if cls == 0:
                right_x.extend(xs)
                right_y.extend(ys)
            elif cls in [1, 2]:
                if (self.direction == 0) or (self.direction == 1 and x_mean < w // 2) or (self.direction == 2 and x_mean > w // 2):
                    dashed_candidates.append((x_mean, xs, ys))

        if dashed_candidates:
            _, xs_best, ys_best = min(dashed_candidates, key=lambda x: abs(x[0] - w // 2))
            left_x.extend(xs_best)
            left_y.extend(ys_best)

        if len(left_x) >= 2 and len(right_x) >= 2:
            left_pts = np.array(sorted(zip(left_x, left_y), key=lambda p: p[1]))
            right_pts = np.array(sorted(zip(right_x, right_y), key=lambda p: p[1]))
        elif len(right_x) >= 2:
            right_pts = np.array(sorted(zip(right_x, right_y), key=lambda p: p[1]))
            left_pts = np.array([[x - 550, y] for x, y in right_pts], dtype=np.int32)
        elif len(left_x) >= 2:
            left_pts = np.array(sorted(zip(left_x, left_y), key=lambda p: p[1]))
            right_pts = np.array([[x + 550, y] for x, y in left_pts], dtype=np.int32)
        else:
            offset = float(self.prev_offset)
            radius = float(self.prev_curvature)
            return results[0].plot(), offset, radius

        mid_pts = (left_pts + right_pts[:len(left_pts)]) / 2
        offset = mid_pts[-1][0] - w // 2
        curvature = self.calculate_curvature(mid_pts[:, 0], mid_pts[:, 1])
        visual = self.draw_lane_visualization(frame.copy(), left_pts, right_pts)

        offset = float(offset)
        radius = float(curvature if curvature else 0.0)

        self.prev_offset = offset
        self.prev_curvature = radius

        return visual, offset, radius

    
    def obstacle_avoidance(self, frame):
        linear_x, angular_z = 0.0, 0.0
        stopline_detected = False
        stopline_y_threshold = frame.shape[0] * 0.7  # 이미지 하단 30% 내 정지선 감지

        results = self.yolo_detect_model(frame)
        for result in results:
            boxes = result.boxes
            for box in boxes:
                cls = int(box.cls[0])
                conf = float(box.conf[0])
                class_name = result.names[cls]

                if conf < 0.5:
                    continue

                # 바운딩 박스 중심 계산
                x1, y1, x2, y2 = box.xyxy[0]
                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2

                # 카메라 FOV와 LIDAR 각도 매핑 (FOV 60도 가정)
                fov = 60.0
                img_width = frame.shape[1]
                angle_per_pixel = fov / img_width
                angle = (center_x - img_width / 2) * angle_per_pixel

                # LIDAR 각도 인덱스 계산
                lidar_angle = angle * np.pi / 180.0
                index = int((lidar_angle - self.angle_min) / self.angle_increment)

                # 유효한 인덱스 확인 및 거리 측정
                distance = float('inf')
                if 0 <= index < len(self.lidar_ranges):
                    distance = self.lidar_ranges[index]
                    if distance != float('inf') and distance != float('nan'):
                        self.get_logger().info(
                            f'{class_name} detected at {distance:.2f}m, Angle: {angle:.2f}deg')

                # 보행자 또는 핑키: 50m 이내 정지
                if class_name in ['pedestrian', 'pinky'] and distance < 50.0:
                    self.stop_flag = True
                    self.get_logger().info(f'Stopping robot: {class_name} within 50m')

                # 신호등: 빨간불/노란불 감지
                elif class_name in ['redlight', 'yellowlight']:
                    # 정지선 클래스 확인
                    for inner_box in boxes:
                        inner_cls = int(inner_box.cls[0])
                        inner_conf = float(inner_box.conf[0])
                        inner_class_name = result.names[inner_cls]
                        if inner_conf < 0.5:
                            continue
                        if inner_class_name == 'stopline':
                            # 정지선의 바운딩 박스 하단 y 좌표 확인
                            _, _, _, stopline_y2 = inner_box.xyxy[0]
                            if stopline_y2 > stopline_y_threshold:
                                stopline_detected = True
                                self.get_logger().info('Stopline detected near bottom of frame')
                                break

                    # 신호등 + 정지선 감지 시 정지
                    if stopline_detected:
                        self.stop_flag = True
                        self.get_logger().info(f'Stopping robot: {class_name} with stopline detected')

                # 속도 제한 표지판
                elif class_name == 'speedlimit_30':
                    self.base_speed = 0.3
                elif class_name == 'speedlimit_60':
                    self.base_speed = 0.5

        if self.stop_flag:
            linear_x = 0.0
            angular_z = 0.0
        else:
            linear_x = self.base_speed
            angular_z = 0.0

        return results[0].plot(), linear_x, angular_z

    
    def timer_callback(self):
        frame = self.receiver.get_frame()
        if frame is None:
            return
        
        h, w = frame.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeff, (w, h), 1, (w, h))
        undistorted = cv2.undistort(frame, camera_matrix, dist_coeff, None, newcameramtx)
        
        detect_frame, offset, radius = self.obstacle_avoidance(undistorted)

        annotate_frame, linear_x, angular_z = self.lane_keeping(detect_frame)

        msg = CommandInfo()
        msg.vehicle_id = self.vehicle_id
        msg.offset = offset
        msg.radius = radius
        msg.linear_x = linear_x
        msg.angular_z = angular_z
        self.publisher.publish(msg)

        cv2.imshow('frame', annotate_frame)
        cv2.waitKey(1)

        self.video_sender.send_frame(annotate_frame)

    def direction_callback(self, msg):
        self.direction = msg.direction


    def lidar_callback(self, msg):
        self.lidar_ranges = msg.ranges
        self.angle_increment = msg.angle_increment
        self.angle_min = msg.angle_min

def main():
    rclpy.init()

    vehicle1_receiver = VideoReceiver(1, port=6000)
    vehicle1_receiver_thread = threading.Thread(target=vehicle1_receiver.run)
    vehicle1_receiver_thread.start()

    vehicle2_receiver = VideoReceiver(2, port=6001)
    vehicle2_receiver_thread = threading.Thread(target=vehicle2_receiver.run)
    vehicle2_receiver_thread.start()

    node1 = commandPublisher(vehicle1_receiver)
    node2 = commandPublisher(vehicle2_receiver)

    executor = MultiThreadedExecutor()
    executor.add_node(node1)
    executor.add_node(node2)

    try:
        while rclpy.ok():
            rclpy.spin_once(node1, timeout_sec=0.01)
            rclpy.spin_once(node2, timeout_sec=0.01)

    except KeyboardInterrupt:
        pass

    finally:
        node1.destroy_node()
        node2.destroy_node()
        rclpy.shutdown()

        vehicle1_receiver.stop()
        vehicle2_receiver.stop()
        vehicle1_receiver_thread.join()
        vehicle2_receiver_thread.join()

        node1.video_sender.close()
        node2.video_sender.close()

        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
