import rclpy
from rclpy.node import Node
from ai_server_package_msgs.msg import CommandInfo
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
        _, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
        compressed = zlib.compress(buffer.tobytes())
        self.udp_socket.sendto(compressed, self.client_address)

    def close(self):
        self.udp_socket.close()

class commandPublisher(Node):
    def __init__(self, vehicle_receiver, port=6000):
        super().__init__(f"command_publisher{vehicle_receiver.vehicle_id}")
        self.receiver = vehicle_receiver
        self.vehicle_id = vehicle_receiver.vehicle_id
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.publisher = self.create_publisher(CommandInfo, '/drive', 10)
        
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        device = "cuda" if torch.cuda.is_available() else "cpu"
        self.yolo_seg_model = YOLO("admin/src/ai_server/ai_train/runs/segment/yolov8_epoch200/weights/best.pt").to(device)
        self.yolo_detect_model = YOLO("/home/pepsi/Downloads/yolo_detect.pt").to(device)

        self.prev_offset = 0.0
        self.prev_curvature = 0.0
        self.lidar_ranges = None
        self.angle_increment = None
        self.angle_min = None
        self.base_speed = 0.2
        self.stop_flag = False

        self.video_sender = VideoSender("192.168.0.134", 9999)


    def get_centroid_from_mask(self, roi_top, mask):
        y_indices, x_indices = np.where(mask > 0.5)
        if len(x_indices) == 0:
            return None
        cx = np.mean(x_indices)
        cy = np.mean(y_indices) + roi_top
        return (cx, cy)

    def draw_target_visualization(self, image, w, h, middle, border, dotted):
        target = None
        if middle and border:
            target = ((middle[0] + border[0]) / 2, (middle[1] + border[1]) / 2)
        elif middle:
            target = (middle[0] + 160, middle[1])
        elif border:
            target = (border[0] - 160, border[1])
        elif dotted:
            target = dotted  # 점선 단독일 때 그대로 target

        if middle:
            cv2.circle(image, (int(middle[0]), int(middle[1])), 5, (0, 255, 255), -1)
            cv2.putText(image, "middle", (int(middle[0])-20, int(middle[1])-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 2)
        if border:
            cv2.circle(image, (int(border[0]), int(border[1])), 5, (255, 255, 255), -1)
            cv2.putText(image, "border", (int(border[0])-20, int(border[1])-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2)
        if dotted:
            cv2.circle(image, (int(dotted[0]), int(dotted[1])), 5, (255, 200, 0), -1)
            cv2.putText(image, "dotted", (int(dotted[0])-20, int(dotted[1])-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,200,0), 2)
        if target:
            cv2.circle(image, (int(target[0]), int(target[1])), 5, (255, 150, 200), -1)
            cv2.putText(image, "target", (int(target[0])-20, int(target[1])-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200,100,150), 2)
            robot_pos = (int(w // 2), h - 10)
            cv2.arrowedLine(image, robot_pos, (int(target[0]), int(target[1])), (255, 100, 200), 2, tipLength=0.2)
        return image, target

    def lane_keeping(self, frame):
        h, w = frame.shape[:2]
        roi_top = int(h * 2 / 3)
        roi_frame = frame[roi_top:h, 0:w]

        results = self.yolo_seg_model(roi_frame, conf=0.8, verbose=False)
        annotated_frame = frame.copy()

        masks = results[0].masks.data.cpu().numpy() if results[0].masks is not None else None 
        classes = results[0].boxes.cls.cpu().numpy() if results[0].boxes is not None else None 

        image_center = w / 2
        offset = self.prev_offset

        if masks is None or classes is None:
            return annotated_frame, offset

        middle_centroid = None
        border_centroid = None
        dotted_centroid = None

        for i, cls in enumerate(classes):
            centroid = self.get_centroid_from_mask(roi_top, masks[i])
            if centroid is None:
                continue

            if cls == 0:
                border_centroid = centroid
            elif cls == 1:
                middle_centroid = centroid
            elif cls == 2:
                dotted_centroid = centroid

        # 🔸 시각화 및 offset 계산
        annotated_frame, target = self.draw_target_visualization(
            annotated_frame, w, h, middle_centroid, border_centroid, dotted_centroid
        )

        if target:
            offset = target[0] - image_center
            self.prev_offset = offset

        return annotated_frame, offset
    
    def obstacle_avoidance(self, frame):
        linear_x = 0.0
        if self.angle_min is None or self.angle_increment is None or self.lidar_ranges is None:
            self.get_logger().warn("LIDAR 정보 수신 전이므로 obstacle_avoidance 스킵")
            return frame, self.base_speed
        
        stopline_detected = False
        stopline_y_threshold = frame.shape[0] * 0.7

        results = self.yolo_detect_model(frame, conf=0.6, verbose=False)
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

                # 카메라 FOV와 LIDAR 각도 매핑
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
                        self.get_logger().info(f'{class_name} detected at {distance:.2f}m, Angle: {angle:.2f}deg')

                # 신호등: 빨간불/노란불 감지
                if class_name in ['redlight', 'yellowlight']:
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
                                self.get_logger().info('Stopline detected')
                                break
                        else:
                            stopline_detected = True
                            self.get_logger().info(f'{class_name} detected')
                            break

                    # 신호등 + 정지선 감지 시 정지
                    if stopline_detected:
                        self.stop_flag = True
                        self.get_logger().info(f'Stopping robot: {class_name} with stopline detected')

                 # 횡단보도 감지: 잠깐 정지 후 이동
                elif class_name == 'crosswalk':
                    self.stop_flag = False
                    self.base_speed = 0.1
                    self.get_logger().info('crosswalk detected')

                # 보행자 또는 핑키: 50m 이내 정지
                elif class_name in ['pedestrian', 'pinky'] and distance < 20.0:
                    self.stop_flag = True
                    self.get_logger().info(f'Stopping robot: {class_name} within 20cm')

                # 파란불 감지 시 이동
                elif class_name == 'greenlight':
                    self.stop_flag = False
                    self.get_logger().info('green light detected')
                
                # 속도 제한 표지판
                elif class_name == 'speedlimit_30':
                    self.stop_flag = False
                    self.base_speed = 0.1
                    self.get_logger().info('speed limit 30 detected')
                elif class_name == 'speedlimit_60':
                    self.stop_flag = False
                    self.base_speed = 0.2
                    self.get_logger().info('speed limit 60 detected')

                else:
                    self.stop_flag = False
                    self.base_speed = 0.2

        
        linear_x = 0.0 if self.stop_flag else self.base_speed
        return results[0].plot(), linear_x

    
    def timer_callback(self):
        frame = self.receiver.get_frame()
        if frame is None:
            return
        
        h, w = frame.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeff, (w, h), 1, (w, h))
        undistorted = cv2.undistort(frame, camera_matrix, dist_coeff, None, newcameramtx)

        # HSV로 변환
        hsv = cv2.cvtColor(undistorted, cv2.COLOR_BGR2HSV)

        # 밝기 줄이기 (0.0 ~ 1.0 사이 값)
        brightness_factor2 = 0.7                    
        saturation_factor = 1.5
        hsv[:,:,2] = np.clip(hsv[:,:,2] * brightness_factor2, 0, 255).astype(np.uint8)
        hsv[:,:,1] = np.clip(hsv[:,:,1] * saturation_factor, 0, 255).astype(np.uint8)

        result = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        
        detect_frame, linear_x = self.obstacle_avoidance(result)

        annotate_frame, offset = self.lane_keeping(detect_frame)

        self.get_logger().info(f"linear_x = {linear_x}, offset = {offset}")

        msg = CommandInfo()
        msg.vehicle_id = int(self.vehicle_id)
        msg.offset = float(offset)
        msg.linear_x = float(linear_x)
        self.publisher.publish(msg)

        cv2.imshow('frame', annotate_frame)
        cv2.waitKey(1)

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
