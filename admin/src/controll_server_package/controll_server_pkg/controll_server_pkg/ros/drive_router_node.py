import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import Twist
from ai_server.ai_server_package_msgs.msg import DetectionWithPinky, YoloSegResult
import time
import networkx as nx
import cv2
import numpy as np
from signal_processor import SignalProcessor

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, error):
        self.integral += error
        derivative = error - self.prev_error
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

class DriveRouterNode(Node):
    def __init__(self):
        super().__init__('drive_router_node')
        self.declare_parameter('goal_node', 'Q')
        self.goal_node = self.get_parameter('goal_node').get_parameter_value().string_value

        self.G = nx.DiGraph()
        self.G.add_edges_from([
            ('A', 'R'), ('B', 'A'), ('C', 'B'), ('C', 'E'), ('E', 'K'), ('D', 'C'),
            ('F', 'G'), ('G', 'K'), ('H', 'I'), ('I', 'M'),
            ('J', 'F'), ('K', 'O'), ('L', 'B'), ('L', 'H'), ('M', 'Q'), ('Q', 'P'),
            ('N', 'J'), ('O', 'N'), ('P', 'L'), ('P', 'L'),
            ('R', 'S'), ('S', 'T'), ('S', 'U'), ('T', 'L'), ('U', 'V'), ('V', 'D')
        ])

        self.positions = {
            "A": (192, 170), "B": (621, 88), "C": (1130, 64), "D": (1500, 115), "E": (1150, 159),
            "F": (282, 244), "G": (631, 216), "H": (1138, 274), "I": (1387, 328),
            "J": (252, 472), "K": (780, 502), "L": (904, 515), "M": (1466, 560),
            "N": (267, 694), "O": (598, 734), "P": (1114, 817), "Q": (1448, 770),
            "R": (210, 944), "S": (555, 984), "T": (575, 857), "U": (1156, 941), "V": (1512, 904)
        }

        self.marker_positions = {
            "A": (0.213, 0.112), "B": (0.225, 0.178), "C": (0.209, 0.223), "D": (0.17, 0.225), "E": (0.166, 0.182),
            "F": (0.172, 0.094), "G": (0.179, 0.137), "H": (0.134, 0.149), "I": (0.104, 0.156),
            "J": (0.095, 0.025), "K": (0.77, 0.065), "L": (0.068, 0.071), "M": (0.027, 0.094),
            "N": (0.043, -0.021), "O": (0.01, -0.011), "P": (-0.031, 0.005), "Q": (-0.032, 0.036),
            "R": (-0.026, -0.089), "S": (-0.063, -0.075), "T": (-0.027, -0.045), "U": (-0.068, -0.022), "V": (-0.069, 0.015)
        }

        self.sp = SignalProcessor(window_size=5, alpha=0.3)
        self.behavior = {"forward": 0, "left": 1, "right": 2, 'stop': 3}
        self.pid_controller = PIDController(kp=0.005, ki=0.0, kd=0.001)

        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
        self.subscription = self.create_subscription(DetectionWithPinky, 'drive', self.listener_callback, qos_profile)
        self.cmd_vel_pub_pinky1 = self.create_publisher(Twist, '/pinky1/cmd_vel', qos_profile)
        self.cmd_vel_pub_pinky2 = self.create_publisher(Twist, '/pinky2/cmd_vel', qos_profile)

        self.latest_seg_result = None
        self.arrived = False

        self.video = cv2.VideoCapture(2)
        time.sleep(2.0)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.aruco_dict_type = cv2.aruco.DICT_4X4_100
        self.k = np.load("admin/src/ai_server/ai_train/calib_images/camera_matrix.npy")
        self.d = np.load("admin/src/ai_server/ai_train/calib_images/distortion_coeffs.npy")
        self.marker_length = 0.03

        self.get_logger().info("Drive Router Node Started")

    def heuristic(self, n1, n2):
        x1, y1 = self.positions[n1]
        x2, y2 = self.positions[n2]
        return np.hypot(x2 - x1, y2 - y1)

    def get_direction(self, current_pos, next_pos, goal_pos):
        vec_current_to_next = np.array(next_pos) - np.array(current_pos)
        vec_current_to_goal = np.array(goal_pos) - np.array(current_pos)
        angle = np.degrees(np.arctan2(vec_current_to_goal[1], vec_current_to_goal[0]) - np.arctan2(vec_current_to_next[1], vec_current_to_next[0]))
        angle = (angle + 360) % 360
        if angle < 45 or angle > 315:
            return "forward"
        elif 45 <= angle <= 135:
            return "left"
        elif 225 <= angle <= 315:
            return "right"
        else:
            return "stop"

    def find_nearest_node(self, robot_pos):
        return min(self.marker_positions.items(), key=lambda x: np.linalg.norm(np.array(x[1]) - np.array(robot_pos)))[0]

    def pose_estimation(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = cv2.aruco.getPredefinedDictionary(self.aruco_dict_type)
        detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())
        corners, ids, _ = detector.detectMarkers(gray)
        direction = 'stop'

        if self.goal_node not in self.marker_positions:
            self.get_logger().error(f"유효하지 않은 goal_node: {self.goal_node}")
            self.arrived = True
            return self.behavior["stop"]

        if ids is not None and len(corners) > 0:
            for i in range(len(ids)):
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], self.marker_length, self.k, self.d)
                pos = tvec[0][0]
                x, y = pos[0], pos[1]
                robot_pos = (round(self.sp.moving_average(x), 3), round(self.sp.moving_average(y), 3))
                node = self.find_nearest_node(robot_pos)

                if np.linalg.norm(np.array(robot_pos) - np.array(self.marker_positions[self.goal_node])) < 0.03:
                    self.get_logger().info("도착")
                    self.arrived = True
                    return self.behavior["stop"]

                if nx.has_path(self.G, node, self.goal_node):
                    path = nx.shortest_path(self.G, node, self.goal_node)
                    if len(path) >= 2:
                        current_node, next_node = path[0], path[1]
                        current_pos = self.marker_positions[current_node]
                        next_pos = self.marker_positions[next_node]
                        goal_pos = self.marker_positions[self.goal_node]
                        direction = self.get_direction(current_pos, next_pos, goal_pos)

                cv2.aruco.drawDetectedMarkers(frame, corners)
                cv2.drawFrameAxes(frame, self.k, self.d, rvec, tvec, self.marker_length * 0.5)
        else:
            self.get_logger().warn("마커 인식 실패")
            return self.behavior["stop"]

        return self.behavior[direction]

    def listener_callback(self, msg):
        if msg.results:
            self.latest_seg_result = msg

    def timer_callback(self):
        ret, frame = self.video.read()
        if not ret or self.latest_seg_result is None:
            return

        seg_result: YoloSegResult = self.latest_seg_result.results[0]
        mask_flat = np.array(seg_result.mask, dtype=np.float32)
        mask_img = mask_flat.reshape((seg_result.mask_height, seg_result.mask_width)).astype(np.uint8) * 255

        M = cv2.moments(mask_img)
        cx = int(M["m10"] / M["m00"]) if M["m00"] != 0 else mask_img.shape[1] // 2
        img_center = mask_img.shape[1] // 2
        error = cx - img_center

        behavior = self.pose_estimation(frame)

        pid_output = self.pid_controller.compute(error)
        pid_output = max(min(pid_output, 1.0), -1.0)

        twist = Twist()
        if self.arrived:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        else:
            twist.linear.x = 0.5
            if behavior == 0:
                twist.angular.z = -pid_output
            elif behavior == 1:
                twist.angular.z = max(min(0.8 - pid_output, 1.0), -1.0)
            elif behavior == 2:
                twist.angular.z = max(min(-0.8 - pid_output, 1.0), -1.0)
            elif behavior == 3:
                twist.linear.x = 0.0
                twist.angular.z = 0.0

        if self.latest_seg_result.pinky_num == 1:
            self.cmd_vel_pub_pinky1.publish(twist)
        elif self.latest_seg_result.pinky_num == 2:
            self.cmd_vel_pub_pinky2.publish(twist)

        self.get_logger().info(f"linear.x={twist.linear.x:.2f}, angular.z={twist.angular.z:.2f}")
        cv2.imshow("frame", frame)
        cv2.waitKey(1)

    def destroy_node(self):
        self.video.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DriveRouterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
