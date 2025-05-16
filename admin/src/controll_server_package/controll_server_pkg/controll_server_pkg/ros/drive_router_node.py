import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import Twist
from ai_server.ai_server_package_msgs.msg import CommandInfo, DriveInfo
from controll_server_pkg.common.PIDController import PID
import time
import networkx as nx
import cv2
import numpy as np
from signal_processor import SignalProcessor

class DriveRouterNode(Node):
    def __init__(self, manager):
        super().__init__('drive_router_node')
        self.declare_parameter('goal_node', 'R')
        self.goal_node = self.get_parameter('goal_node').get_parameter_value().string_value

        # Declare ROS2 parameters (PID parameters and tolerance)
        self.declare_parameter('P', 1.0)
        self.declare_parameter('I', 0.0)
        self.declare_parameter('D', 0.0)
        self.declare_parameter('max_state', 5.0)
        self.declare_parameter('min_state', -5.0)
        self.declare_parameter('tolerance', 0.01)
        
        # Get initial parameter values for PID and tolerance
        P = self.get_parameter('P').value
        I = self.get_parameter('I').value
        D = self.get_parameter('D').value
        max_state = self.get_parameter('max_state').value
        min_state = self.get_parameter('min_state').value

        # Create PID instance and assign parameters
        self.pid = PID()
        self.pid.P = P
        self.pid.I = I
        self.pid.D = D
        self.pid.max_state = max_state
        self.pid.min_state = min_state

        self.G = nx.DiGraph()
        self.G.add_edges_from([
            ('A', 'R'), ('B', 'A'), ('C', 'B'), ('C', 'E'), ('E', 'K'), ('D', 'C'),
            ('F', 'G'), ('G', 'K'), ('K', 'O'),('O', 'N'), ('N', 'J'),('J', 'F'), ('N', 'Y'), ('Y', 'F'), ('K', 'U'),
            ('L', 'B'), ('L', 'H'), ('H', 'I'), ('I', 'M'), ('I', 'Z'), ('Z', 'Q'), ('M', 'Q'), ('Q', 'P'), ('P', 'L'),
            ('R', 'S'), ('S', 'T'), ('S', 'U'), ('T', 'L'), ('U', 'V'), ('V', 'D')
        ])

        self.positions = {
            "A": (192, 170), "B": (621, 88), "C": (1130, 64), "D": (1500, 115), "E": (1150, 159),
            "F": (282, 244), "G": (631, 216), "H": (1138, 274), "I": (1387, 328),
            "J": (252, 472), "K": (780, 502), "L": (904, 515), "M": (1466, 560), "Y": (450, 483), "Z": (1196, 552),
            "N": (267, 694), "O": (598, 734), "P": (1114, 817), "Q": (1448, 770),
            "R": (210, 944), "S": (555, 984), "T": (575, 857), "U": (1156, 941), "V": (1512, 904),
        }

        self.marker_positions = {
            "A": (0.213, 0.112), "B": (0.225, 0.178), "C": (0.209, 0.223), "D": (0.17, 0.225), "E": (0.166, 0.182),
            "F": (0.172, 0.094), "G": (0.179, 0.137), "H": (0.134, 0.149), "I": (0.104, 0.156),
            "J": (0.095, 0.025), "K": (0.77, 0.065), "L": (0.068, 0.071), "M": (0.027, 0.094), "Y": (0.1, 0.052), "Z": (0.051, 0.096),
            "N": (0.043, -0.021), "O": (0.01, -0.011), "P": (-0.031, 0.005), "Q": (-0.032, 0.036),
            "R": (-0.026, -0.089), "S": (-0.063, -0.075), "T": (-0.027, -0.045), "U": (-0.068, -0.022), "V": (-0.069, 0.015),
        }

        self.explicit_directions = {
            # 좌측 루프
            ('A', 'R'): 'left',
            ('R', 'S'): 'forward',
            ('S', 'T'): 'left',
            ('T', 'L'): 'left',
            ('L', 'B'): 'left',
            ('B', 'A'): 'forward',

            # 우측 루프
            ('C', 'B'): 'forward',
            ('C', 'E'): 'left',
            ('E', 'K'): 'left',
            ('K', 'O'): 'right',
            ('K', 'U'): 'left',
            ('O', 'N'): 'right',
            ('N', 'Y'): 'right',
            ('N', 'J'): 'forward',
            ('J', 'F'): 'right',
            ('Y', 'F'): 'right',
            ('F', 'G'): 'forward',
            ('G', 'K'): 'right',

            # 오른쪽 아랫부분
            ('P', 'L'): 'right',
            ('Q', 'P'): 'forward',
            ('Z', 'Q'): 'right',
            ('M', 'Q'): 'right',
            ('I', 'Z'): 'right',
            ('I', 'M'): 'right',
            ('H', 'I'): 'forward',
            ('L', 'H'): 'right',

            # 오른쪽 진입 경로
            ('V', 'D'): 'forward',
            ('U', 'V'): 'forward',
            ('S', 'U'): 'forward',

            # 중앙
            ('D', 'C'): 'left',
        }

        self.sp = SignalProcessor(window_size=5, alpha=0.3)
        self.behavior = {"forward": 0, "left": 1, "right": 2, 'stop': 3}

        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
        self.subscriber = self.create_subscription(CommandInfo, 'drive', self.yolo_callback, qos_profile)
        self.publisher = self.create_publisher(DriveInfo, '/nav_direction', 10)
        self.cmd_vel_pub_pinky1 = self.create_publisher(Twist, '/cmd_vel', qos_profile)
        self.cmd_vel_pub_pinky2 = self.create_publisher(Twist, '/cmd_vel', qos_profile)

        self.vehicle_id = None
        self.offset = 0
        self.radius = 0
        self.linear_x = 0
        self.angular_z = 0
        self.manager = manager
        self.arrived = False

        self.video = cv2.VideoCapture(2)
        time.sleep(2.0)
        self.timer = self.create_timer(0.1, self.timer_callback)

        ARUCO_DICT = {
            "DICT_4X4_100": cv2.aruco.DICT_4X4_100
        }

        self.aruco_dict_type = ARUCO_DICT["DICT_4X4_100"]
        self.k = np.load("admin/src/ai_server/ai_train/calib_images/camera_matrix.npy")
        self.d = np.load("admin/src/ai_server/ai_train/calib_images/distortion_coeffs.npy")
        self.marker_length = 0.03  # 3cm
        self.last_behavior = ""

        self.get_logger().info("goal_node:" + self.goal_node)
        self.get_logger().info("Drive Router Node Started")

    def heuristic(self, n1, n2):
        x1, y1 = self.positions[n1]
        x2, y2 = self.positions[n2]
        return np.hypot(x2 - x1, y2 - y1)

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
                    self.last_behavior = "stop"
                    return self.behavior[self.last_behavior]

                if nx.has_path(self.G, node, self.goal_node):
                    path = nx.shortest_path(self.G, node, self.goal_node)
                    if len(path) >= 2:
                        current_node = path[0]
                        next_node = path[1]
                        direction = self.explicit_directions[(current_node, next_node)]
                        self.last_behavior = self.behavior[direction]

                        msg = DriveInfo()
                        msg.direction = self.behavior[direction]
                        self.publisher.publish(msg)
                        
                        self.get_logger().info(f"current_node: {current_node}, next_node: {next_node}, 행동: {self.behavior[direction]}")

                cv2.aruco.drawDetectedMarkers(frame, corners)
                cv2.drawFrameAxes(frame, self.k, self.d, rvec, tvec, self.marker_length * 0.5)
        else:
            self.get_logger().warn("마커 인식 실패")
            return self.behavior["stop"]

        return self.last_behavior
    

    def timer_callback(self):
        ret, frame = self.video.read()
        if not ret:
            return

        behavior = self.pose_estimation(frame)

        twist = Twist()
        if self.arrived:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        else:
            # 네비게이션 행동에 따라 PID 제어 적용
            if behavior == 0:  # 전진
                twist.linear.x = self.linear_x
                # PID로 오프셋 기반 각속도 계산 (목표: 오프셋 = 0)
                angular_z = self.pid.update(self.offset)
                twist.angular.z = angular_z

            elif behavior == 1:  # 좌회전
                twist.linear.x = self.linear_x
                # PID로 좌회전 제어, 최소 회전 속도 보장
                base_output = self.pid.update(self.offset)
                safe_radius = max(abs(self.radius), 1e-3)
                curvature_gain = max(1.0, min(3.0, 2.0 / (safe_radius / 100.0)))
                angular_z = base_output * curvature_gain * 1.2
                twist.angular.z = min(angular_z, 0.7)   # 최소 좌회전 각속도 0.5

            elif behavior == 2:  # 우회전
                twist.linear.x = self.linear_x
                # PID로 우회전 제어, 최소 회전 속도 보장
                base_output = self.pid.update(self.offset)
                safe_radius = max(abs(self.radius), 1e-3)
                curvature_gain = max(1.0, min(3.0, 2.0 / (safe_radius / 100.0)))
                angular_z = base_output * curvature_gain * 1.2
                twist.angular.z = min(angular_z, -0.7)

            elif behavior == 3:  # 정지
                twist.linear.x = 0.0
                twist.angular.z = 0.0

        if self.pinky_num == 1:
            self.cmd_vel_pub_pinky1.publish(twist)
        elif self.pinky_num == 2:
            self.cmd_vel_pub_pinky2.publish(twist)

        self.get_logger().info(f"linear.x={twist.linear.x:.2f}, angular.z={twist.angular.z:.2f}")
        cv2.waitKey(1)

    def yolo_callback(self, msg):
        self.vehicle_id = msg.vehicle_id
        self.offset = msg.offset
        self.radius = msg.radius
        self. linear_x = msg.linear_x
        self. angular_z = msg.angular_z
        

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
