import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import Twist
from ai_server_package_msgs.msg import CommandInfo
from controll_server_pkg.common.PIDController import PID
import time
import networkx as nx
import cv2
import numpy as np
from signal_processor import SignalProcessor
from controll_server_pkg.common.manager import ServiceManager
from controll_server_package_msgs.srv import TaxiEvent

class DriveRouterNode(Node):
    def __init__(self, manager: ServiceManager):
        super().__init__('drive_router_node')
        self.manager = manager
        self.manager.set_drive_router_node(self.handle_message)

        self.goal_node = None

        self.pid = PID(kp=0.010, ki=0.0, kd=0.04)

        self.G = nx.DiGraph()
        self.G.add_edges_from([
            ('B', 'A'), ('A', 'R'), ('C', 'B'), ('C', 'E'), ('E', 'K'), ('D', 'C'),
            ('F', 'G'), ('G', 'K'), ('H', 'I'), ('I', 'M'),
            ('N', 'J'), ('N', 'J'),('J', 'F'), ('K', 'U'), ('K', 'O'), ('L', 'B'), ('L', 'H'), ('I', 'M'), ('M', 'Q'), ('Q', 'P'),
            ('O', 'N'), ('P', 'L'), ('P', 'L'),
            ('R', 'S'), ('S', 'T'), ('S', 'U'), ('T', 'L'), ('U', 'V'), ('V', 'D')
        ])

        # 3. 노드 위치 설정 (두 번째 이미지 기준 좌표)
        self.positions = {
            "A": (192, 170), "B": (621, 88), "C": (1130, 64), "D": (1500, 115), "E": (1150, 159), 
            "F": (282, 244), "G": (631, 216), "H": (1138, 274), "I": (1387, 328),
            "J": (245, 473), "K": (780, 502), "L": (904, 515), "M": (1452, 555),
            "N": (267, 694), "O": (598, 734), "P": (1114, 817), "Q": (1448, 770),
            "R": (210, 944), "S": (555, 984), "T": (575, 857), "U": (1156, 941), "V": (1512, 904)
        }

        self.marker_positions = {
            "A": (0.262, 0.161), "B": (0.241, 0.196), "C": (0.215, 0.229), "D": (0.17, 0.217), "E": (0.177, 0.192),
            "F": (0.205, 0.127), "G": (0.183, 0.146), "H": (0.139, 0.16), "I": (0.102, 0.148),
            "J": (0.127, 0.057), "K": (0.08, 0.069), "L": (0.1, 0.097), "M": (0.039, 0.094),
            "N": (0.063, 0.005), "O": (0.031, 0.002), "P": (-0.013, 0.015), "Q": (-0.022, 0.033),
            "R": (-0.015, -0.078), "S": (-0.036, -0.055), "T": (-0.005, -0.017), "U": (-0.055, -0.018), "V": (-0.045, 0.021)
        }

        self.explicit_directions = {
            # 좌측 루프
            ('A', 'R'): 'forward',
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
            ('O', 'N'): 'forward',
            ('N', 'J'): 'forward',
            ('J', 'F'): 'forward',
            ('F', 'G'): 'forward',
            ('G', 'K'): 'right',

            # 오른쪽 아랫부분
            ('P', 'L'): 'right',
            ('Q', 'P'): 'forward',
            ('H', 'I'): 'forward',
            ('I', 'M'): 'forward',
            ('M', 'Q'): 'forward',
            ('H', 'I'): 'forward',
            ('L', 'H'): 'right',

            # 오른쪽 진입 경로
            ('V', 'D'): 'forward',
            ('U', 'V'): 'forward',
            ('S', 'U'): 'forward',

            # 중앙
            ('D', 'C'): 'forward',
        }

        self.sp = SignalProcessor(window_size=5, alpha=0.3)
        self.behavior = {"forward": 0, "left": 1, "right": 2, 'stop': 3}
        self.path = None
        self.current_index = 0

        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
        self.subscriber = self.create_subscription(CommandInfo, 'drive', self.yolo_callback, qos_profile)
        self.cmd_vel_pub_pinky1 = self.create_publisher(Twist, '/taxi1/cmd_vel', qos_profile)
        self.cmd_vel_pub_pinky2 = self.create_publisher(Twist, '/taxi2/cmd_vel', qos_profile)

        self.vehicle_id = None
        self.offset = 0
        self.linear_x = 0
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
        self.arrived_cnt = 0

        self.get_logger().info("Drive Router Node Started")

    def heuristic(self, n1, n2):
        x1, y1 = self.positions[n1]
        x2, y2 = self.positions[n2]
        return np.hypot(x2 - x1, y2 - y1)

    def find_nearest_node(self, robot_pos):
        return min(self.marker_positions.items(), key=lambda x: np.linalg.norm(np.array(x[1]) - np.array(robot_pos)))[0]
    
    def set_goal_path(self, current_node, goal_node):
        if nx.has_path(self.G, current_node, goal_node):
            self.path = nx.shortest_path(self.G, current_node, goal_node)
            self.current_index = 0
            self.arrived = False
            self.get_logger().info(f"경로 설정됨: {self.path}")
        else:
            self.get_logger().error("경로 없음")

    def update_current_node(self, robot_pos):
        # path 내에서 가장 가까운 노드 탐색
        remaining_path = self.path[self.current_index:]
        nearest_node = min(
            remaining_path,
            key=lambda node: np.linalg.norm(np.array(self.marker_positions[node]) - np.array(robot_pos))
        )
        nearest_index = self.path.index(nearest_node)

        # 목표에 가까워지면 다음 노드로 이동
        goal_pos = self.marker_positions[self.path[-1]]
        if np.linalg.norm(np.array(robot_pos) - np.array(goal_pos)) < 0.03:
            self.arrived = True
            self.last_behavior = "stop"

            # 택시 도착 이벤트 전송
            if self.arrived_cnt > 1:
                self.manager.taxi_event_service(self.vehicle_id, 14, "destination")

            return None

        if nearest_index < len(self.path) - 1:
            self.current_index = nearest_index
            current_node = self.path[self.current_index]
            next_node = self.path[self.current_index + 1]
            direction = self.explicit_directions[(current_node, next_node)]
            self.last_behavior = self.behavior[direction]
            self.get_logger().info(f"[경로추적] {current_node} → {next_node}, 행동: {direction}")

        return self.last_behavior


    def pose_estimation(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = cv2.aruco.getPredefinedDictionary(self.aruco_dict_type)
        detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())
        corners, ids, _ = detector.detectMarkers(gray)

        if ids is not None and len(corners) > 0:
            for i in range(len(ids)):
                # if int(ids[i][0] != self.vehicle_id):
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], self.marker_length, self.k, self.d)
                pos = tvec[0][0]
                x, y = pos[0], pos[1]
                robot_pos = (round(self.sp.low_pass_filter(x), 3), round(self.sp.low_pass_filter(y), 3))
                self.manager.set_location(self.vehicle_id, robot_pos[0], robot_pos[1])

                if self.goal_node is None:
                    self.get_logger().info("대기중")
                    return   

                if self.goal_node not in self.marker_positions:
                    self.get_logger().error(f"유효하지 않은 goal_node: {self.goal_node}")
                    self.arrived = True
                    return self.behavior["stop"]

                # 최초 경로 설정
                if self.path is None or self.goal_node != self.path[-1]:
                    current_node = self.find_nearest_node(robot_pos)
                    self.set_goal_path(current_node, self.goal_node)

                return self.update_current_node(robot_pos)
        else:
            self.get_logger().warn("마커 인식 실패")
            return self.behavior["stop"]

        return self.last_behavior
    

    def timer_callback(self):     
        ret, frame = self.video.read()
        if not ret:
            return

        behavior = self.pose_estimation(frame)
        self.get_logger().info(f"behavior: {behavior}")

        pid_output = self.pid.compute(self.offset)
        pid_output = max(min(pid_output, 1.0), -1.0)

        twist = Twist()
        if self.arrived:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        else:
            if behavior == 0:
                twist.linear.x = 0.5
                twist.angular.z = -pid_output
            elif behavior == 1:
                twist.linear.x = 0.3
                twist.angular.z = max(min(0.8 - pid_output, 1.0), -1.0)
            elif behavior == 2:
                twist.linear.x = 0.3
                twist.angular.z = max(min(-0.8 - pid_output, 1.0), -1.0)
            elif behavior == 3:
                twist.linear.x = 0.0
                twist.angular.z = 0.0

        if self.vehicle_id == 1:
            self.cmd_vel_pub_pinky1.publish(twist)
        elif self.vehicle_id == 2:
            self.cmd_vel_pub_pinky2.publish(twist)

        self.get_logger().info(f"linear.x={twist.linear.x:.2f}, angular.z={twist.angular.z:.2f}")
        cv2.waitKey(1)

    def yolo_callback(self, msg):
        self.vehicle_id = msg.vehicle_id
        self.offset = msg.offset
        self.linear_x = msg.linear_x
        

    def destroy_node(self):
        self.video.release()
        cv2.destroyAllWindows()
        super().destroy_node()
    

    def send_command(self, vehicle_id, event_type):
        client = self.create_client(TaxiEvent, '/set_event_state')
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("운행 서비스 연결 실패")
            return

        req = TaxiEvent.Request()
        req.vehicle_id = vehicle_id
        req.event_type = event_type
        req.data = ""

        self.get_logger().info(f"운행 명령 비동기 전송: 택시 {vehicle_id}")

        future = client.call_async(req)

        def callback(fut):
            try:
                res = fut.result()
                if res.result:
                    self.get_logger().info(f"전송 성공")
                else:
                    self.get_logger().error(f"전송 실패 (응답은 옴)")
            except Exception as e:
                self.get_logger().error(f"전송 실패: {e}")

        future.add_done_callback(callback)


    def handle_message(self, vehicle_id, event_type, data):
        self.get_logger().info(f"📥 handle_message 수신된 메시지: {vehicle_id, event_type, data}")
        
        taxi = self.manager.get_taxi(vehicle_id)
        if not taxi:
            self.get_logger().warn(f"존재하지 않는 택시 ID: {vehicle_id}")
            return f"Taxi {vehicle_id} not found"

        if event_type == 13:
            self.vehicle_id = vehicle_id
            self.goal_node = data
            self.arrived = False
            return "ok"

        # 조건에 해당하지 않음
        self.get_logger().warn(f"처리되지 않은 이벤트: vehicle_id={vehicle_id}, event_type={event_type}")
        return "ignored"

def main(args=None):
    rclpy.init(args=args)
    node = DriveRouterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
