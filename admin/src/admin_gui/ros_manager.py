import threading
import rclpy
from rclpy.node import Node
from controll_server_package_msgs.msg import TaxiState

_ros_node = None

class RosSubscriberNode(Node):
    def __init__(self, callback):
        super().__init__('admin_gui_node')
        self._user_callback = callback
        self.subscription = self.create_subscription(
            TaxiState,
            '/admin_gui_topic',
            self._internal_callback,
            10
        )

    def _internal_callback(self, msg):
        print(f"[ROS DEBUG] 수신됨: vehicle_id={msg.vehicle_id}, state={msg.state}")
        if self._user_callback is not None:
            try:
                self._user_callback(msg)
            except Exception as e:
                print(f"[ERROR] 사용자 콜백 처리 중 오류: {e}")

    def update_callback(self, callback):
        print("[ROS INFO] 콜백 함수 업데이트됨.")
        self._user_callback = callback


def is_ros_initialized():
    return _ros_node is not None


def init_ros(callback):
    """
    ROS를 초기화하고 콜백을 설정한 뒤 백그라운드에서 spin을 실행한다.
    반드시 첫 호출에서 callback을 넘겨줄 것!
    """
    global _ros_node
    if not rclpy.ok():
        rclpy.init()

    if _ros_node is None:
        print("[ROS INFO] ROS 노드 초기화 및 spin 시작")
        _ros_node = RosSubscriberNode(callback)
        thread = threading.Thread(target=rclpy.spin, args=(_ros_node,), daemon=True)
        thread.start()
    else:
        print("[ROS WARNING] 이미 ROS가 초기화되어 있습니다. 콜백만 갱신합니다.")
        _ros_node.update_callback(callback)


def update_callback(callback):
    """
    이미 초기화된 ROS 노드가 있다면 콜백을 갱신한다.
    """
    global _ros_node
    if _ros_node is not None:
        _ros_node.update_callback(callback)
    else:
        print("[ROS WARNING] ROS 노드가 초기화되지 않았습니다. init_ros()를 먼저 호출하세요.")
