import rclpy
from rclpy.node import Node
from controll_server_package_msgs.msg import TaxiState
from controll_server_pkg.common.manager import ServiceManager


class AdminGuiTopicPublisher(Node):
    def __init__(self, manager: ServiceManager):
        super().__init__('admin_gui_topic_publisher')
        self.publisher_ = self.create_publisher(TaxiState, '/admin_gui_topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info("🚀 admin_gui_topic 퍼블리셔 시작됨")
        self.manager = manager
        self.manager.set_admin_gui_topic(self.handle_message)

    def timer_callback(self):
        # 두 대의 택시 상태를 각각 발행
        for vehicle_id, taxi in self.manager.taxis.items():
            msg = TaxiState()
            msg.vehicle_id = taxi.vehicle_id
            msg.state = taxi.state
            msg.location = list(taxi.location)  # tuple → list 변환
            msg.battery = taxi.battery
            msg.passenger_count = taxi.passenger_count
            msg.destination = taxi.destination
            msg.start = taxi.start

            self.publisher_.publish(msg)
            # self.get_logger().info(f"📤 TaxiState 발행: {msg}")

    def handle_message(self, message):
        print(f"[AdminGuiTopic] 수신된 메시지: {message}")
