import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from controll_server_pkg.common.manager import ServiceManager
from controll_server_pkg.model.taxi import Taxi

class TaxiStateNode(Node):
    def __init__(self, manager: ServiceManager):
        super().__init__('taxi_state_node')
        self.manager = manager

        # 🚕 두 대 택시에 대해 구독 생성
        self.create_subscription(Float32, '/pinky_battery_present', lambda msg: self.update_battery(2, msg), 10)
        # self.create_subscription(Float32, '/pinky2_battery', lambda msg: self.update_battery(2, msg), 10)

        self.get_logger().info("📡 Taxi 상태 구독 노드 시작됨 (2대 대응)")

    def update_battery(self, vehicle_id: int, msg):
        battery_percent = msg.data
        taxi = self.manager.get_taxi(vehicle_id)
        if taxi:
            taxi.update_battery(battery_percent)
            self.get_logger().info(f"🔋 택시 {vehicle_id} 배터리 업데이트됨: {battery_percent:.1f}%")
        else:
            self.get_logger().warn(f"🚫 존재하지 않는 택시 ID: {vehicle_id}")

# ✅ 테스트용 실행
def main(args=None):
    rclpy.init(args=args)
    manager = ServiceManager()
    node = TaxiStateNode(manager)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
