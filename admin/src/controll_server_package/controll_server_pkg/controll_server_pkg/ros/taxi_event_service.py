import rclpy
from rclpy.node import Node
from controll_server_package_msgs.srv import TaxiEvent
from controll_server_pkg.common.manager import ServiceManager
from controll_server_pkg.model.taxi import Taxi

class TaxiEventServiceNode(Node):
    def __init__(self, manager: ServiceManager):
        super().__init__('taxi_event_service_node')
        self.manager = manager

        self.srv = self.create_service(
            TaxiEvent,
            'TaxiEvent',
            self.handle_request
        )
        self.get_logger().info("🛠️ TaxiEvent 서비스 서버 실행 중")

    def handle_request(self, request, response):
        vehicle_id = request.vehicle_id
        event_type = request.event_type
        data = request.data

        log_event = {
            1: "승차",
            2: "하차",
            3: "대기",
            4: "운행",
            5: "충전",
            12: "RFID 태깅"
        }.get(event_type, f"알 수 없는 이벤트({event_type})")

        self.get_logger().info(f"📥 수신: 택시 ID={vehicle_id}, 이벤트={log_event}, 데이터={data}")

        taxi = self.manager.get_taxi(vehicle_id)
        if taxi:
            # 예: 상태 코드 업데이트
            taxi.update_state(event_type)

            if event_type == 12:
                self.get_logger().info(f"🆔 수신된 RFID UID: {data}")
                taxi.rfid_uid = data

            self.get_logger().info(f"✅ 택시 {vehicle_id} 상태 업데이트 완료")
            response.result = True
        else:
            self.get_logger().warn(f"🚫 존재하지 않는 택시 ID: {vehicle_id}")
            response.result = False

        return response


def main(args=None):
    rclpy.init(args=args)
    manager = ServiceManager()
    node = TaxiEventServiceNode(manager)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
