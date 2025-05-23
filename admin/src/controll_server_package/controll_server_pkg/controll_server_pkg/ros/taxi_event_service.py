import rclpy
from rclpy.node import Node
from controll_server_package_msgs.srv import TaxiEvent
from controll_server_pkg.common.manager import ServiceManager
from controll_server_pkg.model.taxi import Taxi
from controll_server_pkg.common.database import Database
from time import sleep

class TaxiEventServiceNode(Node):
    """
    ROS2 서비스 노드로서 택시 이벤트를 처리하고, 내부 상태를 관리하며,
    필요 시 Raspberry Pi 또는 ESP32로 명령을 전송한다.
    """

    def __init__(self, manager: ServiceManager):
        super().__init__('taxi_event_service_node')
        self.manager = manager
        self.manager.set_taxi_event_service(self.handle_message)

        # taxi1 및 taxi2 서비스 등록
        self.srv1 = self.create_service(TaxiEvent, 'taxi1/TaxiEvent', self.receive_from_pi)
        self.srv2 = self.create_service(TaxiEvent, 'taxi2/TaxiEvent', self.receive_from_pi)
        self.get_logger().info("🛠️ TaxiEvent 서비스 서버 실행 중 (taxi1 & taxi2)")

    def receive_from_pi(self, request, response):
        """
        Raspberry Pi 등 외부 장치에서 들어온 서비스 요청을 처리한다.
        예: RFID 스캔, 승차/하차 상태 업데이트 등
        """
        db = Database()
        vehicle_id = request.vehicle_id
        event_type = request.event_type
        data = request.data

        log_event = {
            1: "승차", 2: "하차", 3: "대기", 4: "운행", 5: "충전",
            6: "좌측 지시등", 7: "우측 지시등", 8: "비상 지시등",
            9: "지시등 해제", 10: "문 열림", 11: "문 닫힘", 12: "RFID 태그",
            13: "출발", 14: "도착"
        }.get(event_type, f"기타 이벤트({event_type})")

        self.get_logger().info(f"📥 수신: 택시 ID={vehicle_id}, 이벤트={log_event}, 데이터={data}")

        taxi = self.manager.get_taxi(vehicle_id)
        if taxi:
            if event_type == 12 and taxi.state == "boarding":
                self.get_logger().info(f"🆔 수신된 RFID UID: {data}")
                passenger_info = db.execute_select(
                    "SELECT * FROM Passengers WHERE rfid = %s and passenger_id = %s",
                    (data, taxi.passenger_id)
                )
                if passenger_info:
                    self.get_logger().info(f"🔓 유효한 RFID. 문 열림 명령 전송")
                    self.send_to_pi(vehicle_id, 10)
                else:
                    self.get_logger().warn(f"🚫 존재하지 않는 RFID UID: {data}")
            elif event_type in [1, 2]:
                taxi.passenger_state = log_event

            self.get_logger().info(f"✅ 택시 {vehicle_id} 상태 업데이트 완료")
            response.result = True
        else:
            self.get_logger().warn(f"🚫 존재하지 않는 택시 ID: {vehicle_id}")
            response.result = False

        return response

    def send_to_pi(self, vehicle_id, event_type):
        """
        택시(vehicle_id)에 지정된 이벤트 명령을 Raspberry Pi(또는 ESP32)로 전송한다.
        """
        service_name = f"/taxi{vehicle_id}/set_event_state"
        client = self.create_client(TaxiEvent, service_name)

        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(f"❗ 서비스 연결 실패: {service_name}")
            return

        req = TaxiEvent.Request()
        req.vehicle_id = vehicle_id
        req.event_type = event_type
        req.data = ""

        self.get_logger().info(f"📤 명령 비동기 전송: {service_name} (이벤트: {event_type})")
        future = client.call_async(req)

        def callback(fut):
            try:
                res = fut.result()
                if res.result:
                    self.get_logger().info(f"✅ 택시 {vehicle_id} 성공")
                else:
                    self.get_logger().error(f"❌ 택시 {vehicle_id} 실패 (응답은 옴)")
            except Exception as e:
                self.get_logger().error(f"❌ 응답 처리 실패: {e}")

        future.add_done_callback(callback)

    def handle_message(self, vehicle_id, event_type, data):
        """
        내부 이벤트 (예: 서버 흐름에서 호출)를 처리하는 콜백 함수.
        """
        self.get_logger().info(f"📥 handle_message 수신: {vehicle_id}, {event_type}, {data}")
        taxi = self.manager.get_taxi(vehicle_id)
        if not taxi:
            self.get_logger().warn(f"🚫 존재하지 않는 택시 ID: {vehicle_id}")
            return f"Taxi {vehicle_id} not found"

        if event_type == 11 and taxi.state == "boarding":
            if taxi.passenger_state == "승차":
                self.send_to_pi(vehicle_id, event_type)

                # 반복 재시도
                max_retries = 5
                retry_interval = 1.0  # 초
                for attempt in range(max_retries):
                    result = self.manager.drive_router_node(vehicle_id, 13, taxi.destination_node)
                    if result == "ok":
                        self.send_to_pi(vehicle_id, 9)
                        taxi.state = "driving"
                        return "ok"
                    
                    self.get_logger().warn(f"🔁 주행 명령 재시도 {attempt + 1}/{max_retries} (결과: {result})")
                    time.sleep(retry_interval)
                
                self.get_logger().error("❌ 주행 명령 재시도 실패 (최대 횟수 초과)")
                return "error"

            else:
                self.get_logger().warn(f"🚫 탑승 상태가 아님: {taxi.passenger_state}")
                return f"승객 상태={taxi.passenger_state} (expected '승차')"

        elif event_type == 11 and taxi.state == "landing":
            if taxi.passenger_state == "하차":
                self.send_to_pi(vehicle_id, event_type)
                result = self.manager.drive_router_node(vehicle_id, 13, taxi.start_node)
                if result == "ok":

                    if taxi.battery < 60:
                        taxi.state = "charging"
                    else:
                        "ready"
                    self.send_to_pi(vehicle_id, 9)

                return "ok"
            else:
                self.get_logger().warn(f"🚫 하차 상태가 아님: {taxi.passenger_state}")
                return f"승객 상태={taxi.passenger_state} (expected '하차')"

        elif event_type == 14 and data == "destination" and taxi.state == "dispatch":
            taxi.state = "boarding"
            self.send_to_pi(vehicle_id, 8)
            return "ok"

        elif event_type == 14 and data == "destination" and taxi.state == "driving":
            taxi.state = "landing"
            self.send_to_pi(vehicle_id, 8)
            self.send_to_pi(vehicle_id, 10)
            return "ok"

        self.get_logger().warn(f"⚠️ 처리되지 않은 이벤트: {vehicle_id}, {event_type}, state={taxi.state}")
        return "ignored"


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
