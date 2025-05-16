import rclpy
from rclpy.node import Node
from controll_server_package_msgs.srv import TaxiEvent
from controll_server_pkg.common.manager import ServiceManager
from controll_server_pkg.model.taxi import Taxi
from controll_server_pkg.common.database import Database

class TaxiEventServiceNode(Node):
    def __init__(self, manager: ServiceManager):
        super().__init__('taxi_event_service_node')
        self.manager = manager
        
        self.manager.set_taxi_event_service(self.handle_message)

        self.srv = self.create_service(
            TaxiEvent,
            'TaxiEvent',
            self.handle_request
        )
        self.get_logger().info("🛠️ TaxiEvent 서비스 서버 실행 중")

    def handle_request(self, request, response):
        
        db = Database()

        vehicle_id = request.vehicle_id
        event_type = request.event_type
        data = request.data

        log_event = {
            1: "승차",
            2: "하차",
            3: "대기",
            4: "운행",
            5: "충전",
            6: "좌측 지시등",
            7: "우측 지시등",
            8: "비상 지시등",
            9: "지시등 해제",
            10: "문 열림",
            11: "문 닫힘",
            12: "RFID 태그",
            13: "출발",
            14: "도착"
        }.get(event_type, f"기타 이벤트({event_type})")

        self.get_logger().info(f"📥 수신: 택시 ID={vehicle_id}, 이벤트={log_event}, 데이터={data}")

        taxi = self.manager.get_taxi(vehicle_id)
        if taxi:
            if event_type == 12 and taxi.state == "dispatch":
                self.get_logger().info(f"🆔 수신된 RFID UID: {data}")

                # RFID 검증: DB 조회
                passenger_info = db.execute_select(
                    "SELECT * FROM Passengers WHERE rfid = %s and passenger_id = %s",
                    (data, taxi.passenger_id)
                )

                if passenger_info:
                    self.get_logger().info(f"🔓 유효한 RFID. 문 열림 명령 전송")
                    self.send_door_command(vehicle_id, 10)
                else:
                    self.get_logger().warn(f"🚫 존재하지 않는 RFID UID: {data}")

            
            elif event_type == 1 or event_type == 2:
                # 승차/하차 이벤트 처리
                taxi.passenger_state = log_event

            self.get_logger().info(f"✅ 택시 {vehicle_id} 상태 업데이트 완료")            

            response.result = True
        else:
            self.get_logger().warn(f"🚫 존재하지 않는 택시 ID: {vehicle_id}")
            response.result = False

        return response


    def send_command(self, vehicle_id, event_type):
        client = self.create_client(TaxiEvent, '/set_event_state')
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("❗ 문 열림 서비스 연결 실패")
            return

        req = TaxiEvent.Request()
        req.vehicle_id = vehicle_id
        req.event_type = event_type
        req.data = ""

        self.get_logger().info(f"📤 문 열림 명령 비동기 전송: 택시 {vehicle_id}")

        future = client.call_async(req)

        def callback(fut):
            try:
                res = fut.result()
                if res.result:
                    self.get_logger().info(f"✅ 택시 {vehicle_id} 문 열림 성공")
                else:
                    self.get_logger().error(f"❌ 택시 {vehicle_id} 문 열림 실패 (응답은 옴)")
            except Exception as e:
                self.get_logger().error(f"❌ 문 열림 응답 처리 실패: {e}")

        future.add_done_callback(callback)

    def handle_message(self, vehicle_id, event_type, data):
        self.get_logger().info(f"📥 handle_message 수신된 메시지: {vehicle_id, event_type, data}")
        
        taxi = self.manager.get_taxi(vehicle_id)
        if not taxi:
            self.get_logger().warn(f"🚫 존재하지 않는 택시 ID: {vehicle_id}")
            return f"Taxi {vehicle_id} not found"

        # 🟡 문 닫힘 이벤트 (event_type == 11)
        if event_type == 11 and taxi.state == "boarding":
            if taxi.passenger_state == "승차":
                self.send_command(vehicle_id, event_type)                
                return "ok"
            else:
                self.get_logger().warn(f"🚫 택시 {vehicle_id}의 승객 상태가 '승차'가 아님: {taxi.passenger_state}")
                return f"taxi.passenger_state={taxi.passenger_state} (expected '승차')"        

        # 🔵 운행 시작 이벤트 (event_type == 14)
        elif event_type == 14 and taxi.state == "drive_start":
            taxi.state = "boarding"
            self.send_command(vehicle_id, 8)  # 비상등 켜기 등
            return "ok"
        
        elif event_type == 14 and taxi.state == "drive_destination":
            taxi.state = "landing"
            self.send_command(vehicle_id, 8)
            self.send_command(vehicle_id, 10)
            return "ok"

        # ⚠️ 조건에 해당하지 않음
        self.get_logger().warn(f"⚠️ 처리되지 않은 이벤트: vehicle_id={vehicle_id}, event_type={event_type}, state={taxi.state}")
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
