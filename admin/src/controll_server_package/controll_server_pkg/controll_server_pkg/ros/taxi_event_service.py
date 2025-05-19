import rclpy
from rclpy.node import Node
from controll_server_package_msgs.srv import TaxiEvent
from controll_server_pkg.common.manager import ServiceManager
from controll_server_pkg.model.taxi import Taxi
from controll_server_pkg.common.database import Database


class TaxiEventServiceNode(Node):
    """
    ROS2 서비스 노드로서 택시 이벤트를 처리하고, 내부 상태를 관리하며,
    필요 시 Raspberry Pi 또는 ESP32로 명령을 전송한다.
    """

    def __init__(self, manager: ServiceManager):
        super().__init__('taxi_event_service_node')

        # 택시 상태 및 객체를 관리하는 매니저 인스턴스
        self.manager = manager

        # 내부 콜백 등록 (다른 모듈에서 이벤트 발생 시 호출될 함수 설정)
        self.manager.set_taxi_event_service(self.handle_message)

        # 외부에서 들어오는 서비스 요청 처리 (주로 Raspberry Pi -> 서버)
        self.srv = self.create_service(
            TaxiEvent,
            'TaxiEvent',
            self.receive_from_pi
        )
        self.get_logger().info("🛠️ TaxiEvent 서비스 서버 실행 중")

    def receive_from_pi(self, request, response):
        """
        Raspberry Pi 등 외부 장치에서 들어온 서비스 요청을 처리한다.
        예: RFID 스캔, 승차/하차 상태 업데이트 등
        """
        db = Database()

        vehicle_id = request.vehicle_id
        event_type = request.event_type
        data = request.data

        # 이벤트 종류 출력용 로그 변환
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

        # 해당 vehicle_id의 택시 인스턴스를 매니저에서 조회
        taxi = self.manager.get_taxi(vehicle_id)
        if taxi:
            # RFID 이벤트: 배차 상태일 때만 처리
            if event_type == 12 and taxi.state == "dispatch":
                self.get_logger().info(f"🆔 수신된 RFID UID: {data}")

                # RFID 검증: 해당 RFID와 승객 ID가 일치하는지 DB에서 조회
                passenger_info = db.execute_select(
                    "SELECT * FROM Passengers WHERE rfid = %s and passenger_id = %s",
                    (data, taxi.passenger_id)
                )

                if passenger_info:
                    self.get_logger().info(f"🔓 유효한 RFID. 문 열림 명령 전송")
                    self.send_to_pi(vehicle_id, 10)  # 문 열기 명령 전송
                else:
                    self.get_logger().warn(f"🚫 존재하지 않는 RFID UID: {data}")

            # 승차 / 하차 이벤트 처리
            elif event_type == 1 or event_type == 2:
                taxi.passenger_state = log_event  # 상태값 업데이트

            self.get_logger().info(f"✅ 택시 {vehicle_id} 상태 업데이트 완료")
            response.result = True
        else:
            self.get_logger().warn(f"🚫 존재하지 않는 택시 ID: {vehicle_id}")
            response.result = False

        return response

    def send_to_pi(self, vehicle_id, event_type):
        """
        택시(vehicle_id)에 지정된 이벤트 명령을 Raspberry Pi(또는 ESP32)로 전송한다.
        비동기 방식으로 전송되며, 응답을 수신하면 로그 출력.
        """
        client = self.create_client(TaxiEvent, '/set_event_state')

        # 서비스가 준비될 때까지 대기
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("❗ 서비스 연결 실패")
            return

        # 요청 객체 구성
        req = TaxiEvent.Request()
        req.vehicle_id = vehicle_id
        req.event_type = event_type
        req.data = ""

        self.get_logger().info(f"📤 명령 비동기 전송: 택시 {vehicle_id} (이벤트: {event_type})")

        # 비동기 호출
        future = client.call_async(req)

        # 응답 핸들러 등록
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
        내부 이벤트 (예: 서버 처리 흐름에서의 호출)를 처리하는 콜백 함수.
        조건에 따라 택시 상태를 업데이트하고 명령을 전송한다.
        """
        self.get_logger().info(f"📥 handle_message 수신된 메시지: {vehicle_id, event_type, data}")

        taxi = self.manager.get_taxi(vehicle_id)
        if not taxi:
            self.get_logger().warn(f"🚫 존재하지 않는 택시 ID: {vehicle_id}")
            return f"Taxi {vehicle_id} not found"

        # 🟡 문 닫힘 처리 (승차 상태 확인)
        if event_type == 11 and taxi.state == "boarding":
            if taxi.passenger_state == "승차":
                self.send_to_pi(vehicle_id, event_type)
                reasult = self.manager.drive_router_node(vehicle_id, 13, taxi.start_node)
                if reasult == "ok":
                    taxi.state = "drive_start"
                    self.send_to_pi(vehicle_id, 9)
                return "ok"
            else:
                self.get_logger().warn(f"🚫 택시 {vehicle_id}의 승객 상태가 '승차'가 아님: {taxi.passenger_state}")
                return f"taxi.passenger_state={taxi.passenger_state} (expected '승차')"

        # 🔵 출발지 도착 → 운행 시작 준비
        elif event_type == 14 and taxi.state == "drive_start":
            taxi.state = "boarding"
            self.send_to_pi(vehicle_id, 8)  # 비상등 켜기
            return "ok"

        # 🔵 목적지 도착 → 하차 준비
        elif event_type == 14 and taxi.state == "drive_destination":
            taxi.state = "landing"
            self.send_to_pi(vehicle_id, 8)   # 비상등 켜기
            self.send_to_pi(vehicle_id, 10)  # 문 열기
            return "ok"

        # ⚠️ 특별 처리 대상 아님
        self.get_logger().warn(f"⚠️ 처리되지 않은 이벤트: vehicle_id={vehicle_id}, event_type={event_type}, state={taxi.state}")
        return "ignored"


def main(args=None):
    rclpy.init(args=args)

    # 택시 매니저 생성 및 노드 실행
    manager = ServiceManager()
    node = TaxiEventServiceNode(manager)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
