from controll_server_pkg.model.taxi import Taxi

class ServiceManager:
    def __init__(self):
        # 🔗 모듈 핸들러들
        self.api = None
        self.socket = None
        self.ros_admin_service = None
        self.ros_admin_topic = None
        self.ros_drive = None
        self.taxi_event_service = None
        self.taxi_state_node = None
        self.drive_router_node = None
        self.admin_gui_topic = None

        # 🚕 택시 객체 2대 초기화
        self.taxis = {
            1: Taxi(1,4,'Y'),
            2: Taxi(2,6,'Z'),
        }

    # 🔗 모듈 등록 메서드
    def set_api(self, handler):
        self.api = handler

    def set_socket(self, handler):
        self.socket = handler

    def set_ros_admin_service(self, handler):
        self.ros_admin_service = handler

    def set_ros_admin_topic(self, handler):
        self.ros_admin_topic = handler

    def set_ros_drive(self, handler):
        self.ros_drive = handler

    def set_taxi_event_service(self, handler):
        self.taxi_event_service = handler

    def set_taxi_state_node(self, handler):
        self.taxi_state_node = handler

    def set_drive_router_node(self, handler):
        self.drive_router_node = handler

    def set_admin_gui_topic(self, handler):
        self.admin_gui_topic = handler

    def set_location(self, vehicle_id, x, y):
        taxi = self.taxis.get(vehicle_id)
        if taxi:
            taxi.update_location(x, y)
            print(f"🚖 택시 {vehicle_id} 위치 업데이트됨: ({x}, {y})")
        else:
            print(f"🚫 존재하지 않는 택시 ID: {vehicle_id}")

    # 🧠 택시 객체 참조 메서드
    def get_taxi(self, vehicle_id):
        return self.taxis.get(vehicle_id)

    def get_location(self, vehicle_id):
        taxi = self.taxis.get(vehicle_id)
        if taxi:
            return taxi.location
        else:
            print(f"🚫 존재하지 않는 택시 ID: {vehicle_id}")
            return (0.0, 0.0)

    # 🔁 모든 모듈에 메시지 브로드캐스트
    def broadcast(self, message):
        print(f"[Broadcast] {message}")
        for target in [self.api, self.socket, self.ros_admin_service, self.ros_admin_topic, self.ros_drive]:
            if target:
                target.handle_message(message)
