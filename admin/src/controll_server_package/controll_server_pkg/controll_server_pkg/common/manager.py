from controll_server_pkg.model.taxi import Taxi

class ServiceManager:
    def __init__(self):
        # 🔗 모듈 핸들러들
        self.api = None
        self.socket = None
        self.ros_admin_service = None
        self.ros_admin_topic = None
        self.ros_drive = None

        # 🚕 택시 객체 2대 초기화
        self.taxis = {
            1: Taxi(1,4),
            2: Taxi(2,6),
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

    # 🧠 택시 객체 참조 메서드
    def get_taxi(self, vehicle_id):
        return self.taxis.get(vehicle_id)

    # 🔁 모든 모듈에 메시지 브로드캐스트
    def broadcast(self, message):
        print(f"[Broadcast] {message}")
        for target in [self.api, self.socket, self.ros_admin_service, self.ros_admin_topic, self.ros_drive]:
            if target:
                target.handle_message(message)
