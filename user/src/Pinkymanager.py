import socket
import json
import threading
import time
from PyQt6.QtCore import QObject, pyqtSignal
from Icon_coordinates import CoordinateMapper
from UserSession import UserSession
import requests

class PinkyManager(QObject):
    _instance = None

    # --- PyQt 시그널 선언 ---
    position_updated = pyqtSignal(float, float)
    state_updated = pyqtSignal(str)
    battery_updated = pyqtSignal(float)
    start_reached = pyqtSignal()
    destination_reached = pyqtSignal()

    @classmethod
    def get_instance(cls, mapper=None):
        """싱글톤 인스턴스 반환, 최초 한 번만 mapper 필요"""
        if cls._instance is None:
            if mapper is None:
                raise ValueError("PinkyManager 싱글톤 최초 생성 시 mapper가 필요합니다.")
            cls._instance = cls(mapper)
        else:
            if mapper is not None:
                cls._instance.mapper = mapper  # 필요시 mapper 갱신
        return cls._instance

    def __init__(self, mapper):
        super().__init__()

        self.mapper = mapper
        self.vehicle_id = UserSession.get_taxi_id()
        if self.vehicle_id is None:
            raise ValueError("[ERROR] UserSession에 taxi_id가 설정되지 않았습니다.")
        
        self.socket = None

        self.pinky_x = 0
        self.pinky_y = 0
        self.battery = 100.0
        self.state = "unknown"
        self.start_location = [0.0, 0.0]
        self.destination_location = [0.0, 0.0]
        

        self.running = False
        self.host = "192.168.1.4"
        self.port = 9000

    def start(self):
        if self.running:
            print("[PinkyManager] 이미 실행 중")
            return
        self.running = True
        threading.Thread(target=self.connect_to_server, daemon=True).start()
        print("[PinkyManager] 수신 시작")

    def stop(self):
        self.running = False
        if self.socket:
            try:
                self.socket.shutdown(socket.SHUT_RDWR)
            except Exception:
                pass
            self.socket.close()
            self.socket = None
        print("[PinkyManager] 수신 중지")

    def connect_to_server(self):

        # 디버깅-----------------
        # while self.running:
        #     try:
        #         self.fallback_to_status_api()
        #     except Exception as e:
        #         print(f"[PinkyManager] fallback 호출 중 예외: {e}")
                
        #     time.sleep(2)
        # ----------------------

        while self.running:
            try:
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.connect((self.host, self.port))
                print("[PinkyManager] 서버에 연결됨")
                vehicle_info = {"vehicle_id": self.vehicle_id}
                self.socket.sendall(json.dumps(vehicle_info).encode())
                print(f"[PinkyManager] vehicle_id 전송 완료: {vehicle_info}")

                while self.running:
                    data = self.socket.recv(4096).decode().strip()
                    if not data:
                        print("[PinkyManager] 서버 연결 끊김")
                        break
                    try:
                        message = json.loads(data)
                        self.update_pinky(message)
                    except json.JSONDecodeError:
                        print(f"[PinkyManager] JSON 파싱 실패: {data}")

            except Exception as e:
                print(f"[PinkyManager] 예외 발생: {e}")
                time.sleep(3)

    # 디버깅용 fallback
    # def fallback_to_status_api(self):
    #     try:
    #         response = requests.get("http://192.168.1.3:8000/status_taxis", timeout=3)
    #         if response.status_code == 200:
    #             data = response.json()
    #             taxis = data.get("taxis", {})
    #             taxi_data = taxis.get(str(self.vehicle_id), None)
    #             if taxi_data:
    #                 print(f"[DEBUG] fallback 데이터 수신됨: {taxi_data}")
    #                 self.update_pinky(taxi_data)
    #             else:
    #                 print(f"[DEBUG] 해당 vehicle_id({self.vehicle_id})에 대한 데이터가 없습니다.")
    #         else:
    #             print(f"[DEBUG] fallback 요청 실패: {response.status_code}")
    #     except Exception as e:
    #         print(f"[DEBUG] fallback API 호출 중 오류: {e}")

    #     print("[DEBUG] 서버 연결 실패 - 테스트용 가짜 좌표 사용")
    #     dummy_data = {
    #         "location": [0.200,  0.1200],
    #         "start": [-0.015, -0.078],
    #         "destination": [0.08, 0.069],
    #         "battery": 85.0,
    #         "state": "dispatch"
    #     }
    #     self.update_pinky(dummy_data)

    def update_pinky(self, message):
        real_x, real_y = message.get("location", [0.0, 0.0])
        self.start_location = message.get("start", [0.0, 0.0])
        self.destination_location = message.get("destination", [0.0, 0.0])
        self.battery = message.get("battery", 100.0)
        self.state = message.get("state", "unknown")
        px, py = self.mapper.world_to_pixel(real_x, real_y)
        self.pinky_x, self.pinky_y = px, py
        self.position_updated.emit(self.pinky_x, self.pinky_y)
        self.state_updated.emit(self.state)
        self.battery_updated.emit(self.battery)
        self.check_reach_conditions()

    def check_reach_conditions(self, reach_threshold=5):
        if self.state == "boarding":
            self.start_reached.emit()
        if self.state == "landing":
            self.destination_reached.emit()

    def is_near(self, x, y, target, threshold=5):
        return abs(x - target[0]) < threshold and abs(y - target[1]) < threshold
