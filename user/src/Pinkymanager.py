import requests
from PyQt6.QtCore import QObject, QTimer, pyqtSignal
from Restapi import RestAPIManager
from UserSession import UserSession

class PinkyManager(QObject):
    position_updated = pyqtSignal(float, float)  # 위치 갱신 신호
    start_reached = pyqtSignal()                 # 호출지 도달 신호
    destination_reached = pyqtSignal()           # 목적지 도달 신호

    def __init__(self):
        super().__init__()
        self.pinky_x = 0
        self.pinky_y = 0
        self.battery = 100.0
        self.start_destination = None
        self.final_destination = None

        self.timer = QTimer()
        self.timer.timeout.connect(self.fetch_pinky_position_from_server)
        self.timer_interval = 500  # 0.5초 (500ms)마다 위치 갱신
        self.api_manager = RestAPIManager()
        self.taxi_id = UserSession.get_taxi_id()

        self.map_width = 521
        self.map_height = 351

    def set_destinations(self, start_coords, final_coords):
        """
        출발지와 목적지 설정 (아르코마커 좌표 -> 맵 좌표로 변환)
        """
        self.start_destination = self.convert_aruco_to_map_coords(start_coords)
        self.final_destination = self.convert_aruco_to_map_coords(final_coords)
        print(f"[PinkyManager] Start: {self.start_destination}, Final: {self.final_destination}")

    def convert_aruco_to_map_coords(self, aruco_coords):
        """
        아르코마커 좌표 -> 맵 이미지 좌표로 변환
        """
        aruco_x, aruco_y = aruco_coords
        map_x = self.map_width * (aruco_x + 1) / 2
        map_y = self.map_height * (1 - (aruco_y + 1) / 2)
        return map_x, map_y
    
    def map_coords(self, x, y):
        # 1000 x 1000 범위 -> Map 크기 변환
        mapped_x = (x / 1000) * self.map_width
        mapped_y = (y / 1000) * self.map_height
        
        # Map 범위를 벗어나지 않도록 제한
        mapped_x = max(0, min(mapped_x, self.map_width - 50))  # 50은 Pinky 이미지 크기
        mapped_y = max(0, min(mapped_y, self.map_height - 50))  # 50은 Pinky 이미지 크기
        return mapped_x, mapped_y
    
    def start(self):
        if self.taxi_id:
            self.timer.start(self.timer_interval)
        else:
            print("[ERROR] Taxi ID가 설정되지 않았습니다.")

    def stop(self):
        self.timer.stop()

    def fetch_pinky_position_from_server(self):
        """
        서버에서 지정된 taxi_id의 위치 정보만 가져옴.
        """
        if not self.taxi_id:
            print("[ERROR] Taxi ID가 설정되지 않았습니다.")
            return

        response = self.api_manager.send_get_request(f"/status_taxis?vehicle_id={self.taxi_id}")
        if response:
            x, y = response.get("x", 0), response.get("y", 0)
            map_x, map_y = self.map_coords(x, y)
            self.position_updated.emit(map_x, map_y)
            print(f"[PinkyManager] 위치 갱신 - Map 좌표: X: {int(map_x)}, Y: {int(map_y)}")

            # 출발지 및 목적지 도달 확인
            if self.start_destination and self.is_near(map_x, map_y, self.start_destination):
                self.start_reached.emit()
            if self.final_destination and self.is_near(map_x, map_y, self.final_destination):
                self.destination_reached.emit()
        else:
            print("[PinkyManager] 서버에서 지정된 택시 정보를 찾을 수 없습니다.")

    def is_near(self, x, y, target):
        return abs(x - target[0]) < 5 and abs(y - target[1]) < 5
