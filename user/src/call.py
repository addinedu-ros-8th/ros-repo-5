import os
from PyQt6.QtWidgets import QMainWindow, QApplication, QLabel
from PyQt6.QtCore import Qt, QTimer, QPointF
from PyQt6.QtGui import *
from PyQt6 import uic
from ui_utils import *
from riding import *

class CallWindow(QMainWindow):
    def __init__(self, start_icon_name, destination_icon_name):
        super().__init__()
        uic.loadUi(self.get_ui_path("/home/lim/dev_ws/addintexi/UserGUI/ui/1_call.ui"), self)
        self.Pinky.setVisible(False)
        
        # Pinky 이미지 초기 설정
        self.pinky_image = QLabel(self)
        self.pinky_image.setPixmap(QPixmap("/mnt/data/transparent_pinky.png"))
        self.pinky_image.setGeometry(0, 0, 50, 50)  # 초기 크기와 위치 설정
        self.pinky_image.setScaledContents(True)
        self.pinky_image.setVisible(True)

        # Pinky의 초기 위치
        self.pinky_x = 0
        self.pinky_y = 0
        self.destination_x = 300  # 예제 목적지 x 좌표 (실제로는 서버에서 받아온 좌표)
        self.destination_y = 200  # 예제 목적지 y 좌표
        
        # 실시간 위치 업데이트 타이머 (500ms 간격)
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_pinky_position)
        self.timer.start(500)

        # 아이콘 핸들러 설정
        self.icon_handler = IconHandler(self, {
            "Icon1": "Info1",
            "Icon2": "Info2",
            "Icon3": "Info3",
            "Icon4": "Info4",
            "Icon5": "Info5",
            "Icon6": "Info6"
        })

        # LineEditHandler 설정
        self.line_edit_handler = LineEditHandler(self)
        self.location_names = LocationManager.get_location_names()
        self.start_location = self.start  # QLineEdit (출발지)
        self.destination_location = self.destination  # QLineEdit (목적지)


        # 출발지/목적지 아이콘 유지
        self.start_location = self.start  # QLineEdit (출발지)
        self.destination_location = self.destination
        self.icon_handler.set_fixed_icons(start_icon_name, destination_icon_name)
        print(f"[DEBUG] 출발지: {start_icon_name}, 목적지: {destination_icon_name}")

        # 전달된 출발지/목적지 이름 설정
        self.set_location_text(start_icon_name, destination_icon_name)

    #----------------------------------------------------------------
    # 디버깅용 
    # ----------------------------------------------------------------
        
        # 디버그 버튼 설정
        self.debug_button = QPushButton("Debug", self)
        self.debug_button.setGeometry(300, 500, 80, 40)
        self.debug_button.clicked.connect(self.debug_button_clicked)

        # 디버그 클릭 횟수
        self.debug_click_count = 0
    
    
    def debug_button_clicked(self):
        self.debug_click_count += 1
        print(f"[DEBUG] Debug 버튼 클릭: {self.debug_click_count}")

        if self.debug_click_count == 2:
            print("[DEBUG] Debug 2회 클릭 - Riding 페이지로 이동")
                        
            # call.py 페이지로 이동 -> 디버깅용 코드
            from riding import RidingWindow
            self.riding_window = RidingWindow(
                start_icon=self.start_icon,
                destination_icon=self.destination_icon
            )
            self.riding_window.show()
            self.close()
          

    def transition_to_riding_page(self):
        from riding import RidingWindow
        self.riding_window = RidingWindow()
        self.riding_window.show()
        self.close()

    #---------------------------------------------------------------



    def get_ui_path(self, ui_file):
  
        base_dir = os.path.dirname(os.path.abspath(__file__))
        return os.path.join(base_dir, "ui", ui_file)
    


    def set_location_text(self, start_icon_name, destination_icon_name):
        location_names = LocationManager.get_location_names()

        # 출발지 LineEdit 업데이트
        start_name = self.location_names.get(start_icon_name, "출발지 선택")
        self.line_edit_handler.update_line_edit("start", start_name)
        print(f"[DEBUG] 출발지 설정: {start_name}")

        # 목적지 LineEdit 업데이트
        destination_name = self.location_names.get(destination_icon_name, "목적지 선택")
        self.line_edit_handler.update_line_edit("destination", destination_name)
        print(f"[DEBUG] 목적지 설정: {destination_name}")

    def update_pinky_position(self):
        """
        서버에서 Pinky의 위치 좌표 받아와 이미지 이동
        """
        # 서버에서 Pinky 위치 수신 (임시 예제 좌표, 실제로는 서버에서 받아옴)
        response = self.get_pinky_position_from_server()
        if response:
            self.pinky_x, self.pinky_y = response
            print(f"[DEBUG] Pinky 위치: ({self.pinky_x}, {self.pinky_y})")

            # Pinky 이미지 위치 업데이트
            self.pinky_image.move(self.pinky_x, self.pinky_y)

            # 도착 확인
            if self.is_pinky_at_destination():
                self.timer.stop()
                self.transition_to_next_page()

    def get_pinky_position_from_server(self):
        """
        서버에서 Pinky 좌표 받아오기 (임시 예제)
        """
        # 서버에서 좌표 받아오는 코드 (임시, 실제 서버 URL 사용)
        # response = requests.get("http://localhost:8000/get_pinky_position")
        # if response.status_code == 200:
        #     return response.json()["x"], response.json()["y"]
        # return None

        # 임시로 좌표를 시뮬레이션 (테스트용)
        self.pinky_x += 10
        self.pinky_y += 5
        return self.pinky_x, self.pinky_y

    def is_pinky_at_destination(self):
        """
        Pinky가 목적지에 도착했는지 확인 (오차 범위: 10px)
        """
        distance_x = abs(self.pinky_x - self.destination_x)
        distance_y = abs(self.pinky_y - self.destination_y)

        return distance_x <= 10 and distance_y <= 10

    def transition_to_next_page(self):
        """
        목적지 도착 시 다음 페이지로 자동 전환
        """
        print("[INFO] Pinky가 목적지에 도착했습니다. 승차 페이지로 이동합니다.")
        from riding import RidingWindow  # 승차 페이지 (예제)
        self.riding_window = RidingWindow()
        self.riding_window.show()