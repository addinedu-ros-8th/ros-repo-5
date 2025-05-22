import os
from PyQt6.QtWidgets import QMainWindow, QLabel, QPushButton, QApplication
from PyQt6.QtCore import Qt, QTimer, QPointF
from PyQt6.QtGui import *
from PyQt6 import uic
from charge import *
from Icon_coordinates import ICON_COORDINATES
from IconHandler import * 
from Restapi import * 
from Pinkymanager import * 
from LeftMoney import * 
import sys
from PyQt6.QtCore import QLoggingCategory


# stderr (경고 메시지) 출력 차단
sys.stderr = open(os.devnull, 'w')

# 모든 Qt 경고 메시지 차단
QLoggingCategory.setFilterRules("*.debug=false\n*.warning=false\n*.critical=false\n*.fatal=false")

class EndWindow(QMainWindow):
    def __init__(self, start_icon_name, destination_icon_name,mapper):
        super().__init__()
        uic.loadUi(self.get_ui_path("/home/lim/dev_ws/addintexi/UserGUI/ui/4_end.ui"), self)
        self.ChargeBtn.clicked.connect(self.show_charge_page)
        self.left_money_manager = LeftMoneyManager(self.LeftMoney) 
        self.CheckBtn.clicked.connect(self.switch_to_main_window)

        # REST API 클라이언트 생성
        self.api_manager = RestAPIManager()
        self.start_location = None
        self.end_location = None

        # IconHandler로 아이콘 고정 (출발지/목적지)
        self.icon_handler = IconHandler(self, {
            "Icon1": "Info1",
            "Icon2": "Info2",
            "Icon3": "Info3",
            "Icon4": "Info4",
            "Icon5": "Info5",
            "Icon6": "Info6"
        })
        self.icon_handler.set_fixed_icons(start_icon_name, destination_icon_name)

        # 출발지/목적지 상태 저장
        self.start_icon = start_icon_name
        self.destination_icon = destination_icon_name

        # 마우스 클릭 차단 (아이콘)
        self.icon_handler.lock_icons()

        self.mapper = mapper
        self.pinky = PinkyManager(mapper=self.mapper) 
        self.pinky.position_updated.connect(self.update_pinky_position)
        self.setup_pinky_image()



        # PinkyManager 설정
    def setup_pinky_image(self):
        # Map 위젯 위에 Pinky 설정
        self.pinky_image = QLabel(self.Map)  # Map의 자식으로 Pinky 설정
        pinky_pixmap = QPixmap("/home/lim/dev_ws/addintexi/UserGUI/data/map_icon/pinky.png")
        
        # QPixmap 투명 배경 유지 (Alpha 채널 유지)
        self.pinky_image.setPixmap(pinky_pixmap)
        self.pinky_image.setGeometry(0, 0, 80, 80)  # 초기 크기와 위치 설정
        self.pinky_image.setScaledContents(True)
        self.pinky_image.setStyleSheet("background: transparent;")  # 투명 배경 유지
        
        # Pinky 이미지를 항상 맨 앞에 위치 (Map 바로 위)
        self.pinky_image.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents)  # 마우스 이벤트 무시
        self.pinky_image.raise_()  # 맨 앞에 위치

    def update_pinky_position(self, x, y):
        print("[DEBUG] RidingWindow에서 받은 위치:", x, y)
        self.pinky_image.move(int(x - 25), int(y - 25))



    def switch_to_main_window(self):    
        self.pinky.stop()
        self.pinky.set_destinations((0, 0), (0, 0))  # 좌표 초기화    

        from main import MainWindow
        self.main_window = MainWindow()        
        self.main_window.show()
        self.close()

    def get_ui_path(self, ui_file):

        base_dir = os.path.dirname(os.path.abspath(__file__))
        return os.path.join(base_dir, "ui", ui_file)


    def show_charge_page(self):
        from charge import ChargeWindow
        self.charge_window = ChargeWindow(
            previous_window=self,
            start_icon=self.start_icon,
            destination_icon=self.destination_icon
        )
        self.charge_window.show()
        self.hide()

    def restore_state(self, start_icon, destination_icon):
        self.start_icon = start_icon
        self.destination_icon = destination_icon
        self.left_money_manager = LeftMoneyManager(self.LeftMoney)