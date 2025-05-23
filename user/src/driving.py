import os
from PyQt6.QtWidgets import QMainWindow, QApplication, QLabel
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
from PyQt6.QtCore import QPropertyAnimation, QPoint


# stderr (경고 메시지) 출력 차단
sys.stderr = open(os.devnull, 'w')

# 모든 Qt 경고 메시지 차단
QLoggingCategory.setFilterRules("*.debug=false\n*.warning=false\n*.critical=false\n*.fatal=false")


class DrivingWindow(QMainWindow):
    def __init__(self, start_icon_name, destination_icon_name, mapper):
        super().__init__()
        uic.loadUi(self.get_ui_path("/home/lim/git/ros-repo-5/user/src/ui/3_driving.ui"), self)
        self.mapper = mapper

        # REST API 클라이언트 생성
        self.api_manager = RestAPIManager()
        self.start_location = None
        self.end_location = None
        
        
        self.ChargeBtn.clicked.connect(self.show_charge_page)

        # LineEditHandler 설정
        self.line_edit_handler = LineEditHandler(self)
        self.location_names = LocationManager.get_location_names()
        self.left_money_manager = LeftMoneyManager(self.LeftMoney) 
        
        # 아이콘 핸들러 설정
        self.icon_handler = IconHandler(self, {
            "Icon1": "Info1",
            "Icon2": "Info2",
            "Icon3": "Info3",
            "Icon4": "Info4",
            "Icon5": "Info5",
            "Icon6": "Info6"
        })
         
        self.start_location = self.start  # QLineEdit (출발지)
        self.destination_location = self.destination  # QLineEdit (목적지)

        # 출발지/목적지 텍스트 설정
        self.set_location_text(start_icon_name, destination_icon_name)
        
        # 출발지/목적지 이름을 상태로 저장
        self.start_icon = start_icon_name
        self.destination_icon = destination_icon_name
        self.set_location_text(self.start_icon, self.destination_icon)

        # PinkyManager 설정
        self.setup_pinky_image()
        self.pinky_image.setVisible(False)
        self.pinky = PinkyManager.get_instance(mapper=self.mapper)
        self.pinky.position_updated.connect(self.update_pinky_position)
        self.pinky.destination_reached.connect(self.switch_to_end_window)
        self.end_shown = False



        
        # 디버그 버튼 설정
    


        # 출발지/목적지 아이콘 유지
        self.start_location = self.start  # QLineEdit (출발지)
        self.destination_location = self.destination
        self.icon_handler.set_fixed_icons(start_icon_name, destination_icon_name)
    
    def setup_pinky_image(self):
        # Map 위젯 위에 Pinky 설정
        self.pinky_image = QLabel(self.Map)  # Map의 자식으로 Pinky 설정
        pinky_pixmap = QPixmap("/home/lim/dev_ws/addintexi/UserGUI/data/map_icon/pinky.png")
        
        # QPixmap 투명 배경 유지 (Alpha 채널 유지)
        self.pinky_image.setPixmap(pinky_pixmap)
        self.pinky_image.setGeometry(0,0,60, 60)  # 초기 크기와 위치 설정
        self.pinky_image.setScaledContents(True)
        self.pinky_image.setStyleSheet("background: transparent;")  # 투명 배경 유지
        
        # Pinky 이미지를 항상 맨 앞에 위치 (Map 바로 위)
        self.pinky_image.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents)  # 마우스 이벤트 무시
        self.pinky_image.raise_()  # 맨 앞에 위치


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
        self.set_location_text(start_icon, destination_icon)
        self.left_money_manager = LeftMoneyManager(self.LeftMoney) 

    #----------------------------------------------------------------
    # 디버깅용 
    # ----------------------------------------------------------------
    
    def animate_pinky_move(self, x, y):
        end_pos = QPoint(int(x - 30), int(y - 30))
        self.pinky_image.setVisible(True)
        # 이미 이전 애니메이션이 있으면 중
        if hasattr(self, 'pinky_anim') and self.pinky_anim is not None:
            self.pinky_anim.stop()
        self.pinky_anim = QPropertyAnimation(self.pinky_image, b"pos", self)
        self.pinky_anim.setDuration(300)   # 이동 시간(ms) - 0.3초
        self.pinky_anim.setStartValue(self.pinky_image.pos())
        self.pinky_anim.setEndValue(end_pos)
        self.pinky_anim.start()
          
   
          


    #---------------------------------------------------------------------------

    def update_pinky_position(self, x, y):
        if not self.pinky_image.isVisible():
            self.pinky_image.move(int(x - 30), int(y - 30))
            self.pinky_image.setVisible(True)
        else:
            print("[DEBUG] RidingWindow에서 받은 위치:", x, y)
            # self.pinky_image.move(int(x - 30), int(y - 30))
            self.animate_pinky_move(x, y)


    def switch_to_end_window(self):
        if self.end_shown:
            return
        self.end_shown = True
        start_icon_name = self.icon_handler.start_icon.objectName() if self.icon_handler.start_icon else "Unknown"
        destination_icon_name = self.icon_handler.destination_icon.objectName() if self.icon_handler.destination_icon else "Unknown"
        from end import EndWindow
        self.end_window = EndWindow(
            start_icon_name=start_icon_name,
            destination_icon_name=destination_icon_name,
            mapper=self.pinky.mapper
        )
        self.end_window.show()
        self.close()
        

    def set_location_text(self, start_icon_name, destination_icon_name):
        location_names = LocationManager.get_location_names()

        # 출발지 LineEdit 업데이트
        start_name = self.location_names.get(start_icon_name, "출발지 선택")
        self.line_edit_handler.update_line_edit("start", start_name)

        # 목적지 LineEdit 업데이트
        destination_name = self.location_names.get(destination_icon_name, "목적지 선택")
        self.line_edit_handler.update_line_edit("destination", destination_name)

    def get_ui_path(self, ui_file):
        base_dir = os.path.dirname(os.path.abspath(__file__))
        return os.path.join(base_dir, "ui", ui_file)