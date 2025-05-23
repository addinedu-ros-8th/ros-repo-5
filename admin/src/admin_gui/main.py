import sys
import os
import threading
from PyQt6.QtWidgets import QPushButton, QApplication, QMainWindow, QGraphicsScene, QGraphicsEllipseItem, QGraphicsPixmapItem, QGroupBox
from PyQt6.QtGui import QColor, QPixmap
from PyQt6 import uic
from PyQt6.QtCore import QTimer, QPointF
from qt_material import apply_stylesheet
from PyQt6.QtCore import QLoggingCategory


import rclpy
from rclpy.node import Node
from controll_server_package_msgs.msg import TaxiState
from Icon_coordinates_for_main import CoordinateMapper

# stderr (경고 메시지) 출력 차단
sys.stderr = open(os.devnull, 'w')
# 모든 Qt 경고 메시지 차단
QLoggingCategory.setFilterRules("*.debug=false\n*.warning=false\n*.critical=false\n*.fatal=false")


class RosSubscriberNode(Node):
    def __init__(self, gui_callback):
        super().__init__('admin_gui_node')
        self.subscription = self.create_subscription(
            TaxiState,
            '/admin_gui_topic',
            gui_callback,
            10
        )


class AdminMainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        uic.loadUi("main.ui", self)
        self.latest_positions = {}   # 택시별 최신 위치 저장 (px, py)
        self.animations = {} 

        # 상태 코드 → 한글 매핑
        self.STATE_KOR_MAP = {
            "ready": "대기 중",
            "dispatch": "배차 중",
            "drive_start": "출발지로 이동 중",
            "boarded": "승객 승차 중",
            "drive_destination": "목적지 이동 중",
            "landing": "승객 하차 중",
            "completed": "운행 및 하차 완료",
            "charged": "충전 중"
        }

        # 택시 상태 초기화
        self.taxis = {
            1: QGraphicsEllipseItem(0, 0, 20, 20),
            2: QGraphicsEllipseItem(0, 0, 20, 20)
        }

        self.scene = QGraphicsScene()
        self.graphicsView_map.setScene(self.scene)
        for taxi in self.taxis.values():
            taxi.setBrush(QColor("magenta"))
            self.scene.addItem(taxi)

        # self.button_log.clicked.connect(self.print_log)
        self.mapper = CoordinateMapper(
            img_width=961, 
            img_height=521, 
            marker_length=0.1)
        self.setup_pinky_image()  
        
        
        button_1 = QPushButton(self.groupBox_taxi1)
        button_1.setGeometry(self.groupBox_taxi1.rect())
        button_1.setStyleSheet("background: transparent; border: none;")
        button_1.clicked.connect(lambda: self.on_groupbox_click(1))

        button_2 = QPushButton(self.groupBox_taxi2)
        button_2.setGeometry(self.groupBox_taxi2.rect())
        button_2.setStyleSheet("background: transparent; border: none;")
        button_2.clicked.connect(lambda: self.on_groupbox_click(2))


    def update_taxi_status(self, msg):
        vehicle_id = msg.vehicle_id
        
        if vehicle_id in self.pinky_items and len(msg.location) == 2:
            x_world, y_world = msg.location
            px, py = self.mapper.world_to_pixel(x_world, y_world)
            self.latest_positions[vehicle_id] = (px, py)

            # 상태 문자열 한글 변환
            state_kor = self.STATE_KOR_MAP.get(msg.state, msg.state)

            # 공통 스타일 적용
            def styled(text): return f'<span style="font-size:16pt; font-weight:600;">{text}</span>'

            if vehicle_id == 1:
                self.label_pinky1_status.setText(styled(f"운행 상태: {state_kor}"))
                self.label_pinky1_battery.setText(styled(f"배터리: {msg.battery:.0f}%"))
                self.label_pinky1_position.setText(styled(f"위치: ({x_world:.3f}, {y_world:.3f})"))
                self.label_pinky1_log.setText(styled(f"탑승자: {msg.passenger_count}명"))

            elif vehicle_id == 2:
                self.label_pinky2_status.setText(styled(f"운행 상태: {state_kor}"))
                self.label_pinky2_battery.setText(styled(f"배터리: {msg.battery:.0f}%"))
                self.label_pinky2_position.setText(styled(f"위치: ({x_world:.3f}, {y_world:.3f})"))
                self.label_pinky1_log_2.setText(styled(f"탑승자: {msg.passenger_count}명"))
    

    def on_groupbox_click(self, vehicle_id):
        #  택시 1 클릭인지 2클릭인지 받아올 것. 
        print(f"[DEBUG] 택시 {vehicle_id} 그룹박스 클릭됨")
        from individual import IndividualWindow
        self.individual_window = IndividualWindow(vehicle_id=vehicle_id)
        self.individual_window.show()
        self.close()

    def log_click(self): 
        from log import logWindow
        self.log_window = logWindow()
        self.log_window.show()
        self.close()


    def setup_pinky_image(self):
        self.pinky_items = {
            1: QGraphicsPixmapItem(QPixmap("Icon/pinky_1.png").scaled(40, 40)),
            2: QGraphicsPixmapItem(QPixmap("Icon/pinky_2.png").scaled(40, 40)),
        }
        for item in self.pinky_items.values():
            item.setZValue(10) 
            self.scene.addItem(item)


    def update_taxi_icons(self):
        for vehicle_id, (px, py) in self.latest_positions.items():
            if vehicle_id in self.taxis:
                self.taxis[vehicle_id].setPos(px, py)


def ros_spin(gui):
    rclpy.init()
    node = RosSubscriberNode(gui.update_taxi_status)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = AdminMainWindow()
    apply_stylesheet(app, theme='dark_blue.xml')
    window.show()

    # ROS 노드 스레드 실행
    ros_thread = threading.Thread(target=ros_spin, args=(window,), daemon=True)
    ros_thread.start()

    sys.exit(app.exec())
