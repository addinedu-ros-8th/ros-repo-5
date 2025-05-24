import sys
import os
import threading
from PyQt6.QtWidgets import QPushButton, QApplication, QMainWindow, QGraphicsScene, QGraphicsEllipseItem, QGraphicsPixmapItem, QGroupBox
from PyQt6.QtGui import QColor, QPixmap
from PyQt6 import uic
from PyQt6.QtCore import QTimer, QPointF, Qt, pyqtSignal, QPropertyAnimation, QPointF
from qt_material import apply_stylesheet
from PyQt6.QtCore import QLoggingCategory


import rclpy
from rclpy.node import Node
from controll_server_package_msgs.msg import TaxiState
from Icon_coordinates_for_main import CoordinateMapper
from ros_manager import init_ros, update_callback, is_ros_initialized






# stderr (경고 메시지) 출력 차단
sys.stderr = open(os.devnull, 'w')
# 모든 Qt 경고 메시지 차단
QLoggingCategory.setFilterRules("*.debug=false\n*.warning=false\n*.critical=false\n*.fatal=false")

global_ros_node = None


class RosSubscriberNode(Node):
    _instance = None

    @staticmethod
    def get_instance(callback):
        if RosSubscriberNode._instance is None:
            RosSubscriberNode._instance = RosSubscriberNode(callback)
        return RosSubscriberNode._instance

    def __init__(self, callback):
        super().__init__('admin_gui_node')
        self.subscription = self.create_subscription(
            TaxiState, '/admin_gui_topic', callback, 10)

class AdminMainWindow(QMainWindow):
    vehicle_status_updated = pyqtSignal(int, object)
    

    def __init__(self, parent=None):
        super().__init__(parent)
        uic.loadUi("main.ui", self)
        self.latest_positions = {}   # 택시별 최신 위치 저장 (px, py)
        # self.animations = {} 
        self.taxi_states = {}  # vehicle_id → state 저장용 딕셔너리

        
        
        if not is_ros_initialized():
            print("[ROS INIT] 새로운 ROS 노드 생성 및 콜백 등록")
            init_ros(self.update_taxi_status)

        else:
            print("[ROS INIT] 기존 노드에 콜백만 업데이트")
            update_callback(self.update_taxi_status)  


        # 상태 코드 → 한글 매핑
        self.STATE_KOR_MAP = {
            "ready": "대기 중",
            "dispatch": "배차 중",
            "drive_start": "출발지로 이동 중",
            "boarding": "승객 승차 중",
            "drive_destination": "목적지 이동 중",
            "landing": "승객 하차 중",
            "completed": "운행 및 하차 완료",
            "charged": "충전 중"
        }

        self.scene = QGraphicsScene()
        self.graphicsView_map.setScene(self.scene)
        print("[DEBUG] Scene 설정 완료")
   

        self.button_log.clicked.connect(self.log_click)
        self.mapper = CoordinateMapper(img_width=961, img_height=521, marker_length=0.1)
        self.setup_pinky_items()
        
        
        
        for taxi_id, groupbox in [(1, self.groupBox_taxi1), (2, self.groupBox_taxi2)]:
            button = QPushButton(groupbox)
            button.setGeometry(groupbox.rect())
            button.setStyleSheet("background: transparent; border: none;")
            button.clicked.connect(lambda _, tid=taxi_id: self.on_groupbox_click(tid))


    def update_taxi_status(self, msg):
        vehicle_id = msg.vehicle_id
        print("="*40)
        print(f"[DEBUG] 수신된 TaxiState 원본 메시지:\n{msg}")
        print("="*40)
        
        if msg.battery == 0.0:
            print(f"[FORCE STATE] 차량 {vehicle_id} 상태를 '충전 중'으로 설정")
            msg.state = "charged"
            

        self.taxi_states[vehicle_id] = msg.state
        print(f"[DEBUG] 수신됨 vehicle_id={vehicle_id}, location={msg.location}")

        if vehicle_id not in self.pinky_items:
            print(f"[SKIP] vehicle_id={vehicle_id} is not in pinky_items")
            return

        if len(msg.location) != 2:
            print(f"[SKIP] vehicle_id={vehicle_id} location invalid: {msg.location}")
            return

        x_world, y_world = msg.location
        


        if abs(x_world) < 1e-3 and abs(y_world) < 1e-3:
            print(f"[HIDE] 차량 {vehicle_id} 위치가 (0.0, 0.0) → 아이콘 숨김")
            self.pinky_items[vehicle_id].setVisible(False)
            return
        
        px, py = self.mapper.world_to_pixel(x_world, y_world)
        px = max(0, min(px, self.graphicsView_map.width()))
        py = max(0, min(py, self.graphicsView_map.height()))
        print(f"[DEBUG] Affine 결과: px={px}, py={py}")

        self.update_pinky_position(vehicle_id, px, py)
        self.latest_positions[vehicle_id] = (px, py)

        self.vehicle_status_updated.emit(vehicle_id, msg)
        print(f"[DEBUG] 차량 {vehicle_id} | world=({x_world:.3f}, {y_world:.3f}) → pixel=({px}, {py})")

        state_kor = self.STATE_KOR_MAP.get(msg.state, msg.state)
        styled = lambda text: f'<span style="font-size:16pt; font-weight:600;">{text}</span>'

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

        self.update_taxi_summary()



    def on_groupbox_click(self, vehicle_id):
        from individual import IndividualWindow
        self.individual_window = IndividualWindow(vehicle_id, parent_main_window=self)
        try:
             self.vehicle_status_updated.disconnect(self.individual_window.receive_taxi_status)
        except TypeError:
             pass
        self.vehicle_status_updated.connect(self.individual_window.receive_taxi_status)

        self.hide()
        self.individual_window.show()

    def log_click(self): 
        from log import logWindow
        self.log_window = logWindow(
            parent_main_window=self
        )
        self.hide()
        self.log_window.show()
    
    def setup_pinky_items(self):
        self.pinky_items = {}
        
        for vehicle_id in [1, 2]:
            pixmap = QPixmap(f"data/pinky_{vehicle_id}.png").scaled(80, 80, Qt.AspectRatioMode.IgnoreAspectRatio, Qt.TransformationMode.SmoothTransformation)
            item = QGraphicsPixmapItem(pixmap)
            item.setOffset(-40, -40)  # 중심 정렬
            item.setZValue(10)
            item.setVisible(False)
            self.scene.addItem(item)
            self.pinky_items[vehicle_id] = item


    # def animate_icon_move(self, vehicle_id: int, px: int, py: int):
    #     item = self.pinky_items[vehicle_id]
        
    #     # 현재 위치에서 새 위치까지 애니메이션 설정
    #     animation = QPropertyAnimation(item, b"pos", self)
    #     animation.setDuration(300)  # 밀리초 단위 (부드러운 이동)
    #     animation.setStartValue(item.pos())
    #     animation.setEndValue(QPointF(px, py))
    #     animation.start()
        
    #     # 애니메이션이 소멸되지 않도록 저장 (안 그러면 바로 사라짐)
    #     self.animations[vehicle_id] = animation
    #     # 애니메이션 완료 시 자동 삭제
    #     # animation.finished.connect(lambda: self.animations.pop(vehicle_id, None))
    
    def update_pinky_position(self, vehicle_id, x, y):
        if vehicle_id not in self.pinky_items:
            print(f"[WARN] Unknown vehicle_id {vehicle_id} in update_pinky_position")
            return
        
        x = max(40, min(x, self.mapper.img_width - 40))
        y = max(40, min(y, self.mapper.img_height - 40))
        
        item = self.pinky_items[vehicle_id]
        item.setPos(QPointF(x, y))
        
        if not item.isVisible():
            item.setVisible(True)
    
    
            
    def update_taxi_summary(self):
        total = len(self.taxi_states)
        driving = sum(1 for s in self.taxi_states.values() if s.startswith("drive"))
        idle = sum(1 for s in self.taxi_states.values() if s == "ready")
        charging = sum(1 for s in self.taxi_states.values() if s == "charged")
        
        self.label_totalTaxi.setText(f"전체 택시 수: {total}")
        self.label_drivingTaxi.setText(f"운행 중: {driving}")
        self.label_waitingTaxi.setText(f"대기 중: {idle}")
        self.label_chargingTaxi.setText(f"충전 중: {charging}")

            
            

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = AdminMainWindow()
    apply_stylesheet(app, theme='dark_blue.xml')
    window.show()


    sys.exit(app.exec())
