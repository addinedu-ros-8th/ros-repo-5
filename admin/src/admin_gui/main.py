import sys
import threading
from PyQt6.QtWidgets import QApplication, QMainWindow, QGraphicsScene, QGraphicsEllipseItem
from PyQt6.QtGui import QColor
from PyQt6 import uic
from qt_material import apply_stylesheet

import rclpy
from rclpy.node import Node
from controll_server_package_msgs.msg import TaxiState

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
        uic.loadUi("/home/vit/dev_ws/project/ros-repo-5/admin/src/admin_gui/main.ui", self)

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

        self.button_log.clicked.connect(self.print_log)

    def update_taxi_status(self, msg):
        vehicle_id = msg.vehicle_id
        if vehicle_id in self.taxis and len(msg.location) == 2:
            x, y = msg.location
            self.taxis[vehicle_id].setPos(x, y)

            # 상태 문자열 한글 변환
            state_kor = self.STATE_KOR_MAP.get(msg.state, msg.state)

            # 공통 스타일 적용
            def styled(text): return f'<span style="font-size:16pt; font-weight:600;">{text}</span>'

            if vehicle_id == 1:
                self.label_pinky1_status.setText(styled(f"운행 상태: {state_kor}"))
                self.label_pinky1_battery.setText(styled(f"배터리: {msg.battery:.0f}%"))
                self.label_pinky1_position.setText(styled(f"위치: ({x:.3f}, {y:.3f})"))
                self.label_pinky1_log.setText(styled(f"탑승자: {msg.passenger_count}명"))
            elif vehicle_id == 2:
                self.label_pinky2_status.setText(styled(f"운행 상태: {state_kor}"))
                self.label_pinky2_battery.setText(styled(f"배터리: {msg.battery:.0f}%"))
                self.label_pinky2_position.setText(styled(f"위치: ({x:.3f}, {y:.3f})"))
                self.label_pinky1_log_2.setText(styled(f"탑승자: {msg.passenger_count}명"))



    def print_log(self):
        print("LOG 버튼이 눌렸습니다.")

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
