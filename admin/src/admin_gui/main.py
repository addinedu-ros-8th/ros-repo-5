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

            # UI 라벨 업데이트
            if vehicle_id == 1:
                self.label_pinky1_status.setText(f"운행 상태: {msg.state}") # self.state = ready dispatch drive_start boarded drive_destination landing completed charged
                self.label_pinky1_battery.setText(f"배터리: {msg.battery:.0f}%")
                self.label_pinky1_position.setText(f"위치: ({int(x)}, {int(y)})")
                self.label_pinky1_log.setText(f"탑승자: {msg.passenger_count}명")
            elif vehicle_id == 2:
                self.label_pinky2_status.setText(f"운행 상태: {msg.state}") # self.state = ready dispatch drive_start boarded drive_destination landing completed charged
                self.label_pinky2_battery.setText(f"배터리: {msg.battery:.0f}%")
                self.label_pinky2_position.setText(f"위치: ({int(x)}, {int(y)})")
                self.label_pinky1_log_2.setText(f"탑승자: {msg.passenger_count}명")

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
