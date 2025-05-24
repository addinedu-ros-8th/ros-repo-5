# admin_gui_main.py
import sys
from PyQt6.QtWidgets import *
from PyQt6.QtCore import QTimer
import rclpy
from rclpy.node import Node
from controll_server_package_msgs.srv import CmdMoveTo



class AdminServiceNode(Node):
    def __init__(self, gui_callback):
        super().__init__('admin_gui_node')
        self.srv = self.create_service(CmdMoveTo, 'admin_move_to_robot', self.handle_request)
        self.gui_callback = gui_callback

    def handle_request(self, request, response):
        msg = f"[요청 수신] 로봇 {request.pinky_num} → 위치({request.x}, {request.y})"
        print(msg)
        self.gui_callback(msg)  # GUI 업데이트
        response.success = True
        response.message = f"로봇 {request.pinky_num} 이동 명령 수락됨"
        return response

class WindowClass(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Admin GUI")
        self.resize(400, 200)
        self.label = QLabel("서비스 대기 중...", self)
        self.label.setGeometry(20, 20, 360, 40)
        self.label.setWordWrap(True)

        self.node = AdminServiceNode(self.update_status)

        # 타이머로 ROS spin 돌리기
        self.timer = QTimer()
        self.timer.timeout.connect(lambda: rclpy.spin_once(self.node, timeout_sec=0.1))
        self.timer.start(100)

    def update_status(self, msg):
        self.label.setText(msg)

def main():
    rclpy.init()
    app = QApplication(sys.argv)
    window = WindowClass()
    window.show()
    sys.exit(app.exec())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
