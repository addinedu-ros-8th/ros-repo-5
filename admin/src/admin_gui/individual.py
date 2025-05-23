import sys
import os
import numpy as np
import threading
from PyQt6.QtWidgets import QApplication, QMainWindow, QGraphicsScene, QGraphicsEllipseItem, QLabel
from PyQt6.QtGui import QColor, QPixmap, QImage
from PyQt6 import uic
from qt_material import apply_stylesheet
from PyQt6.QtCore import QLoggingCategory, QMetaObject, Qt, QPropertyAnimation, QPoint
from icon_coordination_for_individual import CoordinateMapper
import socket
import cv2 
import zlib

import rclpy
from rclpy.node import Node
from controll_server_package_msgs.msg import TaxiState
from icon_coordination_for_individual import CoordinateMapper


IP = "192.168.1.4"
PORT_MAP= {
    1: 9000,
    2: 9001
}

# PORT = 9999

# stderr (경고 메시지) 출력 차단
sys.stderr = open(os.devnull, 'w')
# 모든 Qt 경고 메시지 차단
QLoggingCategory.setFilterRules("*.debug=false\n*.warning=false\n*.critical=false\n*.fatal=false")


def ros_spin(gui):
    rclpy.init()
    node = RosSubscriberNode(gui.update_taxi_status)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



class UdpVideoReceiver:
    def __init__(self, port, update_callback):
        self.port = port
        self.running = False
        self.update_callback = update_callback  # QPixmap 전달용 콜백

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self.receive_loop, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False

    def receive_loop(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("", self.port))
        print(f"[UDP] Listening on port {self.port}")

        while self.running:
            try:
                data, addr = sock.recvfrom(65535)
                # print(f"[RECV] {len(data)} bytes received from {addr}")

                try:
                    decompressed = zlib.decompress(data)
                except zlib.error as e:
                    # print(f"[ZLIB ERROR] decompress failed: {e}")
                    continue

                np_data = np.frombuffer(decompressed, dtype=np.uint8)
                # print(f"[NP] Decompressed to numpy array, shape: {np_data.shape}")

                frame = cv2.imdecode(np_data, cv2.IMREAD_COLOR)
                if frame is None:
                    # print("[WARN] cv2.imdecode failed - frame is None")
                    continue

                # print(f"[FRAME] Decoded frame size: {frame.shape}")
                height, width, channel = frame.shape
                bytes_per_line = 3 * width
                qimg = QImage(frame.data, width, height, bytes_per_line, QImage.Format.Format_BGR888)
                self.update_callback(QPixmap.fromImage(qimg))

            except Exception as e:
                print(f"[UDP ERROR] {e}")

class IndividualWindow(QMainWindow):
    def __init__(self, vehicle_id):
        super().__init__()
        uic.loadUi("individual.ui", self)
        self.vehicle_id = vehicle_id
        self.Vehicle_ID.setText(f"{vehicle_id}")
        
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
        PORT = PORT_MAP[self.vehicle_id]
        self.receiver = UdpVideoReceiver(port=PORT, update_callback=self.update_frame)
        self.receiver.start()
        self.undoBtn.clicked.connect(self.go_main)
        
        self.mapper = CoordinateMapper(img_width=521, img_height=351, marker_length=0.1)

        self.setup_pinky_image()
        self.pinky_image.setVisible(False)


    def go_main(self):
        # self.closeEvent()
        from main import AdminMainWindow
        self.main_window = AdminMainWindow()
        self.main_window.show()
        self.close()


    def update_frame(self, pixmap: QPixmap):
        # print(f"[UI] Received pixmap of size: {pixmap.width()}x{pixmap.height()}")
        scaled = pixmap.scaled(
            self.realtime_video.width(),
            self.realtime_video.height(),
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation
        )
        self.realtime_video.setPixmap(scaled)


    def closeEvent(self, event):
        self.receiver.stop()
        super().closeEvent(event)

    def update_taxi_status(self, msg):
        if msg.vehicle_id != self.vehicle_id:
            return  # 내가 보고 있는 핑키가 아니라면 무시
    
        QMetaObject.invokeMethod(
            self,
            lambda: self.update_labels(msg),
            Qt.ConnectionType.QueuedConnection
        )

    def update_taxi_status(self, msg):
        vehicle_id = msg.vehicle_id
        
        if vehicle_id in self.pinky_items and len(msg.location) == 2:
            x_world, y_world = msg.location
            px, py = self.mapper.world_to_pixel(x_world, y_world)
            self.latest_positions[vehicle_id] = (px, py)
            # → 바로 부드럽게 애니메이션 이동!
            self.animate_pinky_move(vehicle_id, px, py)

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
    



    def setup_pinky_image(self):
        
        # Map 위젯 위에 Pinky 설정
        self.pinky_image = QLabel(self.Map)  # Map의 자식으로 Pinky 설정
        pinky_pixmap = QPixmap("data/map_icon/pinky.png")
        
        # QPixmap 투명 배경 유지 (Alpha 채널 유지)
        self.pinky_image.setPixmap(pinky_pixmap)
        self.pinky_image.setGeometry(0,0,60, 60)  # 초기 크기와 위치 설정
        self.pinky_image.setScaledContents(True)
        self.pinky_image.setStyleSheet("background: transparent;")  # 투명 배경 유지
        
        # Pinky 이미지를 항상 맨 앞에 위치 (Map 바로 위)
        self.pinky_image.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents)  # 마우스 이벤트 무시
        self.pinky_image.raise_()  # 맨 앞에 위치
    

    def update_pinky_position(self, x, y):
        if not self.pinky_image.isVisible():
            self.pinky_image.move(int(x - 30), int(y - 30))
            self.pinky_image.setVisible(True)
        else:
            print( x, y)
            self.pinky_image.move(int(x - 30), int(y - 30))


 

    


class RosSubscriberNode(Node):
    def __init__(self, gui_callback):
        super().__init__('individual_gui_node')
        self.subscription = self.create_subscription(
            TaxiState,
            '/admin_gui_topic',
            gui_callback,
            10
        )

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = IndividualWindow()
    apply_stylesheet(app, theme='dark_blue.xml')
    window.show()

    # ROS 노드 스레드 실행
    ros_thread = threading.Thread(target=ros_spin, args=(window,), daemon=True)
    ros_thread.start()

    sys.exit(app.exec())
