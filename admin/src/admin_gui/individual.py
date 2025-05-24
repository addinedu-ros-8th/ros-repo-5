import sys
import os
import numpy as np
import threading
from PyQt6.QtWidgets import QApplication, QMainWindow, QGraphicsScene, QGraphicsEllipseItem, QLabel
from PyQt6.QtGui import QColor, QPixmap, QImage
from PyQt6 import uic

from PyQt6.QtCore import QLoggingCategory, QMetaObject, Qt, QPropertyAnimation, QPoint
from icon_coordination_for_individual import *
import socket
import cv2 
import zlib


from icon_coordination_for_individual import CoordinateMapper

ICON_HALF = 30 

IP = "192.168.1.3"
PORT_MAP= {
    1: 9001,
    2: 9000
}

ICON_NODE_NAMES = {
    (0.262, 0.161): "애드인에듀 학원",
    (0.17, 0.217): "기억할게!술집",
    (-0.015, -0.078): "내 거친 생각 정신과",
    (-0.045, 0.021): "불안한 눈빛 안과",
    (0.08, 0.069): "신라호텔",
    (0.1, 0.097): "월드컵 경기장",
    (0.0, 0.0): "준비중"
}

# PORT = 9999

# stderr (경고 메시지) 출력 차단
sys.stderr = open(os.devnull, 'w')
# 모든 Qt 경고 메시지 차단
QLoggingCategory.setFilterRules("*.debug=false\n*.warning=false\n*.critical=false\n*.fatal=false")


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
                    print(f"[ZLIB ERROR] decompress failed: {e}")
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
    def __init__(self, vehicle_id, parent_main_window):
        super().__init__()
        uic.loadUi("individual.ui", self)
        self.vehicle_id = vehicle_id
        self.Vehicle_ID.setText(f"{vehicle_id}")
        self.Map.setFixedSize(521, 351) 
        self.parent_main_window = parent_main_window
        self.Battery_remain.setVisible(False)

        
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
        self.hide()
        self.parent_main_window.show() 
        #self.main_window.show()
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


    def receive_taxi_status(self, vehicle_id, msg):
        if vehicle_id != self.vehicle_id:
            return
        
        if msg.state == "ready" and msg.passenger_count == 0 and msg.battery == 0.0:
            print(f"[FORCE STATE] 차량 {vehicle_id} 상태를 '충전 중'으로 설정")
            msg.state = "charged"
        x_world, y_world = msg.location
        px, py = self.mapper.world_to_pixel(x_world, y_world)
        self.update_pinky_position(px, py)

        state_kor = self.STATE_KOR_MAP.get(msg.state, msg.state)
        styled = lambda text: f"<span style=\"font-size:16pt; font-weight:600;\">{text}</span>"

        node_name = self.find_nearest_node(px, py)


        self.condition.setText(state_kor)
        self.remain_Battery_text.setText(f"{msg.battery:.0f}%")
        self.update_battery_visual(msg.battery)
        self.passenger.setText(styled(f"탑승자: {msg.passenger_count}명"))
        start_x, start_y = msg.start
        dest_x, dest_y = msg.destination
        
        start_name = self.find_nearest_node(start_x, start_y)
        dest_name = self.find_nearest_node(dest_x, dest_y)
        
        self.record_text.setText(f"출발지: {start_name} / 목적지: {dest_name}")

    def update_battery_visual(self, percent: float):
        base_x, base_y = 10, 540       
        base_width = 81
        max_height = 121               # 100%일 때 높이
        
        if percent <= 0:
            self.Battery_remain.setVisible(False)
            return
        
        self.Battery_remain.setVisible(True)
        clamped = min(max(percent, 0), 100)
        new_height = int((clamped / 100.0) * max_height)
        
        # 하단 기준으로 위로 줄어들게 위치 조정
        new_y = base_y + (max_height - new_height)
        self.Battery_remain.setGeometry(base_x, new_y, base_width, new_height)
        self.Battery_remain.setText(f"{int(clamped)}%")




    def setup_pinky_image(self):
        self.pinky_image = QLabel(self.Map)

        if self.vehicle_id == 1: 
            pinky_pixmap = QPixmap("data/pinky_1.png")
        else:
            pinky_pixmap = QPixmap("data/pinky_2.png")

        self.pinky_image.setPixmap(pinky_pixmap)
        self.pinky_image.setGeometry(0, 0, 60, 60)
        self.pinky_image.setScaledContents(True)
        self.pinky_image.setStyleSheet("background: transparent;")
        self.pinky_image.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents)
        self.pinky_image.raise_()
    

    def update_pinky_position(self, x, y):
        x = max(ICON_HALF, min(x, self.mapper.img_width - ICON_HALF))
        y = max(ICON_HALF, min(y, self.mapper.img_height - ICON_HALF))
        
        if not self.pinky_image.isVisible():
            self.pinky_image.move(int(x - 30), int(y - 30))
            self.pinky_image.setVisible(True)
        else:
            self.pinky_image.move(int(x - 30), int(y - 30))

    def find_nearest_node(self, px, py):
        min_dist = float("inf")
        closest_name = "?"
        for (wx, wy), name in ICON_NODE_NAMES.items():
            mapped_x, mapped_y = self.mapper.world_to_pixel(wx, wy)
            dist = (mapped_x - px) ** 2 + (mapped_y - py) ** 2
            if dist < min_dist:
                min_dist = dist
                closest_name = name
        return closest_name





if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = IndividualWindow(vehicle_id=1, parent_main_window=None)

    window.show()

    # ROS 노드 스레드 실행
    # ros_thread = threading.Thread(target=ros_spin, args=(window,), daemon=True)
    # ros_thread.start()

    sys.exit(app.exec())
