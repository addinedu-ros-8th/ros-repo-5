import sys
from PyQt6.QtWidgets import QApplication, QMainWindow, QGraphicsScene, QGraphicsEllipseItem, QGraphicsPixmapItem
from PyQt6.QtGui import QPixmap, QColor
from PyQt6.QtCore import Qt
from PyQt6 import uic


class AdminMainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        uic.loadUi("/home/vit/dev_ws/project/ros-repo-5/admin/src/admin_gui/main.ui", self)

        # UI 초기화
        self.label_totalTaxi.setText("전체 택시 수: 2대")
        self.label_drivingTaxi.setText("운행 중: 1대")
        self.label_waitingTaxi.setText("대기 중: 1대")
        self.label_chargingTaxi.setText("충전 중: 0대")

        self.label_pinky1_status.setText("운행 상태: 운행 중")
        self.label_pinky1_speed.setText("속도: 15 cm/s")
        self.label_pinky1_battery.setText("배터리: 80%")
        self.label_pinky1_position.setText("위치: (35, 60)")
        self.label_pinky1_log.setText("정지선 감지 → 감속")

        self.label_pinky2_status.setText("운행 상태: 대기")
        self.label_pinky2_speed.setText("속도: 0 cm/s")
        self.label_pinky2_battery.setText("배터리: 100%")
        self.label_pinky2_position.setText("위치: (0, 0)")
        self.label_pinky1_log_2.setText("대기 중")

        self.button_log.clicked.connect(self.print_log)

        # 맵 + 택시 아이콘 초기화
        self.setup_graphics_view()

    def setup_graphics_view(self):
        # QGraphicsScene 생성
        self.scene = QGraphicsScene()
        self.graphicsView_map.setScene(self.scene)

        # 지도 이미지
        bg_pixmap = QPixmap("/home/vit/dev_ws/project/ros-repo-5/admin/src/admin_gui/data/map/map.png")
        if not bg_pixmap.isNull():
            self.bg_item = QGraphicsPixmapItem(bg_pixmap)
            self.scene.addItem(self.bg_item)

             # 🎯 수정된 fitInView (boundingRect + Qt enum 사용)
            self.graphicsView_map.fitInView(self.bg_item.boundingRect(), Qt.AspectRatioMode.KeepAspectRatio)
        else:
            print("⚠️ map.png 이미지가 존재하지 않거나 경로가 잘못되었습니다.")

        # 택시 아이콘 추가
        self.taxi_item = QGraphicsEllipseItem(0, 0, 20, 20)
        self.taxi_item.setBrush(QColor("magenta"))
        self.scene.addItem(self.taxi_item)

        # 초기 위치
        self.taxi_item.setPos(150, 300)

    def print_log(self):
        print("LOG 버튼이 눌렸습니다.")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = AdminMainWindow()
    window.show()
    sys.exit(app.exec())
