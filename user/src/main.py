import sys
from PyQt6.QtWidgets import QApplication, QMainWindow, QLineEdit, QLabel
from PyQt6.QtGui import QPalette, QColor
from PyQt6.QtCore import QEvent, Qt
# from call import *
# from destination import *
# from riding import *
# from driving import *
# from end import *
from charge_page import *
from ui_utils import * 


# from_class = uic.loadUiType("/Users/gaji/dev/ros2project/usergui/ui/Main.ui")[0]
from_class = uic.loadUiType("/home/lim/dev_ws/addintexi/UserGUI/ui/0_Main.ui")[0]


class MainWindow(QMainWindow, from_class):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        
        # TCP 클라이언트 생성
        self.socket_manager = SocketManager(ip, port)
        self.start_location = None
        self.end_location = None

        # 아이콘 및 정보창 설정
        self.icon_handler = IconHandler(self, {
            "Icon1": "Info1",
            "Icon2": "Info2",
            "Icon3": "Info3",
            "Icon4": "Info4",
            "Icon5": "Info5",
            "Icon6": "Info6"
        })
                # 아이콘 로드 확인 (디버그)
        for i in range(1, 7):
            icon = getattr(self, f"Icon{i}", None)
            if icon:
                print(f"[DEBUG] 아이콘 {i} 로드됨: {icon}")
            else:
                print(f"[ERROR] 아이콘 {i} 로드 실패")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()
    sys.exit(app.exec())