from PyQt6.QtWidgets import *
from PyQt6 import uic
import sys
from PyQt6.QtGui import QIcon
from PyQt6.QtCore import QSize
from ui_utils import *

# from call import CallPage
# from destination import destinationPage
# from riding import ridingPage
# from driving import drivingPage
# from end import endPage


TEST_MONEY = 10000

def EnterChargePage(self):
    self.charge_window = ChargeWindow()
    self.charge_window.show()


class ChargeWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        # UI 파일 로드
        uic.loadUi("/home/lim/dev_ws/addintexi/UserGUI/ui/Charge.ui", self)

        # 이미지 설정
        icon = QIcon("/home/lim/dev_ws/addintexi/UserGUI/data/icon/Undo.png")
        self.Undobtn.setIcon(icon)
        self.Undobtn.setIconSize(QSize(50, 50))
        
        self.nowmoney.setText(str(TEST_MONEY)) 

        self.Undobtn.clicked.connect(self.Comeback_Home)

    def Comeback_Home(self):
        uic.loadUi("/home/lim/dev_ws/addintexi/UserGUI/ui/0_Main.ui", self)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = ChargeWindow()
    window.show()
    sys.exit(app.exec())
