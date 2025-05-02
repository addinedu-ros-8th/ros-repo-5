from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5 import uic
import requests

ESP32_IP = "http://192.168.0.2"  # ESP32에서 출력된 IP 주소

form_class = uic.loadUiType("TrafficLight.ui")[0]

class TrafficLightGUI(QMainWindow, form_class):
    def __init__(self):
        super().__init__()
        self.setupUi(self)

        self.red_btn.clicked.connect(lambda: self.send_command('R'))
        self.yellow_btn.clicked.connect(lambda: self.send_command('Y'))
        self.green_btn.clicked.connect(lambda: self.send_command('G'))

    def send_command(self, command):
        try:
            requests.get(f"{ESP32_IP}/command", params={"cmd": command})
        except Exception as e:
            print("ESP32 연결 실패:", e)

if __name__ == "__main__":
    app = QApplication([])
    gui = TrafficLightGUI()
    gui.show()
    app.exec_()
