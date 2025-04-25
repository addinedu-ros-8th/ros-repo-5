import sys
from PyQt6.QtWidgets import *
from PyQt6.QtGui import *
from PyQt6 import uic
from PyQt6.QtCore import QRegularExpression, QTimer, QThread, pyqtSignal
import socket
from struct import Struct
import time

ip = "192.168.0.56"
port = 9000

from_class = uic.loadUiType("Main.ui")[0]

class WindowClass(QMainWindow, from_class):
    def __init__(self):
        super().__init__()
        self.setupUi(self)

        self.setWindowTitle("Addin Taxi")
        self.connect()
        self.send_command(10, 30, 40, 50, 5)

    def connect(self):
        self.sock = socket.socket()
        self.sock.connect((ip, int(port)))

        self.connected = True
        self.struct_format = Struct('@iiiii')  # 보낼 데이터 (x1, y1, x2, y2, count)
        self.result_format = Struct('@i')      # 받을 데이터 (결과 int 하나)

    def recv_all(self, size):
        data = b''
        while len(data) < size:
            packet = self.sock.recv(size - len(data))
            if not packet:
                return None
            data += packet
        return data

    def send_command(self, x1, y1, x2, y2, person_count):
        if self.connected:
            data = self.struct_format.pack(x1, y1, x2, y2, person_count)
            self.sock.send(data)

            recv_data = self.recv_all(self.result_format.size)
            if recv_data:
                result = self.result_format.unpack(recv_data)[0]
                print("결과:", result)
            else:
                print("수신 실패")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    myWindows = WindowClass()
    myWindows.show()
    sys.exit(app.exec())