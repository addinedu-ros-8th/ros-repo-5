import sys
from PyQt6.QtWidgets import *
from PyQt6.QtGui import *
from PyQt6 import uic
import socket

ip = ""  # 서버 IP
port = 9000          # 서버 포트

from_class = uic.loadUiType("/Users/gaji/dev/ros2project/usergui/ui/Main.ui")[0]

# ----------------------------------------------
# TCP 클라이언트 설정 (소켓)
# ----------------------------------------------
class SocketManager:
    def __init__(self, ip, port):
        self.server_ip = ip
        self.server_port = port
        self.client_socket = None
        self.connect_to_server()

    def connect_to_server(self):
        try:
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_socket.connect((self.server_ip, self.server_port))
            print(f"[INFO] 서버 {self.server_ip}:{self.server_port}에 연결됨")
        except Exception as e:
            print(f"[ERROR] 서버 연결 실패: {e}")

    def send_message(self, message: str):
        if self.client_socket:
            try:
                self.client_socket.send(message.encode('utf-8'))
                response = self.client_socket.recv(1024).decode('utf-8')
                print(f"[서버 응답] {response}")  # 터미널에 서버 응답 출력
                return response
            except Exception as e:
                print(f"[ERROR] 메시지 전송 실패: {e}")
        else:
            print("[ERROR] 서버에 연결되지 않음")

    def close_connection(self):
        if self.client_socket:
            self.client_socket.close()
            print("[INFO] 서버 연결이 종료되었습니다.")

# ----------------------------------------------
# 메인 페이지 (GUI)
# ----------------------------------------------
class WindowClass(QMainWindow, from_class):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.socket_manager = SocketManager(ip, port)  # TCP 클라이언트 생성

        #btn
        self.ChargeBtn.clicked.connect(self.EnterChargePage)
        self.CheckBtn.clicked.connect(self.SendMessage)

    def SendMessage(self):
        message = "[USER_SEND]테스트메세지"  
        response = self.socket_manager.send_message(message)
        if response:
            print(f"[클라이언트] 서버로부터 응답: {response}")
        else:
            print("[클라이언트] 서버로부터 응답 없음")

    def closeEvent(self, event):
        # 종료 시 서버 연결 닫기
        self.socket_manager.close_connection()

    
    def EnterChargePage(self):
        self.charge_window = ChargeWindow()
        self.charge_window.show()

#----------------------------------------------
# 결제 페이지 
#---------------------------------------------- 
 
class ChargeWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        uic.loadUI("/Users/gaji/dev/ros2project/usergui/ui/Charge.ui",self)



if __name__ == "__main__":
    app = QApplication(sys.argv)
    myWindows = WindowClass()
    myWindows.show()
    sys.exit(app.exec())

