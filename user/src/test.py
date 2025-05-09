import sys
from PyQt6.QtWidgets import QApplication, QMainWindow, QPushButton, QLabel, QVBoxLayout, QWidget
from PyQt6.QtCore import Qt
import socket

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
                print(f"[서버 응답] {response}")
                return response
            except Exception as e:
                print(f"[ERROR] 메시지 전송 실패: {e}")
        else:
            print("[ERROR] 서버에 연결되지 않음")

    def close_connection(self):
        if self.client_socket:
            self.client_socket.close()
            print("[INFO] 서버 연결이 종료되었습니다.")

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("반응형 PyQt6 UI")
        self.setGeometry(200, 200, 400, 300)

        # 소켓 매니저
        self.socket_manager = SocketManager("127.0.0.1", 9000)

        # UI 구성
        self.status_label = QLabel("현재 상태: 대기 중", self)
        self.status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.action_button = QPushButton("호출하기", self)
        self.action_button.clicked.connect(self.handle_action)

        self.cancel_button = QPushButton("취소하기", self)
        self.cancel_button.setVisible(False)
        self.cancel_button.clicked.connect(self.cancel_action)

        layout = QVBoxLayout()
        layout.addWidget(self.status_label)
        layout.addWidget(self.action_button)
        layout.addWidget(self.cancel_button)

        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)

        # 상태 변수
        self.current_state = "ready"

    def handle_action(self):
        if self.current_state == "ready":
            self.status_label.setText("현재 상태: 택시 호출 중")
            self.action_button.setText("승차하기")
            self.cancel_button.setVisible(True)
            self.current_state = "waiting"
            self.socket_manager.send_message("[USER_SEND] 택시 호출")
        
        elif self.current_state == "waiting":
            self.status_label.setText("현재 상태: 탑승 중")
            self.action_button.setText("하차하기")
            self.current_state = "riding"
            self.socket_manager.send_message("[USER_SEND] 승차 확인")
        
        elif self.current_state == "riding":
            self.status_label.setText("현재 상태: 탑승 완료, 결제 대기")
            self.action_button.setText("결제하기")
            self.current_state = "completed"
            self.socket_manager.send_message("[USER_SEND] 하차 확인")
        
        elif self.current_state == "completed":
            self.status_label.setText("현재 상태: 결제 완료")
            self.action_button.setText("호출하기")
            self.cancel_button.setVisible(False)
            self.current_state = "ready"
            self.socket_manager.send_message("[USER_SEND] 결제 완료")

    def cancel_action(self):
        self.status_label.setText("현재 상태: 호출 취소")
        self.action_button.setText("호출하기")
        self.cancel_button.setVisible(False)
        self.current_state = "ready"
        self.socket_manager.send_message("[USER_SEND] 호출 취소")

    def closeEvent(self, event):
        self.socket_manager.close_connection()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
