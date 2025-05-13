import sys
from PyQt6.QtWidgets import QApplication, QMainWindow, QWidget, QPushButton, QLabel, QVBoxLayout, QStackedWidget
from PyQt6 import uic

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("반응형 UI - 상태 유지 + Undo + 충전")
        self.setGeometry(200, 200, 600, 400)
        
        # 상태 저장
        self.current_state = "start"  # 시작 상태
        self.saved_data = {
            "departure": None,
            "destination": None,
            "remaining_money": 10000,
            "taxi_position": None
        }
        self.ui_stack = []  # 상태 스택 (Undo 구현)

        # Main UI 로드
        self.load_main_ui()

    def load_main_ui(self):
        # Main UI 파일 로드
        uic.loadUi("/Users/gaji/dev/ros2project/usergui/ui/Main.ui", self)

        # 버튼 설정
        self.ChargeBtn.clicked.connect(self.load_charge_ui)
        self.UndoBtn.clicked.connect(self.undo_last_action)
        self.CheckBtn.clicked.connect(self.handle_action)

        # 상태 복구 (출발지, 목적지 등)
        if self.saved_data["departure"]:
            self.DepartureLabel.setText(f"출발지: {self.saved_data['departure']}")
        if self.saved_data["destination"]:
            self.DestinationLabel.setText(f"목적지: {self.saved_data['destination']}")

        # 잔액 표시
        self.MoneyLabel.setText(f"남은 금액: {self.saved_data['remaining_money']}원")

    def handle_action(self):
        # 현재 단계에 따라 처리
        if self.current_state == "start":
            self.current_state = "select_departure"
            self.save_current_ui()
            self.saved_data["departure"] = "Witness! 술집"  # 예제 출발지
            self.load_main_ui()

        elif self.current_state == "select_departure":
            self.current_state = "select_destination"
            self.save_current_ui()
            self.saved_data["destination"] = "핑크 우체국"  # 예제 목적지
            self.load_main_ui()

        elif self.current_state == "select_destination":
            self.current_state = "waiting_taxi"
            self.save_current_ui()
            self.load_main_ui()
            self.status_label.setText("택시 호출 중...")

        elif self.current_state == "waiting_taxi":
            self.current_state = "ride"
            self.save_current_ui()
            self.load_main_ui()
            self.status_label.setText("택시에 탑승 중...")

        elif self.current_state == "ride":
            self.current_state = "in_transit"
            self.save_current_ui()
            self.load_main_ui()
            self.status_label.setText("목적지로 이동 중...")

        elif self.current_state == "in_transit":
            self.current_state = "arrived"
            self.save_current_ui()
            self.load_main_ui()
            self.status_label.setText("목적지 도착. 하차 버튼 활성화")

        elif self.current_state == "arrived":
            self.current_state = "start"  # 다시 처음으로
            self.save_current_ui()
            self.saved_data["departure"] = None
            self.saved_data["destination"] = None
            self.load_main_ui()
            self.status_label.setText("처음 화면으로 돌아갑니다.")

    def load_charge_ui(self):
        # 현재 상태를 저장 (충전 후 복구)
        self.save_current_ui()

        # Charge UI 파일 로드
        uic.loadUi("/Users/gaji/dev/ros2project/usergui/ui/Charge.ui", self)
        self.BackBtn.clicked.connect(self.restore_previous_ui)
        self.ChargeConfirmBtn.clicked.connect(self.process_charge)

    def process_charge(self):
        # 임의로 5000원 충전 (예제)
        self.saved_data["remaining_money"] += 5000
        print(f"[INFO] 충전 완료. 남은 금액: {self.saved_data['remaining_money']}원")
        self.restore_previous_ui()

    def restore_previous_ui(self):
        # 이전 UI 복구
        if self.ui_stack:
            last_ui, last_state = self.ui_stack.pop()
            self.current_state = last_state
            self.saved_data.update(last_ui)
            self.load_main_ui()
        else:
            self.load_main_ui()

    def save_current_ui(self):
        # 현재 상태를 저장 (Undo 및 충전 복구용)
        self.ui_stack.append((self.saved_data.copy(), self.current_state))

    def undo_last_action(self):
        if len(self.ui_stack) > 1:
            self.ui_stack.pop()  # 현재 상태를 삭제
            last_ui, last_state = self.ui_stack.pop()  # 직전 상태로 복구
            self.current_state = last_state
            self.saved_data.update(last_ui)
            self.load_main_ui()
        else:
            print("[INFO] 더 이상 이전 단계로 돌아갈 수 없습니다.")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())

# import sys
# from PyQt6.QtWidgets import *
# from PyQt6.QtGui import *
# from PyQt6 import uic
# import socket
# from charge_page import EnterChargePage

# ip = ""  # 서버 IP
# port = 9000          # 서버 포트

# # from_class = uic.loadUiType("/Users/gaji/dev/ros2project/usergui/ui/Main.ui")[0]
# from_class = uic.loadUiType("/home/lim/dev_ws/addintexi/UserGUI/ui/0_Main.ui")[0]


# # ----------------------------------------------
# # TCP 클라이언트 설정 (소켓)
# # ----------------------------------------------
# class SocketManager:
#     def __init__(self, ip, port):
#         self.server_ip = ip
#         self.server_port = port
#         self.client_socket = None
#         self.connect_to_server()

#     def connect_to_server(self):
#         try:
#             self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#             self.client_socket.connect((self.server_ip, self.server_port))
#             print(f"[INFO] 서버 {self.server_ip}:{self.server_port}에 연결됨")
#         except Exception as e:
#             print(f"[ERROR] 서버 연결 실패: {e}")

#     def send_message(self, message: str):
#         if self.client_socket:
#             try:
#                 self.client_socket.send(message.encode('utf-8'))
#                 response = self.client_socket.recv(1024).decode('utf-8')
#                 print(f"[서버 응답] {response}")  # 터미널에 서버 응답 출력
#                 return response
#             except Exception as e:
#                 print(f"[ERROR] 메시지 전송 실패: {e}")
#         else:
#             print("[ERROR] 서버에 연결되지 않음")

#     def close_connection(self):
#         if self.client_socket:
#             self.client_socket.close()
#             print("[INFO] 서버 연결이 종료되었습니다.")

# # ----------------------------------------------
# # 메인 페이지 (GUI)
# # ----------------------------------------------
# class WindowClass(QMainWindow, from_class):
#     def __init__(self):
#         super().__init__()
#         self.setupUi(self)
#         self.socket_manager = SocketManager(ip, port)  # TCP 클라이언트 생성

#         #상태 저장 
#         self.current_state = "start" 
#         self.saved_data = {
#             "departure" : None,
#             "destination" : None,
#             "texi_position": None,
#         }
#         self.ui_stack = []

#         #btn
#         self.ChargeBtn.clicked.connect(self.EnterChargePage)
#         self.CheckBtn.clicked.connect(self.SendMessage)

#         #Save 
        

#     def SendMessage(self):
#         message = "[USER_SEND]테스트메세지"  
#         response = self.socket_manager.send_message(message)
#         if response:
#             print(f"[클라이언트] 서버로부터 응답: {response}")
#         else:
#             print("[클라이언트] 서버로부터 응답 없음")

#     def closeEvent(self, event):
#         # 종료 시 서버 연결 닫기
#         self.socket_manager.close_connection()




# if __name__ == "__main__":
#     app = QApplication(sys.argv)
#     myWindows = WindowClass()
#     myWindows.show()
#     sys.exit(app.exec())
