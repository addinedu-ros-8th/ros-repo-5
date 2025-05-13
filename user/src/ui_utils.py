from PyQt6.QtWidgets import QLineEdit, QWidget
from PyQt6.QtCore import QEvent, Qt
import socket
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QPushButton, QLabel, QMessageBox



# 소켓 연결
ip = "127.0.0.1"  
port = 9000  


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
# 색상 변경 기능 (Hover 및 클릭 이벤트 처리)
# ----------------------------------------------

# ----------------------------------------------
# 아이콘 및 이미지 설정 (모듈화)
# ----------------------------------------------
class IconHandler:
    def __init__(self, main_window, icons_info):
        """
        main_window: QMainWindow - 아이콘을 관리할 메인 윈도우
        icons_info: dict - { "Icon1": "Info1", "Icon2": "Info2", ... }
        """
        self.main_window = main_window
        self.icons_info = icons_info
        self.clicked_icon = None  # 클릭된 아이콘 저장

        # 아이콘 설정
        self.setup_icons()

    def setup_icons(self):
        for icon_name, info_name in self.icons_info.items():
            icon = getattr(self.main_window, icon_name)
            info = getattr(self.main_window, info_name)

            # 기본 스타일 직접 적용
            icon.setStyleSheet(self.default_style())
            icon.setAlignment(Qt.AlignmentFlag.AlignCenter)
            icon.setText("")  # 텍스트 제거
            info.setVisible(False)  # 기본적으로 숨김

            # 마우스 이벤트 직접 연결
            icon.mousePressEvent = lambda event, i=icon: self.set_clicked_icon(i)
            icon.enterEvent = lambda event, i=icon, inf=info: self.on_hover(i, inf)
            icon.leaveEvent = lambda event, i=icon, inf=info: self.on_leave(i, inf)

    def default_style(self):
        return """
        QLineEdit {
            background-color: #E0E0E0;
            color: #000000;
            border: 2px solid #4A4A4A;
            border-radius: 20px;
            min-width: 40px;
            min-height: 40px;
            max-width: 40px;
            max-height: 40px;
            text-align: center;
        }
        """

    def hover_style(self):
        return """
        QLineEdit {
            background-color: #A9D39E;
            color: #000000;
            border: 2px solid #4A4A4A;
            border-radius: 20px;
            min-width: 40px;
            min-height: 40px;
            max-width: 40px;
            max-height: 40px;
            text-align: center;
        }
        """

    def click_style(self):
        return """
        QLineEdit {
            background-color: #76C17E;
            color: #000000;
            border: 2px solid #4A4A4A;
            border-radius: 20px;
            min-width: 40px;
            min-height: 40px;
            max-width: 40px;
            max-height: 40px;
            text-align: center;
        }
        """

    def set_clicked_icon(self, icon):
        # 이전 클릭된 아이콘 스타일 초기화
        if self.clicked_icon and self.clicked_icon != icon:
            self.clicked_icon.setStyleSheet(self.default_style())

        # 새 클릭된 아이콘 스타일 적용
        self.clicked_icon = icon
        self.clicked_icon.setStyleSheet(self.click_style())

    def on_hover(self, icon, info):
        if self.clicked_icon != icon:
            icon.setStyleSheet(self.hover_style())
        info.setVisible(True)

    def on_leave(self, icon, info):
        if self.clicked_icon != icon:
            icon.setStyleSheet(self.default_style())
        info.setVisible(False)



# 이미지 크기 자동 조정 함수 (auto_resize_logo)
def auto_resize_logo(widget, line_edit, image_path):
    """
    QLineEdit의 크기 변경에 따라 로고 이미지 자동 조정
    """
    width = line_edit.width()
    height = line_edit.height()

    line_edit.setStyleSheet(f"""
        QLineEdit {{
            background-image: url({image_path});
            background-repeat: no-repeat;
            background-position: center;
            background-size: contain;  
            border: none;
            padding: 0px;  
        }}
    """)


class ResizeEventFilter(QWidget):
    def __init__(self, line_edit, image_path):
        super().__init__()
        self.line_edit = line_edit
        self.image_path = image_path

    def eventFilter(self, obj, event):
        if event.type() == QEvent.Resize:
            set_image(self.line_edit, self.image_path)
        return super().eventFilter(obj, event)
