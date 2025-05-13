from PyQt6.QtWidgets import QLineEdit, QWidget
from PyQt6.QtCore import QEvent, Qt
import requests
import json
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QPushButton, QLabel, QMessageBox

API_SERVER_IP = "192.168.0.32"
API_SERVER_PORT = 8000


# # ----------------------------------------------
# # RestAPIManage
# # ----------------------------------------------
class RestAPIManager:
    def __init__(self):
        self.server_ip = API_SERVER_IP
        self.server_port = API_SERVER_PORT
        self.base_url = f"http://{self.server_ip}:{self.server_port}"

    def send_post_request(self, endpoint: str, data: dict):
        """
        서버로 POST 요청을 전송하고 응답을 반환.
        """
        url = f"{self.base_url}{endpoint}"
        print(f"[INFO] POST 요청 URL: {url}")
        print(f"[INFO] 요청 데이터: {data}")

        try:
            response = requests.post(url, json=data)
            if response.status_code == 200:
                print(f"[INFO] 서버 응답: {response.json()}")
                return response.json()
            else:
                print(f"[ERROR] 서버 오류: {response.status_code} - {response.text}")
                return None
        except requests.RequestException as e:
            print(f"[ERROR] 서버와 연결할 수 없습니다: {e}")
            return None

    def send_get_request(self, endpoint: str):
        """
        서버로 GET 요청을 전송하고 응답을 반환.
        """
        url = f"{self.base_url}{endpoint}"
        print(f"[INFO] GET 요청 URL: {url}")

        try:
            response = requests.get(url)
            if response.status_code == 200:
                print(f"[INFO] 서버 응답: {response.json()}")
                return response.json()
            else:
                print(f"[ERROR] 서버 오류: {response.status_code} - {response.text}")
                return None
        except requests.RequestException as e:
            print(f"[ERROR] 서버와 연결할 수 없습니다: {e}")
            return None

# # ----------------------------------------------
# # 아이콘 및 이미지 설정 모듈화
# # ----------------------------------------------

class IconHandler:
    def __init__(self, main_window, icons_info, icon_coordinates):
        """
        main_window: QMainWindow - 아이콘을 관리할 메인 윈도우
        icons_info: dict - { "Icon1": "Info1", "Icon2": "Info2", ... }
        icon_coordinates: dict - { "Icon1": (start_x, start_y, dest_x, dest_y), ... }
        """
        self.main_window = main_window
        self.icons_info = icons_info
        self.icon_coordinates = icon_coordinates  # 아이콘별 좌표 저장
        self.clicked_icon = None  # 클릭된 아이콘 저장
        self.selected_icon_number = None  # 선택된 아이콘 번호 저장
        self.selected_coordinates = None  # 선택된 아이콘의 좌표 저장

        # 아이콘 설정
        self.setup_icons()

    def setup_icons(self):
        for icon_number, (icon_name, info_name) in enumerate(self.icons_info.items(), start=1):
            icon = getattr(self.main_window, icon_name)
            info = getattr(self.main_window, info_name)

            # 기본 스타일 직접 적용
            icon.setStyleSheet(self.default_style())
            icon.setAlignment(Qt.AlignmentFlag.AlignCenter)
            icon.setText("")  # 텍스트 제거
            info.setVisible(False)  # 기본적으로 숨김

            # 마우스 이벤트 직접 연결
            icon.mousePressEvent = lambda event, num=icon_number: self.set_clicked_icon(num)
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

    def set_clicked_icon(self, icon_number):
        # 이전 클릭된 아이콘 초기화
        if self.clicked_icon:
            self.clicked_icon.setStyleSheet(self.default_style())

        # 선택된 아이콘 번호 및 좌표 설정
        self.selected_icon_number = icon_number
        icon_name = f"Icon{icon_number}"
        self.clicked_icon = getattr(self.main_window, icon_name)
        self.clicked_icon.setStyleSheet(self.click_style())

        # 좌표값 설정
        if icon_name in self.icon_coordinates:
            self.selected_coordinates = self.icon_coordinates[icon_name]
            print(f"[DEBUG] 선택된 아이콘: {icon_name} (번호: {icon_number}), 좌표: {self.selected_coordinates}")
        else:
            self.selected_coordinates = None
            print(f"[WARNING] 선택된 아이콘에 좌표값이 지정되지 않았습니다.")

    def get_selected_icon_number(self):
        return self.selected_icon_number

    def get_selected_coordinates(self):
        return self.selected_coordinates

    def on_hover(self, icon, info):
        if self.clicked_icon != icon:
            icon.setStyleSheet(self.hover_style())
        info.setVisible(True)

    def on_leave(self, icon, info):
        if self.clicked_icon != icon:
            icon.setStyleSheet(self.default_style())
        info.setVisible(False)

#--------------------------------------------------------
# 남은 돈 보여주기
#--------------------------------------------------------

class LeftMoneyManager:
    def __init__(self, display_widget):
        """
        잔액 관리 매니저
        :param display_widget: 잔액을 표시할 QLineEdit 또는 QLabel
        """
        self.display_widget = display_widget
        self.current_balance = 0  # 기본 잔액 초기화
        self.update_balance()     # 서버에서 최신 잔액 불러오기

    def update_balance(self):
        """
        서버에서 최신 잔액을 불러와 표시
        """
        response = self.send_post_request("/get_balance", {})
        if response and "balance" in response:
            self.current_balance = response["balance"]
            self.display_widget.setText(str(self.current_balance))
            print(f"[INFO] 최신 잔액: {self.current_balance}")
        else:
            self.display_widget.setText("잔액 불러오기 실패")
            print("[ERROR] 잔액 정보를 불러올 수 없습니다.")

    def charge_balance(self, amount):
        """
        서버에 충전 요청 (금액 추가)
        :param amount: 충전할 금액
        """
        if amount <= 0:
            print("[ERROR] 유효하지 않은 충전 금액")
            return

        response = self.send_post_request("/charge", {"amount": amount})
        if response and response.get("status") == "success":
            self.current_balance = response["balance"]
            self.display_widget.setText(str(self.current_balance))
            print(f"[INFO] {amount}원 충전 완료. 잔액: {self.current_balance}")
        else:
            print("[ERROR] 충전 실패 - 서버 오류")

    def send_post_request(self, endpoint: str, data: dict):
        """
        서버로 POST 요청 전송
        """
        url = f"http://localhost:8000{endpoint}"
        print(f"[INFO] POST 요청 URL: {url}")
        
        try:
            response = requests.post(url, json=data)
            if response.status_code == 200:
                print(f"[INFO] 서버 응답: {response.json()}")
                return response.json()
            else:
                print(f"[ERROR] 서버 오류: {response.status_code} - {response.text}")
        except requests.RequestException as e:
            print(f"[ERROR] 서버와 연결할 수 없습니다: {e}")
        return None

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

