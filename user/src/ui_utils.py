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
    def __init__(self, main_window, icons_info):
        self.main_window = main_window
        self.icons_info = icons_info
        self.start_icon = None
        self.destination_icon = None
        self.fixed_mode = False  # 아이콘 고정 모드 (CallWindow)
        self.line_edit_handler = LineEditHandler(main_window)

        # 아이콘 설정
        self.setup_icons()

    def setup_icons(self):
        for icon_name, info_name in self.icons_info.items():
            icon = getattr(self.main_window, icon_name, None)
            info = getattr(self.main_window, info_name, None)

            if icon:
                icon.setStyleSheet(self.default_style())
                icon.setAlignment(Qt.AlignmentFlag.AlignCenter)
                icon.setText("")  # 텍스트 제거
                if info:
                    info.setVisible(False)

                if not self.fixed_mode:  # 고정 모드가 아닐 때만 클릭 가능
                    icon.mousePressEvent = lambda event, i=icon_name: self.set_icon(i)
                icon.enterEvent = lambda event, i=icon, inf=info: self.on_hover(i, inf)
                icon.leaveEvent = lambda event, i=icon, inf=info: self.on_leave(i, inf)

            

    def set_icon(self, icon_name):
        """
        아이콘 클릭 시 출발지 또는 목적지로 설정
        """
        icon = getattr(self.main_window, icon_name, None)
        if not icon:
            print(f"[ERROR] 아이콘 {icon_name}을 찾을 수 없습니다.")
            return

        if not self.start_icon:  # 출발지 설정
            self.start_icon = icon
            self.start_icon.setStyleSheet(self.start_style())
            self.line_edit_handler.update_line_edit("start", LocationManager.get_location_names().get(icon_name, ""))
            print(f"[DEBUG] 출발지 설정: {icon_name}")
        elif not self.destination_icon and self.start_icon != icon:  # 목적지 설정
            self.destination_icon = icon
            self.destination_icon.setStyleSheet(self.destination_style())
            self.line_edit_handler.update_line_edit("destination", LocationManager.get_location_names().get(icon_name, ""))
            print(f"[DEBUG] 목적지 설정: {icon_name}")
        else:
            self.reset_icons()
            self.start_icon = icon
            self.start_icon.setStyleSheet(self.start_style())
            print(f"[DEBUG] 출발지 재설정: {icon_name}")


    def reset_icons(self):
        """
        출발지와 목적지 초기화 (상태만 초기화, 스타일은 유지)
        """
        if self.start_icon:
            self.start_icon.setStyleSheet(self.default_style())
        if self.destination_icon:
            self.destination_icon.setStyleSheet(self.default_style())

        self.start_icon = None
        self.destination_icon = None
        self.line_edit_handler.clear_line_edits()

    def get_selected_coordinates(self):
        """
        출발지와 목적지 좌표 반환
        """
        if self.start_icon and self.destination_icon:
            start_coords = self.get_coordinates(self.start_icon)
            destination_coords = self.get_coordinates(self.destination_icon)
            print(f"[DEBUG] 출발지 좌표: {start_coords}, 목적지 좌표: {destination_coords}")
            return start_coords, destination_coords
        
        print("[ERROR] 출발지 또는 목적지가 선택되지 않았습니다.")
        return None, None

    def get_coordinates(self, icon):
        """
        아이콘의 좌표 가져오기 (예제 좌표 설정)
        """
        coordinates = {
            "Icon1": (500, 300),
            "Icon2": (600, 400),
            "Icon3": (700, 500),
            "Icon4": (800, 600),
            "Icon5": (900, 700),
            "Icon6": (1000, 800)
        }

        if icon and icon.objectName() in coordinates:
            return coordinates[icon.objectName()]
        
        print(f"[ERROR] 아이콘 {icon}의 좌표를 찾을 수 없습니다.")
        return None
    
    def on_hover(self, icon, info):
        """
        마우스 오버 시 아이콘 스타일 변경
        """
        if not icon:
            print("[ERROR] on_hover: 아이콘이 지정되지 않았습니다.")
            return
        
        if icon == self.start_icon:
            icon.setStyleSheet(self.start_style())
        elif icon == self.destination_icon:
            icon.setStyleSheet(self.destination_style())
        else:
            icon.setStyleSheet(self.hover_style())

        if info:
            info.setVisible(True)

    def on_leave(self, icon, info):
        """
        마우스 벗어날 때 스타일 복구
        """
        if not icon:
            print("[ERROR] on_leave: 아이콘이 지정되지 않았습니다.")
            return
        
        if icon == self.start_icon:
            icon.setStyleSheet(self.start_style())
        elif icon == self.destination_icon:
            icon.setStyleSheet(self.destination_style())
        else:
            icon.setStyleSheet(self.default_style())

        if info:
            info.setVisible(False)

    def set_fixed_icons(self, start_icon_name, destination_icon_name):
        """
        출발지와 목적지 아이콘을 고정 상태로 설정
        """
        self.fixed_mode = True

        # 출발지 아이콘 설정
        if start_icon_name in self.icons_info:
            self.start_icon = getattr(self.main_window, start_icon_name)
            self.start_icon.setStyleSheet(self.start_style())
            print(f"[DEBUG] 출발지 고정: {start_icon_name}")

        # 목적지 아이콘 설정
        if destination_icon_name in self.icons_info:
            self.destination_icon = getattr(self.main_window, destination_icon_name)
            self.destination_icon.setStyleSheet(self.destination_style())
            print(f"[DEBUG] 목적지 고정: {destination_icon_name}")

        # 고정 상태에서 마우스 클릭 막기
        self.setup_fixed_mode()


    def setup_fixed_mode(self):
        """
        고정 모드에서는 아이콘 클릭 비활성화
        """
        for icon_name in self.icons_info.keys():
            icon = getattr(self.main_window, icon_name, None)
            if icon:
                icon.setEnabled(False)  # 고정 모드에서 클릭 비활성화
                icon.setStyleSheet(self.default_style())

        # 출발지와 목적지는 고정된 스타일 유지
        if self.start_icon:
            self.start_icon.setStyleSheet(self.start_style())
        if self.destination_icon:
            self.destination_icon.setStyleSheet(self.destination_style())

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

    def start_style(self):
        return """
        QLineEdit {
            background-color: #76C17E; /* 초록색 (출발지) */
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

    def destination_style(self):
        return """
        QLineEdit {
            background-color: #E57373; /* 빨간색 (목적지) */
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

#-------------------------------
# LineEdit에 목적지 표시 
#---------------------------------
class LineEditHandler:
    def __init__(self, main_window):
        self.main_window = main_window

    def update_line_edit(self, target, text):
        """
        QLineEdit에 위치 텍스트 업데이트
        """
        if target == "start":
            line_edit = self.main_window.start  # QLineEdit 이름: start
        elif target == "destination":
            line_edit = self.main_window.destination  # QLineEdit 이름: destination
        else:
            return

        if line_edit:
            line_edit.setText(text)
            print(f"[DEBUG] {target} 위치 업데이트: {text}")

    def clear_line_edits(self):
        """
        모든 QLineEdit 초기화
        """
        self.update_line_edit("start", "")
        self.update_line_edit("destination", "")


# LocationManager.py (위치 이름 관리)
class LocationManager:
    @staticmethod
    def get_location_names():
        return {
            "Icon1": "애드인에듀학원",
            "Icon2": "기억할게!술집",
            "Icon3": "내 거친 생각 정신과",
            "Icon4": "불안한 눈빛 안과",
            "Icon5": "신라호텔",
            "Icon6": "월드컵 경기장"
        }



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

