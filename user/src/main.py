import sys
import requests
import json
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
from icon_coordinates import ICON_COORDINATES 


# from_class = uic.loadUiType("/Users/gaji/dev/ros2project/usergui/ui/Main.ui")[0]
from_class = uic.loadUiType("/home/lim/dev_ws/addintexi/UserGUI/ui/0_Main.ui")[0]

class MainWindow(QMainWindow, from_class):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.left_money_manager = LeftMoneyManager(self.LeftMoney) 

        # 서버 시작 시 모든 택시 초기화
        self.reset_taxi_ids([1, 2])
        
        # REST API 클라이언트 생성
        self.api_manager = RestAPIManager()
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
        }, ICON_COORDINATES)
 
        # 인원 수 설정: QComboBox
        self.comboBox.addItems(["1명", "2명", "3명", "4명"])
        self.comboBox.setCurrentIndex(0)  # 기본값 1명

        # 호출하기 버튼 연결
        self.CheckBtn.clicked.connect(self.send_selected_location)
        self.ChargeBtn.clicked.connect(self.show_charge_page)

    def show_charge_page(self):
        self.charge_window = ChargeWindow()
        self.charge_window.show()
        self.close()
    
    def send_post_request(self, endpoint: str, data: dict):
        """
        서버로 POST 요청 전송
        """
        url = f"http://localhost:8000/reset_taxi"
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

    def reset_taxi_ids(self, taxi_ids):
        """
        서버로 지정된 택시 ID 초기화 요청 (reset_taxi)
        """
        print(f"[INFO] 초기화할 택시 ID 목록: {taxi_ids}")

        for taxi_id in taxi_ids:
            reset_data = {
                "taxi_id": taxi_id
            }

            response = self.send_post_request("/reset_taxi", reset_data)
            if response and response.get("status") == "reset successful":
                print(f"[INFO] 택시 {taxi_id} 초기화 완료.")
            else:
                print(f"[ERROR] 택시 {taxi_id} 초기화에 실패했습니다.")


    def send_selected_location(self):
        # 선택된 아이콘 번호와 좌표 가져오기
        selected_icon_number = self.icon_handler.get_selected_icon_number()
        selected_coordinates = self.icon_handler.get_selected_coordinates()
        
        # 인원 수: 숫자만 추출
        selected_passenger_count_text = self.comboBox.currentText()
        selected_passenger_count = int(''.join(filter(str.isdigit, selected_passenger_count_text)))


        if selected_icon_number and selected_coordinates:
            start_x, start_y, dest_x, dest_y = selected_coordinates
            client_id = "41"  # 고유 클라이언트 ID (동적 생성 가능)

            # JSON 요청 생성
            request_data = {
                "start_x": start_x,
                "start_y": start_y,
                "dest_x": dest_x,
                "dest_y": dest_y,
                "passenger_count": selected_passenger_count,
                "client_id": client_id
            }

            print(f"[DEBUG] 서버로 전송할 JSON: {request_data}")

            # REST API POST 요청 (예: /call_text 엔드포인트)
            response = self.api_manager.send_post_request("/call_taxi", request_data)
            
            # call.py 페이지로 이동 -> 디버깅용 코드
            from call import CallWindow
            self.call_window = CallWindow()
            self.call_window.show()
            self.close()
          
          # 서버 응답 확인
            if response:
                print(f"[INFO] 서버 응답: {response}")
                if response.get("status") == "taxi assigned":
                    print(f"[INFO] 택시 배정 완료 - 택시 ID: {response.get('taxi_id')}")

                    # 선택된 아이콘 번호와 좌표를 애플리케이션 전역에 저장
                    QApplication.instance().setProperty("selected_icon_number", selected_icon_number)
                    QApplication.instance().setProperty("selected_coordinates", selected_coordinates)

                    # # call.py 페이지로 이동
                    # from call import CallWindow
                    # self.call_window = CallWindow()
                    # self.call_window.show()
                    # self.close()
                else:
                    print(f"[ERROR] 서버 오류 또는 응답 처리 문제: {response.get('message')}")
            else:
                print("[ERROR] 서버 응답을 수신할 수 없습니다.")
        else:
            print("[ERROR] 아이콘이 선택되지 않았거나 좌표가 지정되지 않았습니다.")
            


if __name__ == "__main__":
    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()
    sys.exit(app.exec())