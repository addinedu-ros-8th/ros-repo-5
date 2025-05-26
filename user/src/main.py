import sys
import requests
import json
from PyQt6.QtWidgets import QApplication
from PyQt6.QtGui import QPalette, QColor
from PyQt6.QtCore import QEvent, Qt, pyqtSlot, QThread
from charge import *
from Icon_coordinates import ICON_COORDINATES
from IconHandler import * 
from Restapi import * 
from Pinkymanager import * 
from UserSession import UserSession
from LeftMoney import * 
from PyQt6.QtCore import QLoggingCategory


# stderr (경고 메시지) 출력 차단
sys.stderr = open(os.devnull, 'w')
# 모든 Qt 경고 메시지 차단
QLoggingCategory.setFilterRules("*.debug=false\n*.warning=false\n*.critical=false\n*.fatal=false")
from_class = uic.loadUiType("ui/0_Main.ui")[0]

class MainWindow(QMainWindow, from_class):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        
        # REST API 클라이언트 생성
        self.api_manager = RestAPIManager()
        self.start_location = None
        self.end_location = None
        self.socket = None
        self.move(1400, 100) 
   


        self.left_money_manager = LeftMoneyManager(self.LeftMoney) 
        
        # 서버 시작 시 모든 택시 초기화
        # self.reset_taxi_ids([1, 2])

        # 아이콘 및 정보창 설정
        self.icon_handler = IconHandler(self, {
            "Icon1": "Info1",
            "Icon2": "Info2",
            "Icon3": "Info3",
            "Icon4": "Info4",
            "Icon5": "Info5",
            "Icon6": "Info6"
        })

 
        # 아이콘 위치 이름
        self.line_edit_handler = LineEditHandler(self)
        self.location_names = LocationManager.get_location_names()
        self.line_edit_handler.clear_line_edits()
 
        # 인원 수 설정: QComboBox
        self.comboBox.addItems(["1명", "2명", "3명", "4명", "5명" ,"6명"])
        self.comboBox.setCurrentIndex(0)  # 기본값 1명

        # 호출하기 버튼 연결
        self.CheckBtn.clicked.connect(self.send_selected_location)
        self.ChargeBtn.clicked.connect(self.show_charge_page)

        

    def update_line_edits(self):
        """
        선택된 아이콘에 따라 LineEdit에 위치 이름 표시
        """
        if self.icon_handler.start_icon:
            start_name = self.location_names.get(self.icon_handler.start_icon.objectName(), "출발지 선택")
            self.line_edit_handler.update_line_edit("start", start_name)
        else:
            self.line_edit_handler.update_line_edit("start", "")

        if self.icon_handler.destination_icon:
            destination_name = self.location_names.get(self.icon_handler.destination_icon.objectName(), "목적지 선택")
            self.line_edit_handler.update_line_edit("destination", destination_name)
        else:
            self.line_edit_handler.update_line_edit("destination", "")

    def show_charge_page(self):
        self.charge_window = ChargeWindow(previous_window="main")
        self.charge_window.show()
        self.hide()


    def reset_taxi_ids(self, taxi_ids):
        """
        서버로 지정된 택시 ID 초기화 요청 (reset_taxi)
        """
        for taxi_id in taxi_ids:
            reset_data = {"vehicle_id": taxi_id}
            response = self.api_manager.send_post_request("/reset_taxi", reset_data)
            if response and response.get("status") == "reset complete":
                print(f"[INFO] 택시 {taxi_id} 초기화 완료.")
            else:
                print(f"[ERROR] 택시 {taxi_id} 초기화에 실패했습니다.")


    def send_selected_location(self):
        self.update_line_edits()

        # 선택된 아이콘 번호와 좌표 가져오기
        start_coords, destination_coords = self.icon_handler.get_selected_coordinates()
        
        # 인원 수: 숫자만 추출
        selected_passenger_count_text = self.comboBox.currentText()
        selected_passenger_count = int(''.join(filter(str.isdigit, selected_passenger_count_text)))


        if start_coords and destination_coords:
            start_x, start_y = start_coords
            dest_x, dest_y = destination_coords
            client_id = UserSession.get_current_user()
            if not client_id: 
                #디버깅
                client_id ="1"

            # JSON 요청 생성
            request_data = {
                "start_x": start_x,
                "start_y": start_y,
                "dest_x": dest_x,
                "dest_y": dest_y,
                "passenger_count": selected_passenger_count,
                "passenger_id": client_id
            }

            print(f"[DEBUG] 서버로 전송할 JSON: {request_data}")

            # REST API POST 요청 
            response = self.api_manager.send_post_request("/call_taxi", request_data)
          
          # 서버 응답 확인
            if response:
                print(f"[INFO] 서버 응답: {response}")
                if response.get("status") == "taxi dispatch":
                    print(f"[INFO] 택시 배정 완료 - 택시 ID: {response.get('vehicle_id')}")
                    taxi_id = response.get("vehicle_id")
                    UserSession.set_taxi_id(taxi_id)
                    
                    # 좌표 저장 (출발지/목적지
                    # self.start_coords = start_coords
                    # self.destination_coords = destination_coords


                    # 선택된 아이콘 번호와 좌표를 애플리케이션 전역에 저장
                    from call import CallWindow
                    self.call_window = CallWindow(
                        self.icon_handler.start_icon.objectName(),
                        self.icon_handler.destination_icon.objectName(),
                    )
                    self.call_window.show()
                    self.close()

                else:
                    print(f"[ERROR] 서버 오류 또는 응답 처리 문제: {response.get('message')}")
                    # print("[ERROR] 디버깅 -> taxi_id =2.")
                    # from call import CallWindow
                    # taxi_id=2
                    # UserSession.set_taxi_id(taxi_id)
                    # self.call_window = CallWindow(
                    #     self.icon_handler.start_icon.objectName(),
                    #     self.icon_handler.destination_icon.objectName(),
                    # )
                    # self.call_window.show()
                    # self.close()
            else:
                print("[ERROR] 서버 응답을 수신할 수 없습니다.")
                # print("[ERROR] 디버깅 -> taxi_id =2.")
                # from call import CallWindow
                # taxi_id=2
                # UserSession.set_taxi_id(taxi_id)
                # self.call_window = CallWindow(
                #     self.icon_handler.start_icon.objectName(),
                #     self.icon_handler.destination_icon.objectName(),
                # )
                # self.call_window.show()
                # self.close()
        else:
            QMessageBox.warning(self, "에러", "출발지와 목적지를 선택해주세요.")

    def get_ui_path(self, ui_file):
        base_dir = os.path.dirname(os.path.abspath(__file__))
        return os.path.join(base_dir, "ui", ui_file)
            

if __name__ == "__main__":
    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()
    sys.exit(app.exec())