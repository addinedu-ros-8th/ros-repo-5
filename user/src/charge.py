import os
import json
from PyQt6.QtWidgets import QMainWindow, QApplication, QMessageBox
from PyQt6.QtGui import QIcon
from PyQt6.QtCore import QSize
from PyQt6 import uic
from charge import *
from Icon_coordinates import ICON_COORDINATES
from IconHandler import * 
from LeftMoney import * 
from Restapi import * 
from Pinkymanager import * 
import sys
from PyQt6.QtCore import QLoggingCategory


# stderr (경고 메시지) 출력 차단
sys.stderr = open(os.devnull, 'w')

# 모든 Qt 경고 메시지 차단
QLoggingCategory.setFilterRules("*.debug=false\n*.warning=false\n*.critical=false\n*.fatal=false")



class ChargeWindow(QMainWindow):
    def __init__(self, previous_window, start_icon=None, destination_icon=None):
        super().__init__()
        uic.loadUi(self.get_ui_path("Charge.ui"), self)
        self.left_money_manager = LeftMoneyManager(self.LeftMoney)
        
        # REST API 클라이언트 생성
        self.api_manager = RestAPIManager()

        
        # 상태 저장 (이전 페이지 정보)
        self.previous_window = previous_window
        self.start_icon = start_icon
        self.destination_icon = destination_icon
        
        # 초기화 로직 (버튼, 금액 설정 등)
        self.setup_ui()
        self.update_balance()  # 최신 잔액 업데이트

    def setup_ui(self):
        """
        ChargeWindow의 UI 초기화 (버튼, 금액 설정)
        """
        # 뒤로가기 버튼 설정
        self.Undobtn.clicked.connect(self.return_to_previous)

        # 초기화 버튼 (Reset) 설정
        self.ResetBtn.clicked.connect(self.update_balance)
        self.CheckBtn.clicked.connect(self.request_charge)

    def update_balance(self):
        """
        서버에서 최신 잔액을 가져와 표시
        """
        client_id = UserSession.get_current_user()
        if not client_id:
            QMessageBox.warning(self, "에러", "로그인되지 않았습니다.")
            return

        # JSON 요청 생성
        request_data = {"passenger_id": client_id}
        response = self.api_manager.send_post_request("/get_balance", request_data)

        if response and "remaining amount" in response:
            current_balance = response["remaining amount"]
            self.LeftMoney.setText(str(current_balance))
            print(f"[INFO] 최신 잔액: {current_balance}")
        else:
            self.LeftMoney.setText("잔액 불러오기 실패")
            print("[ERROR] 잔액 정보를 불러올 수 없습니다.")

    def request_charge(self):
        """
        서버로 충전 요청 (CheckBtn)
        """
        charge_amount = self.charge.text()  # LineEdit에서 입력받기
        if not charge_amount.isdigit() or int(charge_amount) <= 0:
            QMessageBox.warning(self, "에러", "유효한 충전 금액을 입력하세요.")
            return

        client_id = UserSession.get_current_user()
        if not client_id:
            QMessageBox.warning(self, "에러", "로그인되지 않았습니다.")
            return

        # JSON 요청 생성
        request_data = {
            "passenger_id": client_id,
            "pay_amount": int(charge_amount)
        }
        response = self.api_manager.send_post_request("/charge", request_data)

        if response and response.get("status") == "ok":
            QMessageBox.information(self, "충전 완료", f"{charge_amount}원이 충전되었습니다.")
            self.update_balance()
        else:
            QMessageBox.warning(self, "충전 실패", "충전 중 오류가 발생했습니다.")

    def return_to_previous(self):
        """
        이전 페이지로 돌아가기
        """
        if self.previous_window == "main":
            from main import MainWindow
            self.main_window = MainWindow()
            self.main_window.show()
        else:
            # 이전 페이지로 상태 유지하여 돌아가기
            self.previous_window.show()
            if hasattr(self.previous_window, "restore_state"):
                self.previous_window.restore_state(self.start_icon, self.destination_icon)
        
        self.close()

    def get_ui_path(self, ui_file):
        """
        UI 파일 경로 설정 (동적)
        """
        base_dir = os.path.dirname(os.path.abspath(__file__))
        return os.path.join(base_dir, "ui", ui_file)