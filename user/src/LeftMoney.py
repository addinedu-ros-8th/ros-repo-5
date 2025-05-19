from PyQt6.QtWidgets import QLabel, QLineEdit, QMessageBox
from Restapi import RestAPIManager
from UserSession import UserSession

class LeftMoneyManager:
    def __init__(self, display_widget: QLineEdit):
        """
        잔액 관리 매니저 (QLineEdit 사용)
        :param display_widget: 잔액을 표시할 QLineEdit (LeftMoney)
        """
        self.display_widget = display_widget
        self.api_manager = RestAPIManager()
        self.current_balance = 0  # 기본 잔액 초기화
        self.update_balance()     # 서버에서 최신 잔액 불러오기

    def update_balance(self):
        """
        서버에서 최신 잔액을 불러와 QLineEdit에 표시
        """
        client_id = UserSession.get_current_user()
        if not client_id:
            self.display_widget.setText("로그인 필요")
            return

        response = self.api_manager.send_post_request("/get_balance", {"passenger_id": client_id})
        if response and "remaining amount" in response:
            self.current_balance = response["remaining amount"]
            self.display_widget.setText(str(self.current_balance))
            print(f"[INFO] 최신 잔액: {self.current_balance}")
        else:
            self.display_widget.setText("잔액 불러오기 실패")