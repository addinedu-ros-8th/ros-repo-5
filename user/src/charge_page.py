import os
import requests
from PyQt6.QtWidgets import QMainWindow, QApplication, QMessageBox
from PyQt6.QtGui import QIcon
from PyQt6.QtCore import QSize
from PyQt6 import uic
from ui_utils import * 

class ChargeWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        uic.loadUi(self.get_ui_path("Charge.ui"), self)
        self.left_money_manager = LeftMoneyManager(self.LeftMoney)
        
        # 초기화 로직 (버튼, 금액 설정 등)
        self.setup_ui()
        self.update_balance()  # 최신 잔액 업데이트

    def setup_ui(self):
        """
        ChargeWindow의 UI 초기화 (버튼, 금액 설정)
        """
        # 뒤로가기 버튼 설정
        self.Undobtn.setIcon(QIcon(self.get_icon_path("Undo.png")))
        self.Undobtn.setIconSize(QSize(50, 50))
        self.Undobtn.clicked.connect(self.return_to_main)

        # 초기화 버튼 (Reset) 설정
        self.ResetBtn.clicked.connect(self.left_money_manager.update_balance)
        self.CheckBtn.clicked.connect(self.request_charge)

    def update_balance(self):
        """
        서버에서 최신 잔액을 가져와 표시
        """
        response = self.send_post_request("/get_balance", {})
        if response and "balance" in response:
            current_balance = response["balance"]
            self.LeftMoney.setText(str(current_balance))
            print(f"[INFO] 최신 잔액: {current_balance}")
        else:
            self.LeftMoney.setText("잔액 불러오기 실패")
            print("[ERROR] 잔액 정보를 불러올 수 없습니다.")

    def request_charge(self):
        """
        서버로 충전 요청 (CheckBtn)
        """
        charge_amount = 5000  # 충전 금액 (필요시 사용자 입력으로 변경 가능)
        response = self.send_post_request("/charge", {"amount": charge_amount})

        if response and response.get("status") == "success":
            QMessageBox.information(self, "충전 완료", f"{charge_amount}원이 충전되었습니다.")
            self.update_balance()
        else:
            QMessageBox.warning(self, "충전 실패", "충전 중 오류가 발생했습니다.")

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

    def return_to_main(self):
        """
        MainWindow로 돌아가기
        """
        from main import MainWindow  # 순환 참조 방지
        self.main_window = MainWindow()
        self.main_window.show()
        self.close()

    def get_ui_path(self, ui_file):
        """
        UI 파일 경로를 동적으로 설정 (상대 경로 사용)
        """
        base_dir = os.path.dirname(os.path.abspath(__file__))
        return os.path.join(base_dir, "ui", ui_file)

    def get_icon_path(self, icon_file):
        """
        아이콘 파일 경로를 동적으로 설정 (상대 경로 사용)
        """
        base_dir = os.path.dirname(os.path.abspath(__file__))
        return os.path.join(base_dir, "data/icon", icon_file)


if __name__ == "__main__":
    app = QApplication([])
    window = ChargeWindow()
    window.show()
    app.exec()

