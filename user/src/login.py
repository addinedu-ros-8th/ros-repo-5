import os
from PyQt6.QtWidgets import QMainWindow, QApplication, QMessageBox
from PyQt6 import uic
from charge import *
from Icon_coordinates import ICON_COORDINATES
from IconHandler import * 
from Restapi import * 
from Pinkymanager import * 
import sys
from UserSession import UserSession
from PyQt6.QtCore import QLoggingCategory

# stderr (경고 메시지) 출력 차단
sys.stderr = open(os.devnull, 'w')
# 모든 Qt 경고 메시지 차단
QLoggingCategory.setFilterRules("*.debug=false\n*.warning=false\n*.critical=false\n*.fatal=false")


class LoginWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        uic.loadUi("ui/Login.ui", self)
        self.LogIn.clicked.connect(self.login)
        self.move(1400, 100) 
        
        self.pw.setEchoMode(QLineEdit.EchoMode.Password)
        
        # REST API 클라이언트 생성
        self.api_manager = RestAPIManager()

    def login(self):
        user_id = self.id.text()
        password = self.pw.text()

        # 서버에 로그인 요청
        success = self.api_manager.login(user_id, password)

        if success:
            self.go_to_main()
        else:
            QMessageBox.warning(self, "로그인 실패", "아이디 또는 비밀번호가 잘못되었습니다.")

    def go_to_main(self):
        from main import MainWindow
        self.main_window = MainWindow()
        self.main_window.show()
        self.close()

    def get_ui_path(self, ui_file):
        base_dir = os.path.dirname(os.path.abspath(__file__))
        return os.path.join(base_dir, "ui", ui_file)


if __name__ == "__main__":
    app = QApplication(sys.argv)

    # 앱 종료 시 자동 로그아웃
    def on_exit():
        print("[INFO] Application 종료 - 자동 로그아웃 처리")
        UserSession.logout()

    app.aboutToQuit.connect(on_exit)  # 종료 이벤트 연결

    window = LoginWindow()
    window.show()
    sys.exit(app.exec())