import sys
import os
import numpy as np
import threading
from PyQt6.QtWidgets import  QApplication, QMainWindow, QGraphicsScene, QGraphicsEllipseItem
from PyQt6.QtGui import QColor, QPixmap, QImage
from PyQt6 import uic
from qt_material import apply_stylesheet
from PyQt6.QtCore import QLoggingCategory, QDate
import socket
import cv2 
import requests


API_SERVER_IP = "192.168.1.4"
API_SERVER_PORT = 8000

# stderr (경고 메시지) 출력 차단
# sys.stderr = open(os.devnull, 'w')
# # 모든 Qt 경고 메시지 차단
# QLoggingCategory.setFilterRules("*.debug=false\n*.warning=false\n*.critical=false\n*.fatal=false")

class RestAPIManager:
    def __init__(self):
        self.server_ip = API_SERVER_IP
        self.server_port = API_SERVER_PORT
        self.base_url = f"http://{self.server_ip}:{self.server_port}"
        self.timeout = 5  # 5초 타임아웃
        self.session = requests.Session()
        self.session.headers.update({"Content-Type": "application/json"})
        self.max_retries = 3  # 자동 재시도 횟수

    def send_post_request(self, endpoint: str, data: dict):
        """
        서버로 POST 요청을 전송하고 응답을 반환.
        """       
        url = f"{self.base_url}{endpoint}"
        print(f"[INFO] POST 요청 URL: {url}")
        print(f"[INFO] 요청 데이터: {data}")

        try:
            response = self.session.post(url, json=data, timeout=self.timeout)
            if response.status_code == 200:
                print(f"[INFO] 서버 응답: {response.json()}")
                return response.json()
            else:
                print(f"[ERROR] 서버 오류: {response.status_code} - {response.text}")
                return None
        except requests.RequestException as e:
            print(f"[ERROR] 서버 연결 실패: {e}")

        print("[CRITICAL] 서버에 연결할 수 없습니다.")
        return None


    def send_get_request(self, endpoint: str):
        """
        서버로 GET 요청을 전송하고 응답을 반환.
        """
        url = f"{self.base_url}{endpoint}"
        print(f"[INFO] GET 요청 URL: {url}")

        for attempt in range(self.max_retries):
            try:
                response = self.session.get(url, timeout=self.timeout)
                if response.status_code == 200:
                    print(f"[INFO] 서버 응답: {response.json()}")
                    return response.json()
                else:
                    print(f"[ERROR] 서버 오류: {response.status_code} - {response.text}")
                    return None
            except requests.RequestException as e:
                print(f"[ERROR] 서버 연결 실패")

        print("[CRITICAL] 서버에 연결할 수 없습니다.")
        return None

class logWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        uic.loadUi("log.ui", self)
        self.api_manager = RestAPIManager()

        self.undo_btn.clicked.connect(self.on_groupbox_click)
        self.search_btn.clicked.connect(self.search)

        # 콤보박스 초기화
        self.combo_vehicle_id.addItems(["1", "2"])

        # 기본 날짜 설정
        today = QDate.currentDate()
        self.date_start.setDate(today)
        self.date_end.setDate(today)

    def on_groupbox_click(self): 
        from main import AdminMainWindow
        self.main_window = AdminMainWindow()
        self.main_window.show()
        self.close()

    def search(self):
        start_date = self.date_start.date().toString("yyyyMMdd")
        end_date = self.date_end.date().toString("yyyyMMdd")
        vehicle_id = int(self.combo_vehicle_id.currentText())

        print("[DEBUG] 서치 요청:", start_date, end_date, vehicle_id)

        payload = {
            "start_date": start_date,
            "end_date": end_date,
            "vehicle_id": vehicle_id
        }

        response = self.api_manager.send_post_request("/driving_log", payload)
        if response and response.get("status") == "ok":
            log_list = response.get("log_info", [])
            if not log_list:
                self.text_log_result.setText("📭 조회된 주행 로그가 없습니다.")
            else:
                log_text = "\n\n".join([str(log) for log in log_list])
                self.text_log_result.setText(log_text)
        else:
            self.text_log_result.setText("❌ 서버 요청 실패 또는 오류 발생.")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = logWindow()
    apply_stylesheet(app, theme='dark_blue.xml')
    window.show()
    sys.exit(app.exec())
