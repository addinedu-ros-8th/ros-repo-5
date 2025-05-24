import sys
import os
import numpy as np
import threading
from PyQt6.QtWidgets import QHeaderView, QApplication, QMainWindow, QGraphicsScene, QGraphicsEllipseItem
from PyQt6.QtGui import QStandardItemModel, QStandardItem, QColor, QPixmap, QImage

from PyQt6 import uic
from PyQt6.QtCore import QLoggingCategory, QDate
from datetime import datetime
import socket
import cv2 
import requests
from qt_material import apply_stylesheet

API_SERVER_IP = "192.168.1.3"
API_SERVER_PORT = 8000

# stderr (경고 메시지) 출력 차단
sys.stderr = open(os.devnull, 'w')
# # 모든 Qt 경고 메시지 차단
QLoggingCategory.setFilterRules("*.debug=false\n*.warning=false\n*.critical=false\n*.fatal=false")

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
    def __init__(self, parent_main_window):
        super().__init__()
        uic.loadUi("log.ui", self)
        self.api_manager = RestAPIManager()
        self.parent_main_window = parent_main_window

        self.undo_btn.clicked.connect(self.undo_click)
        self.search_btn.clicked.connect(self.search)

        # 콤보박스 초기화
        self.ID_combo.addItems(["1", "2"])

        # 기본 날짜 설정
        today = QDate.currentDate()
        self.start_date.setDate(today)
        self.end_date.setDate(today)
        
        self.model = QStandardItemModel()
        self.tableView.setModel(self.model)
        self.tableView.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)


        self.setStyleSheet("""
            QTableView {
                color: black;
                font-size: 12pt;
                font-weight: bold;
            }
        """)

        self.load_today_logs()


    def load_today_logs(self):
        today_str = QDate.currentDate().toString("yyyyMMdd")
        payload = {
            "start_date": today_str,
            "end_date": today_str,
            "vehicle_id": 1
        }

        response = self.api_manager.send_post_request("/driving_log", payload)
        if response and response.get("status") == "ok":
            self.populate_table(response.get("log_info", []))
        else:
            print("[INFO] 서버 연결 실패 또는 빈 응답 → 기본 테이블 비워둠")


    def undo_click(self): 
        from main import AdminMainWindow
        self.hide()
        self.parent_main_window.show()
        
    def search(self):
        start_date = self.start_date.date().toString("yyyyMMdd")
        end_date = self.end_date.date().toString("yyyyMMdd")
        vehicle_id = int(self.ID_combo.currentText())
        
        print("[DEBUG] 서치 요청:", start_date, end_date, vehicle_id)
        
        payload = {
            "start_date": start_date,
            "end_date": end_date,
            "vehicle_id": vehicle_id
        }
        
        response = self.api_manager.send_post_request("/driving_log", payload)
        
        if response and response.get("status") == "ok":
            self.populate_table(response.get("log_info", []))  # ✅ 테이블에만 표시
        else:
            self.model.clear()
            self.model.setHorizontalHeaderLabels(["에러"])
            self.model.appendRow([QStandardItem("❌ 서버 요청 실패 또는 오류 발생.")])
            
    def populate_table(self, log_info_list):
        self.model.clear()
        self.model.setHorizontalHeaderLabels(["시간", "행동", "사유", "차량번호"])
        
        for log in log_info_list:
            time_item = QStandardItem(log.get("create_date", ""))
            action_item = QStandardItem(log.get("event_action", ""))
            reason_item = QStandardItem(log.get("event_reason", ""))
            number_item = QStandardItem(log.get("vehicle_number", ""))
            
            self.model.appendRow([time_item, action_item, reason_item, number_item])

        for col in range(self.model.columnCount()):
            self.tableView.resizeColumnToContents(col)




if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = logWindow()
    apply_stylesheet(app, theme='dark_blue.xml')
    window.show()
    sys.exit(app.exec())
