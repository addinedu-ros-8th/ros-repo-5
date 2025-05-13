import sys
from PyQt6.QtWidgets import QApplication, QMainWindow, QPushButton, QLabel, QVBoxLayout, QWidget
from PyQt6.QtCore import pyqtSignal
from ui_utils import load_state, show_message
from call import CallPage
from destination import destinationPage
from riding import ridingPage
from driving import drivingPage
from end import endPage
from charge_page import EnterChargePage


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Multi-Page UI with State Management")

        # 상태 저장 (모든 페이지에서 공유)
        self.saved_data = {
            "page_1_data": None,
            "page_2_data": None,
            "page_3_data": None,
            "page_4_data": None,
            "page_5_data": None,
            "page_6_data": None
        }

        # 페이지 스택 (뒤로가기)
        self.ui_stack = []

        # 첫 페이지 로드
        self.current_page = None
        self.show_page(Page1)

    def show_page(self, page_class):
        if self.current_page:
            # 현재 페이지 스택에 저장
            self.ui_stack.append((self.current_page, self.saved_data.copy()))
            self.current_page.close()

        # 새로운 페이지 로드
        self.current_page = page_class(self.saved_data)
        self.current_page.back_signal.connect(self.on_back)
        self.current_page.show()

    def on_back(self, updated_data):
        if self.ui_stack:
            # 이전 상태 복원
            self.saved_data.update(updated_data)
            previous_page, previous_data = self.ui_stack.pop()
            self.saved_data = previous_data
            self.current_page.close()
            self.current_page = previous_page(self.saved_data)
            self.current_page.back_signal.connect(self.on_back)
            self.current_page.show()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
