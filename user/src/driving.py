from PyQt6.QtWidgets import QWidget, QVBoxLayout, QLabel, QPushButton
from ui_utils import load_state, show_message
from call import CallPage
from destination import destinationPage
from riding import ridingPage
from driving import drivingPage
from end import endPage
from charge_page import EnterChargePage

class DrivingWindow(QWidget):
    def __init__(self, saved_data):
        super().__init__()
        self.saved_data = saved_data
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle("driving Page")
        layout = QVBoxLayout()
        self.label = QLabel(f"driving Page - Departure: {self.saved_data.get('departure')}, Destination: {self.saved_data.get('destination')}")
        self.back_button = QPushButton("Back to Main")
        self.back_button.clicked.connect(self.go_back)

        layout.addWidget(self.label)
        layout.addWidget(self.back_button)
        self.setLayout(layout)

    def go_back(self):
        from main import MainWindow
        self.hide()
        self.main_window = MainWindow()
        self.main_window.show()
