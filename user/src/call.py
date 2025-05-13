from PyQt6.QtWidgets import QMainWindow, QApplication, QLabel, QVBoxLayout, QWidget
from PyQt6 import uic

from_class = uic.loadUiType("/home/lim/dev_ws/addintexi/UserGUI/ui/1_call.ui")[0]

class CallWindow(QMainWindow, from_class):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        
        # 애플리케이션 전역에서 선택된 아이콘 번호 및 좌표 가져오기
        selected_icon_number = QApplication.instance().property("selected_icon_number")
        selected_coordinates = QApplication.instance().property("selected_coordinates")
        
        # 선택된 아이콘 정보를 UI에 표시
        if hasattr(self, "selectedIconLabel") and hasattr(self, "selectedCoordinatesLabel"):
            if selected_icon_number and selected_coordinates:
                self.selectedIconLabel.setText(f"선택된 아이콘: {selected_icon_number}")
                self.selectedCoordinatesLabel.setText(f"좌표: {selected_coordinates}")
            else:
                self.selectedIconLabel.setText("선택된 아이콘이 없습니다.")
                self.selectedCoordinatesLabel.setText("좌표가 지정되지 않았습니다.")
        else:
            print("[ERROR] QLabel (selectedIconLabel, selectedCoordinatesLabel) 이 UI 파일에 없습니다.")

        # 다른 페이지로 이동하는 버튼 (예: end 페이지)
        if hasattr(self, "nextButton"):
            self.nextButton.clicked.connect(self.move_to_end)
        else:
            print("[ERROR] QPushButton (nextButton) 이 UI 파일에 없습니다.")

    def move_to_end(self):
        from end import EndWindow
        self.end_window = EndWindow()
        self.end_window.show()
        self.close()
