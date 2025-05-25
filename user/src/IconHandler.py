from PyQt6.QtWidgets import QLineEdit, QWidget, QApplication
from PyQt6.QtCore import QEvent, Qt, pyqtSignal, QObject, QTimer, QObject, QPointF
import requests
import json
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QPushButton, QLabel, QMessageBox


# # ----------------------------------------------
# # 아이콘 및 이미지 설정 모듈화
# # ----------------------------------------------


class IconHandler:
    iconSelected = pyqtSignal(str) 

    def __init__(self, main_window, icons_info):
        self.main_window = main_window
        self.icons_info = icons_info
        self.start_icon = None
        self.destination_icon = None
        self.fixed_mode = False  # 아이콘 고정 모드 (CallWindow)
        self.selected_count = 0
        self.line_edit_handler = LineEditHandler(main_window)

        # 아이콘 설정
        self.setup_icons()

from PyQt6.QtCore import Qt, pyqtSignal, QObject

class IconHandler(QObject):
    # 두 개가 모두 선택되면 "complete" 를 내보내서 MainWindow 가 다음 페이지로 넘어가도록
    iconSelected = pyqtSignal(str)          # "A", "B", … / 또는 "complete"

    def __init__(self, main_window, icons_info):
        super().__init__(main_window)       # QObject 초기화
        self.main_window = main_window
        self.icons_info = icons_info

        self.start_icon = None
        self.destination_icon = None
        self.fixed_mode = False             # 잠금 상태
        self.line_edit_handler = LineEditHandler(main_window)

        self.setup_icons()

    # ------------------------------------------------------------------
    # 1. 아이콘 위젯에 클릭/호버 이벤트 연결
    # ------------------------------------------------------------------
    def setup_icons(self):
        for icon_name, info_name in self.icons_info.items():
            icon = getattr(self.main_window, icon_name, None)
            info = getattr(self.main_window, info_name, None)

            if not icon:
                continue

            icon.setStyleSheet(self.default_style())
            icon.setAlignment(Qt.AlignmentFlag.AlignCenter)
            icon.setText("")                     # 텍스트 제거

            if info:
                info.setVisible(False)

            # 클릭                       ### ← 변경: 이벤트 연결은 항상 해 두고,
            icon.mousePressEvent = lambda e, n=icon_name: self.set_icon(n)
            # 호버
            icon.enterEvent      = lambda e, i=icon, inf=info: self.on_hover(i, inf)
            icon.leaveEvent      = lambda e, i=icon, inf=info: self.on_leave(i, inf)

    # ------------------------------------------------------------------
    # 2. 클릭 처리 – 두 개 선택되면 잠금
    # ------------------------------------------------------------------
    def set_icon(self, icon_name: str):
        """아이콘을 클릭했을 때 호출된다."""
        if self.fixed_mode:               # 잠금되면 무시
            return

        icon = getattr(self.main_window, icon_name, None)
        if not icon:
            return

        # 첫 번째 클릭 → 출발지
        if not self.start_icon:
            self.start_icon = icon
            self.start_icon.setStyleSheet(self.start_style())
            self.line_edit_handler.update_line_edit("start",
                LocationManager.get_location_names().get(icon_name, ""))
            self.iconSelected.emit(icon_name)   # 필요하면 위치이름 대신 아이콘ID

        # 두 번째 클릭이면서 동일 아이콘이 아닐 때 → 목적지
        elif not self.destination_icon and icon != self.start_icon:
            self.destination_icon = icon
            self.destination_icon.setStyleSheet(self.destination_style())
            self.line_edit_handler.update_line_edit("destination",
                LocationManager.get_location_names().get(icon_name, ""))
            self.iconSelected.emit(icon_name)

            # 둘 다 정해졌으니 잠금
            self.lock_icons()
            self.iconSelected.emit("complete")  # MainWindow 에게 “완료” 알림

        # 이미 두 개가 고정되기 전이라면(= start는 있고 dest는 없음) 클릭을 바꿀 기회 줌
        else:
            self.reset_icons()                  # 전부 초기화
            self.set_icon(icon_name)            # 재귀 호출로 처음부터 다시 선택

    # ------------------------------------------------------------------
    # 3. 잠금 / 잠금 해제
    # ------------------------------------------------------------------
    def lock_icons(self):
        """두 아이콘이 정해진 뒤 호출 → 클릭 불가 & 색 유지"""
        self.fixed_mode = True
        for icon_name in self.icons_info:
            icon = getattr(self.main_window, icon_name, None)
            if icon:
                icon.setEnabled(False)          # 클릭 차단

    def unlock_icons(self):
        """EndWindow → MainWindow 로 돌아올 때 호출 → 초기화 & 재사용"""
        self.fixed_mode = False
        self.reset_icons(clear_styles=True)
        for icon_name in self.icons_info:
            icon = getattr(self.main_window, icon_name, None)
            if icon:
                icon.setEnabled(True)           # 다시 클릭 가능

    # ------------------------------------------------------------------
    # 4. 선택 초기화(내부용)
    # ------------------------------------------------------------------
    def reset_icons(self, *, clear_styles=False):
        """start / dest 정보만 지우고, 필요하면 스타일도 기본으로"""
        if clear_styles:
            if self.start_icon:
                self.start_icon.setStyleSheet(self.default_style())
            if self.destination_icon:
                self.destination_icon.setStyleSheet(self.default_style())

        self.start_icon = None
        self.destination_icon = None
        self.line_edit_handler.clear_line_edits()



    def reset_icons(self):
        """
        출발지와 목적지 초기화 (상태만 초기화, 스타일은 유지)
        """
        if self.start_icon:
            self.start_icon.setStyleSheet(self.default_style())
        if self.destination_icon:
            self.destination_icon.setStyleSheet(self.default_style())

        self.start_icon = None
        self.destination_icon = None
        self.line_edit_handler.clear_line_edits()

    def get_selected_coordinates(self):
        """
        출발지와 목적지 좌표 반환
        """
        if self.start_icon and self.destination_icon:
            start_coords = self.get_coordinates(self.start_icon)
            destination_coords = self.get_coordinates(self.destination_icon)
            print(f"[DEBUG] 출발지 좌표: {start_coords}, 목적지 좌표: {destination_coords}")
            return start_coords, destination_coords
        return None, None

    def get_coordinates(self, icon):
        """
        아이콘의 좌표 가져오기 (예제 좌표 설정)
        """
        coordinates = {
            "Icon1": (0.262, 0.161),
            "Icon2": (0.17, 0.217),
            "Icon3": (-0.015, -0.078),
            "Icon4": (-0.045, 0.021),
            "Icon5": (0.08, 0.069),
            "Icon6": (0.1, 0.097)
        }

        if icon and icon.objectName() in coordinates:
            return coordinates[icon.objectName()]
        
        return None
    
    def on_hover(self, icon, info):
        """
        마우스 오버 시 아이콘 스타일 변경
        """
        if not icon:
            return
        
        if icon == self.start_icon:
            icon.setStyleSheet(self.start_style())
        elif icon == self.destination_icon:
            icon.setStyleSheet(self.destination_style())
        else:
            icon.setStyleSheet(self.hover_style())

        if info:
            info.setVisible(True)

    def on_leave(self, icon, info):
        """
        마우스 벗어날 때 스타일 복구
        """
        if not icon:
            return
        
        if icon == self.start_icon:
            icon.setStyleSheet(self.start_style())
        elif icon == self.destination_icon:
            icon.setStyleSheet(self.destination_style())
        else:
            icon.setStyleSheet(self.default_style())

        if info:
            info.setVisible(False)

    def set_fixed_icons(self, start_icon_name, destination_icon_name):
        """
        출발지와 목적지 아이콘을 고정 상태로 설정
        """
        self.fixed_mode = True

        # 출발지 아이콘 설정
        if start_icon_name in self.icons_info:
            self.start_icon = getattr(self.main_window, start_icon_name)
            self.start_icon.setStyleSheet(self.start_style())

        # 목적지 아이콘 설정
        if destination_icon_name in self.icons_info:
            self.destination_icon = getattr(self.main_window, destination_icon_name)
            self.destination_icon.setStyleSheet(self.destination_style())

        # 고정 상태에서 마우스 클릭 막기
        self.setup_fixed_mode()


    def setup_fixed_mode(self):
        """
        고정 모드에서는 아이콘 클릭 비활성화
        """
        for icon_name in self.icons_info.keys():
            icon = getattr(self.main_window, icon_name, None)
            if icon:
                icon.setEnabled(False)  # 고정 모드에서 클릭 비활성화
                icon.setStyleSheet(self.default_style())

        # 출발지와 목적지는 고정된 스타일 유지
        if self.start_icon:
            self.start_icon.setStyleSheet(self.start_style())
        if self.destination_icon:
            self.destination_icon.setStyleSheet(self.destination_style())

    def default_style(self):
        return """
        QLineEdit {
            background-color: #E0E0E0;
            color: #000000;
            border: 2px solid #4A4A4A;
            border-radius: 20px;
            min-width: 40px;
            min-height: 40px;
            max-width: 40px;
            max-height: 40px;
            text-align: center;
        }
        """

    def start_style(self):
        return """
        QLineEdit {
            background-color: #76C17E; /* 초록색 (출발지) */
            color: #000000;
            border: 2px solid #4A4A4A;
            border-radius: 20px;
            min-width: 40px;
            min-height: 40px;
            max-width: 40px;
            max-height: 40px;
            text-align: center;
        }
        """

    def destination_style(self):
        return """
        QLineEdit {
            background-color: #E57373; /* 빨간색 (목적지) */
            color: #000000;
            border: 2px solid #4A4A4A;
            border-radius: 20px;
            min-width: 40px;
            min-height: 40px;
            max-width: 40px;
            max-height: 40px;
            text-align: center;
        }
        """

    def hover_style(self):
        return """
        QLineEdit {
            background-color: #A9D39E;
            color: #000000;
            border: 2px solid #4A4A4A;
            border-radius: 20px;
            min-width: 40px;
            min-height: 40px;
            max-width: 40px;
            max-height: 40px;
            text-align: center;
        }
        """

#-------------------------------
# LineEdit에 목적지 표시 
#---------------------------------
class LineEditHandler:
    def __init__(self, main_window):
        self.main_window = main_window

    def update_line_edit(self, target, text):
        """
        QLineEdit에 위치 텍스트 업데이트
        """
        if target == "start":
            line_edit = self.main_window.start  # QLineEdit 이름: start
        elif target == "destination":
            line_edit = self.main_window.destination  # QLineEdit 이름: destination
        else:
            return

        if line_edit:
            line_edit.setText(text)

    def clear_line_edits(self):
        """
        모든 QLineEdit 초기화
        """
        self.update_line_edit("start", "")
        self.update_line_edit("destination", "")


# LocationManager.py (위치 이름 관리)
class LocationManager:
    @staticmethod
    def get_location_names():
        return {
            "Icon1": "애드인에듀학원",
            "Icon2": "기억할게!술집",
            "Icon3": "내 거친 생각 정신과",
            "Icon4": "불안한 눈빛 안과",
            "Icon5": "신라호텔",
            "Icon6": "월드컵 경기장"
        }
