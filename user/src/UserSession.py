
from PyQt6.QtWidgets import QApplication
from PyQt6.QtGui import QPalette, QColor
from PyQt6.QtCore import QEvent, Qt, pyqtSlot, QThread


#-----------------------------------------------
#LoginHandler
#-----------------------------------------------


class UserSession:
    _current_user = None
    _taxi_id = None  

    @classmethod
    def login(cls, user_id):
        cls._current_user = user_id
        print(f"[INFO] 로그인 성공 - 현재 사용자: {cls._current_user}")

    @classmethod
    def logout(cls):
        if cls._current_user is not None:
            print(f"[INFO] 로그아웃 - 사용자: {cls._current_user}")
        cls._current_user = None

    @classmethod
    def get_current_user(cls):
        return cls._current_user
    
    @classmethod
    def set_taxi_id(cls, taxi_id):
        cls._taxi_id = taxi_id

    @classmethod
    def get_taxi_id(cls):
        return cls._taxi_id
    
    @classmethod
    def is_logged_in(cls):
        return cls._current_user is not None