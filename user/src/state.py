# state.py
from dataclasses import dataclass, field

@dataclass
class RideRequest:
    start_icon: str | None = None     # ex) "icon_A"
    dest_icon:  str | None = None     # ex) "icon_D"

    def is_complete(self) -> bool:
        return self.start_icon is not None and self.dest_icon is not None

    def reset(self):
        self.start_icon = None
        self.dest_icon  = None

# 전역 인스턴스 (싱글톤처럼 사용)
APP_STATE: RideRequest = RideRequest()
