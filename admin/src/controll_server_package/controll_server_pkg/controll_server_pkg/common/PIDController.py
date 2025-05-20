import time

class PID:
    def __init__(self):
        self.P = 0.0
        self.I = 0.0
        self.D = 0.0
        self.max_state = 0.0
        self.min_state = 0.0
        self.pre_state = 0.0
        self.dt = 0.0
        self.integrated_state = 0.0
        self.pre_time = time.time()

    def update(self, state):
        current_time = time.time()
        self.dt = current_time - self.pre_time

        if self.dt <= 0.0:
            self.dt = 1e-6  # 너무 작거나 0일 경우 보정

        # 미분 항
        state_D = (state - self.pre_state) / self.dt

        # 적분 항
        self.integrated_state += state * self.dt

        # PID 계산
        out = self.P * state + self.I * self.integrated_state + self.D * state_D

        # Saturation (출력 제한)
        if out > self.max_state:
            out = self.max_state
        elif out < -self.max_state:
            out = -self.max_state
        elif 0 < out < self.min_state:
            out = self.min_state
        elif -self.min_state < out < 0:
            out = -self.min_state

        # 상태 업데이트
        self.pre_state = state
        self.pre_time = current_time

        return out
