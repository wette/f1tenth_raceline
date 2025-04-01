import math
class PIDController():
    def __init__(self, kp : float, ki : float, kd : float, history_length:int = 10):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.cumulated_error = 0.0
        self.last_error = 0.0

        self.error_history : list[float] = []
        self.history_length = history_length

    def update(self, e: float):
        u = self.kp * e + self.ki * self.cumulated_error - self.kd * (e - self.last_error)
        
        self.last_error = e

        self.error_history.append(e)
        self.error_history = self.error_history[-self.history_length:]
        self.cumulated_error = sum(self.error_history) / len(self.error_history)

        return u