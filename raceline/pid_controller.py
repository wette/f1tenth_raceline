class PIDController():
    def __init__(self, kp : float, ki : float, kd : float, history_length : int):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.error_history = [0.0]
        self.history_length = history_length

        self.last_error = 0.0

    def update(self, e):
        cumulated_error = sum(self.error_history) / len(self.error_history)

        u = self.kp * e + self.ki * cumulated_error - self.kd * (e - self.last_error)

        self.error_history.append(e)
        self.error_history = self.error_history[-self.history_length:]
        
        self.last_error = e

        return u