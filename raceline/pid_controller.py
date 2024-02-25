class PIDController():
    def __init__(self, kp : float, ki : float, kd : float):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.cumulated_error = 0.0
        self.last_error = 0.0

    def update(self, e):
        u = self.kp * e + self.ki * self.cumulated_error - self.kd * (e - self.last_error)

        self.cumulated_error += e
        self.last_error = e

        return u