"""This file implements a PID and PI controll class"""


class PI:
    def __init__(self, kp, ki, min_out, max_out):
        """
        :param kp: Proportional gain
        :param ki: Integral gain
        :param min_out: Minimum output
        :param max_out: Maximum output
        """
        self.kp = kp
        self.ki = ki
        self.min_out = min_out
        self.max_out = max_out
        self.integral = 0

    def compute(self, error, dt):
        self.integral += error * dt
        output = self.kp * error + self.ki * self.integral
        if output > self.max_out:
            output = self.max_out
        elif output < self.min_out:
            output = self.min_out
        return output


class PID:
    def __init__(self, kp, ki, kd, min_out, max_out):
        """
        :param kp: Proportional gain
        :param ki: Integral gain
        :param kd: Derivative gain
        :param min_out: Minimum output
        :param max_out: Maximum output
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min_out = min_out
        self.max_out = max_out
        self.integral = 0
        self.last_error = 0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.last_error) / dt
        self.last_error = error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        if output > self.max_out:
            output = self.max_out
        elif output < self.min_out:
            output = self.min_out
        return output
