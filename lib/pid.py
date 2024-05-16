
class PIDController:
  def __init__(self, kp=1.0, ki=0.1, kd=0.2):
    self._kp = kp
    self._ki = ki
    self._kd = kd
    self.prev_error = 0.0
    self.integral = 0.0

  def update(self, desired_point, current_point):
    error = desired_point - current_point
    self.integral += error
    derivative = error - self.prev_error

    control_signal = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
    self.prev_error = error
    return control_signal
