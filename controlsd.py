import numpy as np

import rospy
from std_msgs.msg import Float64MultiArray

from utils import *

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


# TODO: we need a localizer first (temporarily use carla's locations)
class Controlsd:
  def __init__(self, verbose=False):
    self.verbose = verbose

    self.pid_controller = PIDController()
    self.xy_path = np.zeros((TRAJECTORY_LENGTH, N_COORDINATES))
    self.current_pos = np.array(np.zeros(2, dtype=float))
    self.desired_pos = np.array(np.zeros(2, dtype=float))

    self.path_subscriber = rospy.Subscriber("/planner/lateral_plan")

  def trajectory_callback(self, msg):
    self.xy_path= np.array(msg.data[:-1]).reshape((TRAJECTORY_LENGTH, N_COORDINATES))

  def update_control(self):
    while not rospy.is_shutdown():
      for i in range(TRAJECTORY_LENGTH):
        self.desired_pos = self.xy_path[i]
        self.control_signal = self.pid_controller.update(self.desired_pos, self.current_pos)

        # TODO: update current position

        movement_vector = self.desired_pos - self.current_pos
        self.current_pos += movement_vector * 0.1 # simulate forward movement
