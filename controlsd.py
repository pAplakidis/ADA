import numpy as np

import rospy
from std_msgs.msg import Float64MultiArray, Float32

from lib.utils import *
from lib.mpc import LateralMPC
from lib.simple_steering import calc_steering_angle


# TODO: we need a localizer first (we temporarily use carla's poses)
class Controlsd:
  def __init__(self, verbose=False):
    self.verbose = verbose
    self.K = 0.1

    self.xy_path = np.zeros((PLAN_LENGTH, N_COORDINATES))
    # self.mpc = LateralMPC(horizon=PLAN_LENGTH, dt=0.1, verbose=verbose)

    self.planner_subscriber = rospy.Subscriber("/planner/lateral_path", Float64MultiArray, self.trajectory_callback)
    self.car_state_subscriber = rospy.Subscriber("/car/state", Float64MultiArray, self.car_state_callback)
    self.publisher = rospy.Publisher("/controlsd/controls", Float32, queue_size=10)

  def trajectory_callback(self, msg):
    self.xy_path = np.array(msg.data).reshape((PLAN_LENGTH, N_COORDINATES))

  def car_state_callback(self, msg):
    self.car_state = np.array(msg.data)
    self.update_control()

  def update_control(self):
    # self.mpc.update_path(self.xy_path)
    # self.mpc.update_state(self.car_state)
    # steering_angle = self.mpc.control()

    # steering_angle = self.K * calc_steering_angle(self.xy_path[0], self.xy_path[self.xy_path.shape[0]//2])
    # FIXME: oversteers
    steering_angle = self.K * calc_steering_angle(self.xy_path[0], self.xy_path[10])

    if self.verbose:
      print("[controlsd]: steering_angle:", steering_angle)

    # Publish control message
    output_msg = Float32()
    output_msg.data = steering_angle
    self.publisher.publish(output_msg)
