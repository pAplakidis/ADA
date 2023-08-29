#!/usr/bin/env python3
import numpy as np

import rospy
from std_msgs.msg import Float64MultiArray

# TODO: smoothen out the path curve, or wait for it to be mean,std,prob instead of xy_points,prob
class LateralPlanner:
  def __init__(self, verbose=False, trajectory_length=200, n_coords=2):
    self.verbose = verbose
    self.trajectory_length = trajectory_length
    self.n_coords = n_coords

    self.subscriber = rospy.Subscriber("/sensor/desire", Float64MultiArray, self._desire_callback)

    self.desire = np.array([1, 0, 0]) # forward as default
    self.xy_path = np.zeros((trajectory_length, n_coords))

  def run_inference_loop(self):
    while not rospy.is_shutdown():
      rospy.spin()

  def _desire_callback(self, msg):
    self.desire = msg.data
    if self.verbose:
      print("[plannerd]: desire ->", self.desire)

  # TODO: publish lateral plan
  def update(self, model_outputs):
    self.xy_path= np.array(model_outputs[:-1]).reshape((self.trajectory_length, self.n_coords))
    self.crossroad = model_outputs[-1]
    if self.verbose:
      print("[plannerd]: model_outputs ->", self.xy_path.shape, self.crossroad)


class LongitudinalPlanner:
  def __init__(self):
    pass

  def update(self):
    pass


class Plannerd:
  def __init__(self, verbose=False):
    self.subscriber = rospy.Subscriber("/model/outputs", Float64MultiArray, self.update_lateral)
    # TODO: publisher -> /planner/trajectory, etc

    self.lateral_planner = LateralPlanner(verbose=verbose)
    self.longitudinal_planner = LongitudinalPlanner()

  def update_lateral(self, msg):
    self.lateral_planner.update(msg.data)
