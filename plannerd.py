#!/usr/bin/env python3
import numpy as np

import rospy
from std_msgs.msg import Float64MultiArray
from utils import *

# TODO: reduce path points to ~33 (easier for controls)
# TODO: smoothen out the path curve, or wait for it to be mean,std,prob instead of xy_points,prob
class LateralPlanner:
  def __init__(self, verbose=False, trajectory_length=TRAJECTORY_LENGTH, n_coords=2, combo=False):
    self.combo = combo
    self.verbose = verbose
    self.trajectory_length = trajectory_length
    self.n_coords = n_coords
    self.plan_length = PLAN_LENGTH

    self.subscriber = rospy.Subscriber("/sensor/desire", Float64MultiArray, self._desire_callback)
    self.publisher = rospy.Publisher("/planner/lateral_path", Float64MultiArray, queue_size=10)

    self.desire = np.array([1, 0, 0]) # forward as default
    self.xy_path = np.zeros((trajectory_length, n_coords))
    self.lateral_plan = np.zeros((self.plan_length, n_coords))

  def run_inference_loop(self):
    while not rospy.is_shutdown():
      rospy.spin()

  def _desire_callback(self, msg):
    self.desire = msg.data
    if self.verbose:
      print("[plannerd]: desire ->", self.desire)

  def update(self, model_outputs):
    if self.combo:
      self.xy_path= np.array(model_outputs[:-1]).reshape((self.trajectory_length, self.n_coords))
      self.crossroad = model_outputs[-1]
    else:
      self.xy_path= np.array(model_outputs).reshape((self.trajectory_length, self.n_coords))
    
    # reduce model trajectory from 200 to 33 2D points
    # TODO: maybe the model itself should output 33 points from the start
    j = 0
    step = self.trajectory_length // self.plan_length
    for i in range(0, self.trajectory_length-step, step):
      self.lateral_plan[j][0] = self.xy_path[i][0]
      self.lateral_plan[j][1] = self.xy_path[i][1]
      j += 1

    if self.verbose:
      if self.combo:
        print("[plannerd]: model_outputs ->", self.xy_path.shape, self.crossroad)
      else:
        print("[plannerd]: lateral_plan ->", self.xy_path.shape)

    if self.combo:
      pass
    else:
      output = self.lateral_plan.flatten()
      output_msg = Float64MultiArray()
      output_msg.data = output
      self.publisher.publish(output_msg)


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
