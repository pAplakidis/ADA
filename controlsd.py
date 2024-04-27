import numpy as np
from scipy.optimize import minimize

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


# FIXME: always gives steering_angle = 0.0
class MPCController:
  def __init__(self, horizon=10, dt=0.1):
    self.horizon = horizon  # Prediction horizon
    self.dt = dt  # Time step
    self.state = None  # Current state of the car
    self.path = None  # Target path (list of (x, y) coordinates)
    
  def update_state(self, state):
    self.state = state
  
  def update_path(self, path):
    self.path = path
  
  def cost_function(self, u):
    # Define cost function to be minimized
    cost = 0
    for i in range(self.horizon):
      # Predicted state using the dynamic model
      predicted_state = self.predict_state(u[i])
      # Cost based on distance from path
      cost += np.linalg.norm(predicted_state[:2] - self.path[i])**2
    return cost
  
  def predict_state(self, u):
    # Predict state using dynamic model
    # This is a simple kinematic bicycle model
    x, y, z, theta, v = self.state
    # print("[controlsd]:", x, y, theta, v, u)
    theta += v * np.tan(u) / 2.5 * self.dt  # Steering angle constraint assumed
    x += v * np.cos(theta) * self.dt
    y += v * np.sin(theta) * self.dt
    # print("[controlsd]:", x[0], y[0], theta[0], v)
    
    # TODO: this is a tempfix, need to investigate more
    if isinstance(u, np.ndarray):
      # print("[controlsd]: u is array")
      return np.array([x[0], y[0], theta[0], v])
    else:
      # print("[controlsd]: u is NOT array")
      return np.array([x, y, theta, v])
  
  def control(self):
    # MPC control
    initial_guess = np.zeros(self.horizon)  # Initial guess for steering angles
    bounds = [(-0.5, 0.5)] * self.horizon  # Bounds for steering angle
    constraints = {'type': 'eq', 'fun': lambda u: self.predict_state(u)[-1]}  # State equality constraint
    result = minimize(self.cost_function, initial_guess, bounds=bounds, constraints=constraints)
    return result.x[0]  # Returning only the first steering angle
  

# TODO: we need a localizer first (we temporarily use carla's locations)
class Controlsd:
  def __init__(self, verbose=False):
    self.verbose = verbose

    self.xy_path = np.zeros((PLAN_LENGTH, N_COORDINATES))
    self.planner_subscriber = rospy.Subscriber("/planner/lateral_path", Float64MultiArray, self.trajectory_callback)
    self.car_state_subscriber = rospy.Subscriber("/car/state", Float64MultiArray, self.car_state_callback)
    self.mpc = MPCController(horizon=10, dt=0.1)

  def trajectory_callback(self, msg):
    self.xy_path = np.array(msg.data).reshape((PLAN_LENGTH, N_COORDINATES))

  def car_state_callback(self, msg):
    self.car_state = np.array(msg.data)
    self.update_control()

  def update_control(self):
    self.mpc.update_path(self.xy_path)
    self.mpc.update_state(self.car_state)
    # Get steering angle from MPC controller
    steering_angle = self.mpc.control()
    # Publish control message
    # Example:
    # self.vehicle.apply_control(carla.VehicleControl(steer=steering_angle))
    print("[controlsd]: steering_angle:", steering_angle)
