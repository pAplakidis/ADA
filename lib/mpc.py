import numpy as np
from scipy.optimize import minimize

# FIXME: always gives steering_angle = 0.0
# class LateralMPC:
#   def __init__(self, horizon=10, dt=0.1, verbose=False):
#     self.horizon = horizon  # Prediction horizon
#     self.dt = dt  # Time step
#     self.verbose = verbose

#     self.state = None  # Current state of the car
#     self.path = None  # Target path (list of (x, y) coordinates)
    
#   def update_state(self, state):
#     self.state = state
  
#   def update_path(self, path):
#     self.path = path
  
#   def cost_function(self, u):
#     # Define cost function to be minimized
#     cost = 0
#     for i in range(self.horizon):
#       # Predicted state using the dynamic model
#       predicted_state = self.predict_state(u[i])
#       # Cost based on distance from path
#       cost += np.linalg.norm(predicted_state[:2] - self.path[i])**2
#     return cost
  
#   def predict_state(self, u):
#     # Predict state using dynamic model
#     # This is a simple kinematic bicycle model
#     x, y, z, theta, v = self.state

#     if self.verbose:
#       # print("[mpc]: path:", self.path)
#       print("[mpc]: state before:", x, y, theta, v, u)

#     theta += v * np.tan(u) / 2.5 * self.dt  # Steering angle constraint assumed
#     x += v * np.cos(theta) * self.dt
#     y += v * np.sin(theta) * self.dt

#     if self.verbose:
#       if isinstance(u, np.ndarray):
#         print("[mpc]: state after:", x[0], y[0], theta[0], v)
#       else:
#         print("[mpc]: state after:", x, y, theta, v)
    
#     # TODO: this is a tempfix, need to investigate more
#     if isinstance(u, np.ndarray):
#       return np.array([x[0], y[0], theta[0], v])
#     else:
#       return np.array([x, y, theta, v])
  
#   def control(self):
#     # MPC control
#     initial_guess = np.zeros(self.horizon)  # Initial guess for steering angles
#     bounds = [(-0.5, 0.5)] * self.horizon  # Bounds for steering angle
#     # bounds = [(-1.0, 1.0)] * self.horizon  # Bounds for steering angle
#     constraints = {'type': 'eq', 'fun': lambda u: self.predict_state(u)[-1]}  # State equality constraint
#     result = minimize(self.cost_function, initial_guess, bounds=bounds, constraints=constraints)

#     if self.verbose:
#       print("[mpc]: result:", result)

#     return result.x[0]  # Returning only the first steering angle

# class LateralMPC:
#   def __init__(self, path, N=10, dt=0.1, delta_min=-1.0, delta_max=1.0):
#     self.path = path
#     self.N = N  # Prediction horizon
#     self.dt = dt  # Time step
#     self.delta_min = delta_min  # Minimum steering angle
#     self.delta_max = delta_max  # Maximum steering angle

#   def cost_function(self, delta, x0, x_ref):
#     """
#     Cost function to minimize. Here, we can use a simple quadratic cost.
#     """
#     cost = 0
#     for i in range(self.N):
#         x_pred = self.system_dynamics(x0, delta, i)
#         x_pred = x_pred[:len(x_ref[i])]
#         cost += np.linalg.norm(x_pred - x_ref[i])**2
#     return cost

#   def system_dynamics(self, x0, delta, t):
#     """
#     System dynamics. Here, we can use a simple kinematic bicycle model.
#     """
#     # Assuming a simple kinematic bicycle model
#     L = 2.5  # Length of the vehicle
#     v = 10  # Constant velocity

#     # Extracting current state
#     x, y, theta = x0[0], x0[1], x0[2]

#     # Calculate next state using kinematic bicycle model
#     x += v * np.cos(theta) * self.dt
#     y += v * np.sin(theta) * self.dt
#     theta += (v / L * np.tan(delta) * self.dt)[0]

#     return np.array([x, y, theta])

#   def control(self, x0):
#     """
#     Final control function that returns the calculated steering angle from δmin to δmax.
#     """
#     # Initial guess for the steering angle
#     delta_guess = 0.0

#     # Generate reference trajectory using path
#     x_ref = [self.path[t] for t in range(min(self.N, len(self.path)))]

#     # Minimize the cost function using optimization
#     result = minimize(self.cost_function, delta_guess, args=(x0, x_ref), bounds=[(self.delta_min, self.delta_max)])

#     return result.x[0]

class LateralMPC:
  def __init__(self, N=12, dt=0.1, verbose=False):
    self.N = N    # horizon
    self.dt = dt  # delta

    self.verbose = verbose

  # quadratic cost.
  def cost_function(self, delta, x0, x_ref):
    cost = 0
    for i in range(self.N):
      x_pred = self.system_dynamics(x0, delta, i)
      x_pred = x_pred[:len(x_ref[i])]
      cost += np.linalg.norm(x_pred - x_ref[i])**2
    return cost

  def control(self, x0, path):
    pass