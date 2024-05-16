#!/usr/bin/env python3
import numpy as np

import matplotlib.pyplot as plt

from mpc import LateralMPC
from simple_steering import calc_steering_angle
from utils import *


if __name__ == "__main__":
  path = np.array([[-1.82172731e-02,  5.05969971e-02],
                  [ 5.38640469e-02,  2.42499328e+00],
                  [ 2.85419106e-01,  4.69025421e+00],        
                  [ 6.88517272e-01,  6.83836508e+00],         
                  [ 1.11132038e+00,  8.87389278e+00],                               
                  [ 1.47387958e+00,  1.09293661e+01],
                  [ 1.95761991e+00,  1.29323750e+01],                                                                                                     
                  [ 2.49950886e+00,  1.48841972e+01],
                  [ 3.08764791e+00,  1.67203979e+01],                            
                  [ 3.72625923e+00,  1.85225105e+01],
                  [ 4.41105700e+00,  2.04153347e+01],                                                                                                     
                  [ 5.16160154e+00,  2.22993126e+01],
                  [ 5.90728903e+00,  2.42056961e+01],                                                                                                     
                  [ 6.69345045e+00,  2.61669159e+01],
                  [ 7.39406061e+00,  2.80057640e+01],                                                                                                     
                  [ 8.16782188e+00,  2.98813534e+01],     
                  [ 8.87676620e+00,  3.17526646e+01],                                                                                                     
                  [ 9.61468124e+00,  3.36077919e+01],
                  [ 1.04203596e+01,  3.56346931e+01],         
                  [ 1.12333651e+01,  3.75859108e+01],
                  [ 1.20322685e+01,  3.95131989e+01],
                  [ 1.27745543e+01,  4.14130096e+01],
                  [ 1.34653568e+01,  4.33286362e+01],                                                                                                     
                  [ 1.40972919e+01,  4.52932320e+01],
                  [ 1.47692986e+01,  4.71867256e+01],
                  [ 1.54341364e+01,  4.88444023e+01],      
                  [ 1.59582243e+01,  5.07380867e+01],
                  [ 1.65172901e+01,  5.25768433e+01],                                
                  [ 1.69378071e+01,  5.43379402e+01],                                                                                                     
                  [ 1.74455109e+01,  5.60472717e+01],                                                                                                     
                  [ 1.76204967e+01,  5.79887466e+01],                                                                                                     
                  [ 1.81198978e+01,  5.97701759e+01],          
                  [ 1.85843124e+01,  6.15711174e+01]])

x = path[:, 0]
y = path[:, 1]

# OLD
# dummy_state = [0.0, 0.0, 0.0, # x,y,z
#                0.0,             # theta
#                1.0]             # velocity
# mpc = LateralMPC(horizon=PLAN_LENGTH, dt=0.1, verbose=True)
# mpc.update_path(path)
# mpc.update_state(dummy_state)
# steering_angle = mpc.control()

# TODO: use MPC
# mpc = LateralMPC(path)
# x0 = np.array([0, 0, 0])
# steering_angle = mpc.control(x0)
# print("[+] Calculated Steering angle:", steering_angle)

# calc_steering_angle(path[0], path[3])
calc_steering_angle(path[0], path[path.shape[0]//2])

# Plot
plt.figure(figsize=(8, 6))
plt.plot(x, y, marker='o', linestyle='-')
plt.title('Plot of Array')
plt.xlabel('X values')
plt.ylabel('Y values')
plt.grid(True)
plt.show()
