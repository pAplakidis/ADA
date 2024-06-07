import math

# display image shape
IMG_WIDTH = 1164
IMG_HEIGHT = 874

W, H = 224, 224 # model image shape

N_TRAJECTORIES =  5     # 3 paths predicted by model
TRAJECTORY_LENGTH = 200 # 200 points in each path
PLAN_LENGTH = 33
# NOTE: carla needs 200 to get the proper length, but it is too much for the controls
# so we reduce and pseudo-normalize the trajectory length
N_COORDINATES = 2       # x, y

MIN_STEER_VAL = -1.0
MAX_STEER_VAL = 1.0

ONEOVERSQRT2PI = 1.0 / math.sqrt(2 * math.pi)

DESIRE = {0: "forward",
          1: "right",
          2: "left"}

# specifically for desire (0, 1, 2)
def one_hot_encode(desire):
  vector = [0, 0, 0]
  vector[desire] = 1
  return vector
