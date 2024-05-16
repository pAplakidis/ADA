import math

def calc_steering_angle(p0, p1):
  desired_theta =  90 - math.degrees(math.atan2(p1[1] - p0[1], p1[0] - p0[0]))
  print(desired_theta)
  print()

  return desired_theta / 90
