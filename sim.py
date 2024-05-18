#!/usr/bin/env python3
import threading
import os
import sys
import random
import math
import time
import cv2
import numpy as np
from multiprocessing import Process, Queue

"""
try:
    sys.path.append(glob.glob('/opt/carla-simulator/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'linux-x86_64'))[0])
except IndexError as e:
  print("index error", e)
"""

import carla

import rospy
from std_msgs.msg import Float64MultiArray, Float32

from lib.utils import *
from sim_config import *
from carla_world_settings import *

from rendererd import Rendererd
from camerad import Camerad
from modeld import Modeld
from plannerd import Plannerd
from controlsd import Controlsd

# EXAMPLE RUN: MAP=2 ./sim.py

MODE = os.getenv("MODE")
modes = {0: "Single-Task Model", 1: "Multi-Task Model"}
if MODE:
  print("Using")

# TODO: make these a class? + cleanup
vehicle = None

def controls_callback(msg):
  controls = msg.data
  throttle_input = 0.5
  steering_angle = controls
  print("[sim]: controls = ", controls)

  if vehicle is not None and not CARLA_AUTOPILOT:
    vehicle.apply_control(carla.VehicleControl(throttle=throttle_input, steer=steering_angle))


# init daemons
print("[+] Initializing ROS")
rospy.init_node("ros_integration")
desire_pm = rospy.Publisher("/sensor/desire", Float64MultiArray, queue_size=10)
car_state_pm = rospy.Publisher("/car/state", Float64MultiArray, queue_size=10)
controls_sb = rospy.Subscriber("/controlsd/controls", Float32, controls_callback)

print("[+] Initializing Rendererd")
rendererd = Rendererd()

print("[+] Initializing Modeld")
modeld = Modeld(mode=0)

print("[+] Initializing Plannerd")
plannerd = Plannerd()

print("[+] Initializing Controlsd")
controlsd = Controlsd(verbose=True)

# handle output directories
map_idx = os.getenv("MAP")
if map_idx == None:
  print("Using default map Town01")
  print("""
  List of Towns:
  Town01
  Town02
  Town03
  Town04
  Town05
  Town06
  Town07
  Town08
  Town09
  Town10
  Town11
  Town12
  Just give MAP=<index of town>
  """)
  map_idx = 0
else:
  map_idx = int(map_idx) - 1
curr_map = maps[map_idx]

WEATHER = weather["CloudyNoon"]

# actors lists
actor_list = []
vehicles = []
walkers = []


class Car:
  def __init__(self):
    # TODO: properly init car components
    #self.front_camera = np.zeros((IMG_HEIGHT, IMG_WIDTH, 3))
    self.front_camera = None
    self.pose = None
    self.gyro = None

    self.location = None
    self.theta = None
    self.velocity = None
    self.state = []

  def process_img(self, img):
    img = np.array(img.raw_data)
    img = img.reshape((IMG_HEIGHT, IMG_WIDTH, 4))
    img = img[:, :, :3]
    model_img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    self.front_camera = img

  # TODO: for other sensors publish messages to /sensor/... here
  def process_imu(self, imu):
    self.bearing_deg = math.degrees(imu.compass)
    self.acceleration = [imu.accelerometer.x, imu.accelerometer.y, imu.accelerometer.z]
    self.gyro = [imu.gyroscope.x, imu.gyroscope.y, imu.gyroscope.z]

  def process_gps(self, gps):
    # TODO: update this
    self.gps_location = {
      "timestamp": int(time.time() * 1000),
      "accuracy": 1.0,
      "speed_accuracy": 0.1,
      "bearing_accuracy_deg": 0.1,
      "bearing_deg": self.bearing_deg,
      "latitude": gps.latitude,
      "longitude": gps.longitude,
      "altitude": gps.altitude,
      "speed": 0,
    }

  def update_state(self, location, theta, velocity):
    self.location = location
    self.theta = theta
    self.velocity = velocity
    self.state = [location, theta, velocity]
    print("[=>] Car State:", self.state)


# TODO: using carla's locations instead of GNSS, visual odometry, etc is just a temp hack
# def carla_main(q: Queue):
def carla_main():
  global vehicle

  #fourcc = cv2.CV_FOURCC(*'MP4V')
  location, rotation, desire = None, None, None

  # setup
  client = carla.Client('localhost', 2000)
  client.set_timeout(2.0)  # seconds
  #world = client.get_world()
  print("Loading Map:", curr_map)
  world = client.load_world(curr_map)
  world.set_weather(WEATHER)
  bp_lib = world.get_blueprint_library()
  car = Car()
  
  # spawn traffic
  traffic_manager = client.get_trafficmanager()
  traffic_manager.set_global_distance_to_leading_vehicle(2.5)
  traffic_manager.set_respawn_dormant_vehicles(True)
  print("Spawned Traffic Manager")

  # Spawn traffic/vehicles
  if TRAFFIC:
    for i in range(N_VEHICLES):
      bp = random.choice(bp_lib.filter('vehicle'))
      try:
        spawn_point = random.choice(world.get_map().get_spawn_points())
        vhcl = world.spawn_actor(bp, spawn_point)
        vhcl.set_autopilot(True)
        #traffic_manager.update_vehicle_lights(vhcl, True)
        vehicles.append(vhcl)
      except:
        continue
    print(len(vehicles), "Vehicles Spawned")

  # spawn pedestrians
  if PEDESTRIANS:
    blueprintWalkers = bp_lib.filter("walker.pedestrian.*")

    spawn_points = []
    for i in range(N_PEDESTRIANS):
      spawn_point = carla.Transform()
      spawn_point.location = world.get_random_location_from_navigation()
      if spawn_point.location != None:
        spawn_points.append(spawn_point)

    batch = []
    for spawn_point in spawn_points:
      bp_walker = random.choice(blueprintWalkers)
      batch.append(carla.command.SpawnActor(bp_walker, spawn_point))
    
    results = client.apply_batch_sync(batch, True)
    for i in range(len(results)):
      if results[i].error:
        print(results[i].error)
      else:
        walkers.append({"id": results[i].actor_id})

    batch = []
    walker_controller_bp = bp_lib.find("controller.ai.walker")
    for i in range(len(walkers)):
      batch.append(carla.command.SpawnActor(walker_controller_bp, carla.Transform(), walkers[i]["id"]))
    
    results = client.apply_batch_sync(batch, True)
    for i in range(len(results)):
      if results[i].error:
        print(results[i].error)
      else:
        walkers[i]["con"] = results[i].actor_id

    for i in range(len(walkers)):
      actor_list.append(walkers[i])

  # spawn main car
  vehicle_bp = bp_lib.filter('vehicle.tesla.*')[1]
  spawn_point = random.choice(world.get_map().get_spawn_points())
  vehicle = world.spawn_actor(vehicle_bp, spawn_point)
  print("Main Car Spawned")

  # make tires less slippery
  wheel_control = carla.WheelPhysicsControl(tire_friction=5)
  physics_control = vehicle.get_physics_control()
  physics_control.mass = 2326
  physics_control.wheels = [wheel_control]*4
  physics_control.torque_curve = [[20.0, 500.0], [5000.0, 500.0]]
  physics_control.gear_switch_time = 0.0
  vehicle.apply_physics_control(physics_control)
  vehicle.set_autopilot(CARLA_AUTOPILOT)
  actor_list.append(vehicle)

  # spawn camera
  camera_bp = bp_lib.find('sensor.camera.rgb')
  camera_bp.set_attribute('image_size_x', f'{IMG_WIDTH}')
  camera_bp.set_attribute('image_size_y', f'{IMG_HEIGHT}')
  camera_bp.set_attribute('fov', '70')
  camera_bp.set_attribute('sensor_tick', '0.05')
  spawn_point  = carla.Transform(carla.Location(x=0.8, z=1.13))  # dashcam location
  #spawn_point  = carla.Transform(carla.Location(x=-8., z=2.)) # NOTE: third-person camera view for debugging
  camera = world.spawn_actor(camera_bp, spawn_point, attach_to=vehicle)
  actor_list.append(camera_bp)
  # camera.listen(lambda img: car.process_img(img))
  _camerad = Camerad(car)
  camera.listen(_camerad.camera_callback)
  print("Camera Spawned")

  # spawn IMU
  imu_bp = bp_lib.find("sensor.other.imu")
  imu = world.spawn_actor(imu_bp, spawn_point, attach_to=vehicle)
  imu.listen(lambda imu: car.process_imu(imu))
  print("IMU Spawned")

  # spawn GPS
  gps_bp = bp_lib.find("sensor.other.gnss")
  gps = world.spawn_actor(gps_bp, spawn_point, attach_to=vehicle)
  gps.listen(lambda gps: car.process_gps(gps))
  print("GPS Spawned")

  # Enable synchronous mode
  settings = world.get_settings()
  settings.synchronous_mode = True 
  settings.no_rendering_mode = False
  settings.fixed_delta_seconds = 0.05
  world.apply_settings(settings)
  traffic_manager.set_synchronous_mode(True)

  world.tick()

  # mainloop
  frame_id = 0
  start_time = time.time()
  try:
    print("Starting mainloop ...")
    for _ in range(20):
      world.tick()
    while True:
      if CARLA_AUTOPILOT:
        traffic_manager.update_vehicle_lights(vehicle, True)
        traffic_manager.auto_lane_change(vehicle, True)
      if TRAFFIC:
        for i in range(len(vehicles)):
          traffic_manager.update_vehicle_lights(vehicles[i], True)
          traffic_manager.auto_lane_change(vehicles[i], True)

      # get steering angle
      control = vehicle.get_control()
      steering_angle = control.steer  # TODO: do more with this

      lx,ly,lz = vehicle.get_location().x, vehicle.get_location().y ,vehicle.get_location().z
      #rot = vehicle.get_transform().get_forward_vector()
      #rx, ry, rz = rot.x, rot.y, rot.z
      location = [lx, ly, lz]

      if car.gyro is not None:
        rotation = [car.gyro[0], car.gyro[1], car.gyro[2]]  # NOTE: IMU data could be noisy

      if car.front_camera is not None:
        rendererd._render_img(car.front_camera)
        print("[+] Frame: ", frame_id, "=>", car.front_camera.shape)

        print("[+] Car Location: (x y z)=(", location, ")")
        print("[+] Car Rotation: (x y z)=(", rotation, ")")
        print("[->] IMU DATA => acceleration", car.acceleration, " : gyroscope", car.gyro)
        print("[->] GNSS DATA => latitude", car.gps_location['latitude'],
              " : longtitude", car.gps_location['longitude'],
              " : altitude", car.gps_location['altitude'])
        print("[+] Steering Angle: ", steering_angle)

        # get s state => DESIRE
        light_state = vehicle.get_light_state()
        right_ = bool(light_state & (0x1 << RIGHT__POS))
        left_ = bool(light_state & (0x1 << LEFT__POS))
        print("Blinkers (l/r):", left_, right_)
        if right_ and not left_:
          desire = 1  # desire: right
        elif not right_ and left_:
          desire = 2  # desire: left
        else:
          desire = 0  # desire: forward
        print("DESIRE:", desire, "=>", DESIRE[desire])
        desire = one_hot_encode(desire)
        desire_pm.publish(Float64MultiArray(data=desire))

        frame_id += 1
        curr_time = time.time() - start_time
        print("Current Time: %.2fs"%curr_time)
        print()

      # update car state (location, theta, velocity)
      theta = rotation[2]

      transform = vehicle.get_transform()
      velocity = vehicle.get_velocity()
      forward_vector = transform.get_forward_vector()
      # FIXME: gives negative value while the car is moving forward
      forward_velocity = velocity.x * forward_vector.x + velocity.y * forward_vector.y + velocity.z * forward_vector.z

      car.update_state(location, theta, forward_velocity)
      # print("[sim]: car.state:", car.state)
      state_msg = Float64MultiArray()
      state_msg.data = [car.location[0], car.location[1], car.location[2], car.theta, car.velocity]
      car_state_pm.publish(state_msg)

      world.tick()
  except KeyboardInterrupt:
    print("[~] Stopped recording")


if __name__ == '__main__':
  print("Starting CARLA")
  try:
    carla_main()
  except RuntimeError as re:
    print("[-]", re)
    print("Restarting ...")
  finally:
    print("destroying all actors")
    for a in actor_list:
      a.destroy()
    for v in vehicles:
      v.destroy()
    for w in walkers:
      w.destroy()
    cv2.destroyAllWindows()
    print('Done')

