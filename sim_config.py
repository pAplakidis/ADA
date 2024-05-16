# use CARLA's autopilot (for testing just the model)
CARLA_AUTOPILOT = False

# TODO: make these env variables
TRAFFIC = False     # determines whether to spawn cars or not
N_VEHICLES = 50     # number of vehicles spawned in the map
N_PEDESTRIANS = 100 # number of pedestrians spawned in the map

# in carla.LightState enum, the 4th and 5th bit represent the s (on/off)
RIGHT__POS = 4
LEFT__POS = 5
