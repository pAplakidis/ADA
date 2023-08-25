# Autonomous Driving Agent (ADA)
A toy self-driving car agent heavily inspired by comma.ai's openpilot

## Requirements
python-opencv
carla simulator
Robot Operating System
NumPY
plotly

## Usage
In three terminals run the following in each:
```
./start_ros.py
```

```
./start_carla.py
```

```
./sim.py
```

### TODO:
- cannot work longterm, need to handle memory better
- ROS messaging (DONE)
- log all ROS messages to a file instead of stdout
- camera daemon to feed images  (DONE)
- add try/except for every ROS message published and received
- sensors daemon to feed sensor data (desires for now)  (DONE)
- model daemon to process that image and output stuff (DONE)
- fps and other metrics for performance monitoring
- modeld.cc and .onnx for faster model processing
- planner (longitudinal and lateral) daemon that preprocesses the path and feeds it to the controller
- controls daemon that decides the overall controls from a path using PID
- board daemon that handles the embedded system ADA runs on

