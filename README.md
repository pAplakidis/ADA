# Autonomous Driving Agent (ADA)
A toy self-driving car agent

### TODO:
- ROS or cereal messaging
- camera and sensors daemon to feed images, desires and other data to other daemons
- model daemon to process that image and output stuff
- planner (longitudinal and lateral) daemon that preprocesses the path and feeds it to the controller
- controls daemon that decides the overall controls from a path using PID
- board daemon that handles the embedded system ADA runs on
