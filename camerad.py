import cv2
import numpy as np
import threading

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

IMG_WIDTH = 1164
IMG_HEIGHT = 874

# TODO: use ROS
class Camerad:
  def __init__(self, car):
    self.frame_id = 0
    self.car = car
    self.cv_bridge = CvBridge()
    self.publisher = rospy.Publisher("/camera/image", Image, queue_size=10)

  def camera_callback(self, image):
    img = np.array(image.raw_data)
    img = img.reshape((IMG_HEIGHT, IMG_WIDTH, 4))
    img = img[:, :, :3]
    # TODO: make img rgb and publish message for modeld
    model_img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    ros_img_msg = self.cv_bridge.cv2_to_imgmsg(model_img, encoding="bgr8")
    self.publisher.publish(ros_img_msg)
    self.car.front_camera = img
