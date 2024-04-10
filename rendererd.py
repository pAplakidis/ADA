import threading
import cv2
import numpy as np

import io
import plotly.io as pio
import plotly.express as px
import plotly.graph_objects as go

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray

from utils import *


# TODO: later on make a full grid UI
class Rendererd:
  def __init__(self):
    self.xy_path = np.zeros((TRAJECTORY_LENGTH, N_COORDINATES), dtype=float)
    self.xy_path_norm = np.zeros((TRAJECTORY_LENGTH_FINAL, N_COORDINATES), dtype=float)

    # self.subscriber = rospy.Subscriber("/camera/image", Image, self.process_camera_data)
    self.subscriber = rospy.Subscriber("/model/outputs", Float64MultiArray, self.plot_trajectory)
    self.cv_bridge = CvBridge()

    self.fig = go.FigureWidget()

    self.model_thread = threading.Thread(target=self.run_inference_loop, daemon=True)
    self.model_thread.start()

  def run_inference_loop(self):
    while not rospy.is_shutdown():
      rospy.spin()

      # self.render_display()

  def plot_trajectory(self, msg):
    self.model_outputs = msg.data
    # TODO: make modular (switch to multitask, etc)
    # self.xy_path = np.array(self.model_outputs[:-1]).reshape((TRAJECTORY_LENGTH, N_COORDINATES))
    # self.crossroad = self.model_outputs[-1]
    self.xy_path = np.array(self.model_outputs).reshape((TRAJECTORY_LENGTH, N_COORDINATES))

    # TODO: add this to path planner as well (or to modeld)
    # preprocess path: reduce trajectory length from 200 to 50
    idx = 0
    for i in range(0, TRAJECTORY_LENGTH, TRAJECTORY_LENGTH // TRAJECTORY_LENGTH_FINAL):
      self.xy_path_norm[idx] = self.xy_path[i]
      idx += 1

    if self.fig:
      self.fig.data = []
      path_x = self.xy_path_norm[:, 0]
      path_y = self.xy_path_norm[:, 1]
      marker = {"color": "blue"}
      self.fig.add_scatter(x=path_x, y=path_y, name="path", marker=marker)

  def render_display(self):
    # plot path
    if self.fig:
      self.fig.update_layout(xaxis_range=[-50,50])
      self.fig.update_layout(yaxis_range=[0,50])

      # if self.xy_path_norm is not None and self.crossroad is not None:
      if self.xy_path_norm is not None:
        self._figshow(self.fig)

  def _render_img(self, img):
    cv2.imshow("DISPLAY", img)
    if cv2.waitKey(1) & 0xFF == 27: pass

    # plot path
    self.render_display()

  @staticmethod
  # for plotting path
  def _figshow(fig):
    buf = io.BytesIO()
    pio.write_image(fig, buf)
    buf.seek(0)
    file_bytes = np.asarray(bytearray(buf.read()), dtype=np.uint8)
    img = cv2.imdecode(file_bytes, cv2.IMREAD_COLOR)
    cv2.imshow("Predicted Path", img)
