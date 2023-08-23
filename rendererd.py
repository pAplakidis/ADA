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


# TODO: later on make a full grid UI
class Rendererd:
  def __init__(self):
    # self.subscriber = rospy.Subscriber("/camera/image", Image, self.process_camera_data)
    self.subscriber = rospy.Subscriber("/model/outputs", Float64MultiArray, self.plot_trajectories)
    self.cv_bridge = CvBridge()

    self.fig = go.FigureWidget()

    self.model_thread = threading.Thread(target=self.run_inference_loop, daemon=True)
    self.model_thread.start()

  def run_inference_loop(self):
    while not rospy.is_shutdown():
      rospy.spin()

      # self.render_display()

  def plot_trajectories(self, msg):
    self.model_outputs = msg.data
    self.trajectories = np.array(self.model_outputs[:-1]).reshape((3, 200, 2)) # TODO: generalize it with arguments
    self.crossroad = self.model_outputs[-1]
    print("[rendererd]: model_outputs ->", self.trajectories.shape, self.crossroad)

    if self.fig:
      self.fig.data = []
      for idx, pred_path in enumerate(self.trajectories):
        path_x = pred_path[:, 0]
        path_y = pred_path[:, 1]
        # if modes[0][idx] == torch.max(modes[0]):
        #   marker = {"color": "red"}
        #   name = "best_path"
        # else:
        marker = {"color": "blue"}
        name = "path"+str(idx)
        self.fig.add_scatter(x=path_x, y=path_y, name=name, marker=marker)

  def render_display(self):
    # plot trajectories
    if self.fig:
      self.fig.update_layout(xaxis_range=[-50,50])
      self.fig.update_layout(yaxis_range=[0,50])

      if self.trajectories and self.crossroad:
        self._figshow(self.fig)

  def _render_img(self, img):
    cv2.imshow("DISPLAY", img)
    if cv2.waitKey(1) & 0xFF == 27: pass

    # plot trajectories
    if self.fig:
      self.fig.update_layout(xaxis_range=[-50,50])
      self.fig.update_layout(yaxis_range=[0,50])

      if self.trajectories is not None and self.crossroad is not None:
        self._figshow(self.fig)

  @staticmethod
  # for plotting trajectories
  def _figshow(fig):
    buf = io.BytesIO()
    pio.write_image(fig, buf)
    buf.seek(0)
    file_bytes = np.asarray(bytearray(buf.read()), dtype=np.uint8)
    img = cv2.imdecode(file_bytes, cv2.IMREAD_COLOR)
    cv2.imshow("Predicted Path", img)
