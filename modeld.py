import random
import threading
import math
import cv2
import numpy as np

import torch
import torch.nn as nn
import torch.nn.functional as F
from torchvision.models import efficientnet_b2
import onnxruntime

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray

from utils import *
from ADA_training_stack.lateral.model import MTP, ComboModel, load_model

# TODO: this will later be a C++ loader (onnx)
class Modeld:
  def __init__(self, verbose=False):
    self.verbose = verbose
    self.camera_subscriber = rospy.Subscriber("/camera/image", Image, self.run_model)
    self.desire_subscriber = rospy.Subscriber("/sensor/desire", Float64MultiArray, self._desire_callback)
    self.cv_bridge = CvBridge()
    self.publisher = rospy.Publisher("/model/outputs", Float64MultiArray, queue_size=10)

    self.img_in = np.zeros((1, 3, 224, 224))
    self.desire = np.array([1.0, 0.0, 0.0])

    # init model
    # classic pytorch
    # if torch.cuda.is_available():
    #   torch.cuda.init()
    #   self.device = torch.device("cuda:0")
    # else:
    #   self.device = "cpu"
    """
    self.device = "cpu"
    self.model = ComboModel().to(self.device)
    self.model = load_model("./models/ComboModel.pth", self.model)
    self.model.eval()
    """

    # onnx 
    self.onnx_path = "./models/ComboModel.onnx"
    self.session = onnxruntime.InferenceSession(self.onnx_path)
    self.n_modes=3
    self.regression_loss_weigh=1.
    self.angle_threshold_degrees=5.
    self.n_location_coords_predicted = 2

    self.model_thread = threading.Thread(target=self.run_inference_loop, daemon=True)
    self.model_thread.start()

  def run_inference_loop(self):
    while not rospy.is_shutdown():
      rospy.spin()

  def _desire_callback(self, msg):
    try:
      self.desire = np.array(msg.data)
      if(self.verbose):
        print("[modeld]: received desire ->", self.desire)
    except Exception as e:
      print("[modeld]: error processing desire message ->", e)

  def run_model(self, msg):
    try:
      img_in = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
      img_in = cv2.resize(img_in, (W,H))
      self.img_in = np.moveaxis(img_in, -1, 0)
    except Exception as e:
      print("[modeld]: error processing image:", e)

    try:
      with torch.no_grad():
        X = torch.tensor([self.img_in]).float()#.to(self.device)
        DES = torch.tensor([self.desire]).float()#.to(self.device)

        """
        # pth
        out_path, crossroad = self.model(X, DES)
        trajectories, modes = self._get_trajectory_and_modes(out_path.cpu().numpy())
        xy_path = trajectories[0][0]
        for idx, pred_path in enumerate(trajectories[0]):
          if modes[0][idx] == np.max(modes[0]):
            xy_path = trajectories[0][idx]
        outputs = {
          "trajectory": xy_path,
          "crossroad": crossroad[0]
        }
        """

        # onnx
        outputs = self.session.run(["path", "crossroad"],
                                   {
                                      "road_image": X.cpu().numpy(),
                                      "desire": DES.cpu().numpy()
                                    })

        path, crossroad = outputs
        trajectories, modes = self._get_trajectory_and_modes(path)
        xy_path = trajectories[0][0]
        for idx, pred_path in enumerate(trajectories[0]):
          if modes[0][idx] == np.max(modes[0]):
            xy_path = trajectories[0][idx]

        if self.verbose:
          print("[modeld]: outputs =>", outputs)

        outputs_list = xy_path.flatten().tolist()
        outputs_list.append(crossroad[0])
        outputs_msg = Float64MultiArray()
        outputs_msg.data = outputs_list
        self.publisher.publish(outputs_msg)

    except Exception as e:
      print("[modeld]: error running model:", e)

  # path processing functions
  # splits the model predictions into mode probabilities and path
  def _get_trajectory_and_modes(self, model_pred):
    mode_probs = model_pred[:, -self.n_modes :].copy()
    desired_shape = (
      model_pred.shape[0],
      self.n_modes,
      -1,
      self.n_location_coords_predicted,
    )
    trajectories_no_modes = (
      model_pred[:, : -self.n_modes].copy().reshape(desired_shape)
    )
    return trajectories_no_modes, mode_probs

  # computes the angle between the last points of two paths (degrees)
  @staticmethod
  def _angle_between(ref_traj, traj_to_compare):
    EPSILON = 1e-5

    if (
      ref_traj.ndim != 2
      or traj_to_compare.ndim != 2
      or ref_traj.shape[1] != 2
      or traj_to_compare.shape[1] != 2
    ):
      raise ValueError("Both tensors should have shapes (-1, 2).")

    if torch.isnan(traj_to_compare[-1]).any() or torch.isnan(ref_traj[-1]).any():
      return 180.0 - EPSILON

    traj_norms_product = float(
      torch.norm(ref_traj[-1]) * torch.norm(traj_to_compare[-1])
    )

    # If either of the vectors described in the docstring has norm 0, return 0 as the angle.
    if math.isclose(traj_norms_product, 0):
      return 0.0

    # We apply the max and min operations below to ensure there is no value
    # returned for cos_angle that is greater than 1 or less than -1.
    # This should never be the case, but the check is in place for cases where
    # we might encounter numerical instability.
    dot_product = float(ref_traj[-1].dot(traj_to_compare[-1]))
    angle = math.degrees(
      math.acos(max(min(dot_product / traj_norms_product, 1), -1))
    )

    if angle >= 180:
      return angle - EPSILON

    return angle

  # compute the average of l2 norms of each row in the tensor
  @staticmethod
  def _compute_ave_l2_norms(tensor):
    # l2_norms = torch.norm(tensor, p=2, dim=2)
    l2_norms = torch.norm(tensor, p=2, dim=1)
    avg_distance = torch.mean(l2_norms)
    return avg_distance.item()

  # compute angle between the target path and predicted paths
  def _compute_angles_from_ground_truth(self, target, trajectories):
    angles_from_ground_truth = []
    for mode, mode_trajectory in enumerate(trajectories):
      # For each mode, we compute the angle between the last point of the predicted trajectory for that
      # mode and the last point of the ground truth trajectory.
      # angle = self._angle_between(target[0], mode_trajectory)
      angle = self._angle_between(target, mode_trajectory)

      angles_from_ground_truth.append((angle, mode))
    return angles_from_ground_truth

  # finds the index of the best mode given the angles from the ground truth
  def _compute_best_mode(self, angles_from_ground_truth, target, trajectories):
    angles_from_ground_truth = sorted(angles_from_ground_truth)
    max_angle_below_thresh_idx = -1
    for angle_idx, (angle, mode) in enumerate(angles_from_ground_truth):
      if angle <= self.angle_threshold:
        max_angle_below_thresh_idx = angle_idx
      else:
        break

    if max_angle_below_thresh_idx == -1:
      best_mode = random.randint(0, self.n_modes - 1)
    else:
      distances_from_ground_truth = []
      for angle, mode in angles_from_ground_truth[
        : max_angle_below_thresh_idx + 1
      ]:
        norm = self._compute_ave_l2_norms(target - trajectories[mode, :, :])
        distances_from_ground_truth.append((norm, mode))

      distances_from_ground_truth = sorted(distances_from_ground_truth)
      best_mode = distances_from_ground_truth[0][1]

    return best_mode
