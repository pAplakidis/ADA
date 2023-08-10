#!/usr/bin/env python3
import math
import random
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from torchvision.models import efficientnet_b2

ONEOVERSQRT2PI = 1.0 / math.sqrt(2 * math.pi)

class MTP(nn.Module):
  # n_modes: number of paths output
  # path_len: number of points of each path
  def __init__(self, in_feats, n_modes=3, path_len=200, hidden_feats=4096):
    super(MTP, self).__init__()
    self.n_modes = n_modes
    self.fc1 = nn.Linear(in_feats, hidden_feats)
    self.fc2 = nn.Linear(hidden_feats, int(n_modes * (path_len*2) + n_modes))

  def forward(self, x):
    x = self.fc2(self.fc1(x))

    # normalize the probabilities to sum to 1 for inference
    mode_probs = x[:, -self.n_modes:].clone()
    if not self.training:
      mode_probs = F.softmax(mode_probs, dim=1)
    
    x = x[:, :-self.n_modes]
    return torch.cat((x, mode_probs), 1)


# Inputs: 1 frame and desire
# Outputs: trajectory and crossroad prediction
class ComboModel(nn.Module):
  def __init__(self, n_paths=3):
    super(ComboModel, self).__init__()
    self.n_paths = n_paths
    effnet = efficientnet_b2(pretrained=True)

    self.vision = nn.Sequential(*(list(effnet.children())[:-1]))
    self.policy = MTP(1411, n_modes=self.n_paths)
    self.cr_detector = nn.Sequential(
      nn.Linear(1411, 1024),
      nn.BatchNorm1d(1024),
      nn.ReLU(),
      nn.Linear(1024, 128),
      nn.BatchNorm1d(128),
      nn.ReLU(),
      nn.Linear(128, 84),
      nn.BatchNorm1d(84),
      nn.ReLU(),
      nn.Linear(84, 1)
    )

  def forward(self, x, desire):
    x = self.vision(x)
    x = x.view(-1, self.num_flat_features(x))
    x = torch.cat((x, desire), 1)
    #print(x.shape)
    path = self.policy(x)
    crossroad = torch.sigmoid(self.cr_detector(x))
    return path, crossroad

  def num_flat_features(self, x):
    size = x.size()[1:] # all dimensions except the batch dimension
    num_features = 1
    for s in size:
      num_features *= s
    return num_features

  # TODO: edit those since they belonged to MTP loss function
  # helper functions for handling outputs

  # splits the model predictions into mode probabilities and path
  def _get_trajectory_and_modes(self, model_pred):
    mode_probs = model_pred[:, -self.n_modes:].clone()
    desired_shape = (model_pred.shape[0], self.n_modes, -1, self.n_location_coords_predicted)
    trajectories_no_modes = model_pred[:, :-self.n_modes].clone().reshape(desired_shape)
    return trajectories_no_modes, mode_probs

  # computes the angle between the last points of two paths (degrees)
  @staticmethod
  def _angle_between(ref_traj, traj_to_compare):
    EPSILON = 1e-5

    if (ref_traj.ndim != 2 or traj_to_compare.ndim != 2 or
            ref_traj.shape[1] != 2 or traj_to_compare.shape[1] != 2):
        raise ValueError('Both tensors should have shapes (-1, 2).')

    if torch.isnan(traj_to_compare[-1]).any() or torch.isnan(ref_traj[-1]).any():
        return 180. - EPSILON

    traj_norms_product = float(torch.norm(ref_traj[-1]) * torch.norm(traj_to_compare[-1]))

    # If either of the vectors described in the docstring has norm 0, return 0 as the angle.
    if math.isclose(traj_norms_product, 0):
        return 0.

    # We apply the max and min operations below to ensure there is no value
    # returned for cos_angle that is greater than 1 or less than -1.
    # This should never be the case, but the check is in place for cases where
    # we might encounter numerical instability.
    dot_product = float(ref_traj[-1].dot(traj_to_compare[-1]))
    angle = math.degrees(math.acos(max(min(dot_product / traj_norms_product, 1), -1)))

    if angle >= 180:
        return angle - EPSILON

    return angle

  # compute the average of l2 norms of each row in the tensor
  @staticmethod
  def _compute_ave_l2_norms(tensor):
    #l2_norms = torch.norm(tensor, p=2, dim=2)
    l2_norms = torch.norm(tensor, p=2, dim=1)
    avg_distance = torch.mean(l2_norms)
    return avg_distance.item()

  # compute angle between the target path and predicted paths
  def _compute_angles_from_ground_truth(self, target, trajectories):
    angles_from_ground_truth = []
    for mode, mode_trajectory in enumerate(trajectories):
        # For each mode, we compute the angle between the last point of the predicted trajectory for that
        # mode and the last point of the ground truth trajectory.
        #angle = self._angle_between(target[0], mode_trajectory)
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
      best_mode = random.randint(0, self.n_modes-1)
    else:
      distances_from_ground_truth = []
      for angle, mode in angles_from_ground_truth[:max_angle_below_thresh_idx+1]:
        norm = self._compute_ave_l2_norms(target - trajectories[mode, :, :])
        distances_from_ground_truth.append((norm, mode))

      distances_from_ground_truth = sorted(distances_from_ground_truth)
      best_mode = distances_from_ground_truth[0][1]

    return best_mode

def load_model(path, model):
  model.load_state_dict(torch.load(path))
  print("Loaded model from", path)
  return model


# TODO: pass cereal messaging (sm, pm)
# TODO: output to log instead of terminal
def modeld_thread(model, frame, desire, device):
  with torch.no_grad:
    # TODO: maybe preprocess those in camerad and sensord
    X = torch.tensor([frame, frame]).float().to(device)
    DES = torch.tensor([desire, desire]).float().to(device)

    out_path, crossroad = model(X, DES)
    trajectories, modes = model._get_trajectory_and_modes(out_path)

    # TODO: sort trajectories based on modes/probabilities
    payload = {
      "trajectories": trajectories[0],
      "crossroad": crossroad[0]
    }
    # TODO: publish message instead of returning
    return payload

if __name__ == "__main__":
  device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
  print("[+] Using device:", device)

  model = ComboModel()
  model = load_model("./models/ComboModel.pth", model)
  model.eval()

  # modeld_thread(model, np.zeros((224,224), dtype=float))
