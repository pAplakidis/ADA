import numpy as np
import cereal.messaging as messaging
from cereal import log
from cereal.visionipc import VisionIpcServer, VisionStreamType

W = 1164
H = 874

class Camerad:
  def __init__(self, pm):
    self.pm = pm
    self.frame_id = 0
    self.vipc_server = VisionIpcServer("camerad")

    self.vipc_server.create_buffers(VisionStreamType.VISION_STREAM_ROAD, 5, False, W, H)
    self.vipc_server.start_listener()

  def camera_callback(self, image):
    img = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    img = np.reshape(img, (H, W, 4))
    img = img[:, :, [0, 1, 2]].copy()

    # TODO: convert RGB to BGR
    # TODO: render image here for now

    data = messaging.new_message("roadCameraState")
    msg = {
      "frameId": self.frame_id,
      "transform": [1.0, 0.0, 0.0,
                    0.0, 1.0, 0.0,
                    0.0, 0.0, 1.0]
    }
    setattr(data, "roadCameraState", msg)
    print("roadCameraState:", data)
    self.pm.send("roadCameraState", data)
