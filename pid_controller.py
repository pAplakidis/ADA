#!/usr/bin/env python3

class PID:
  def __init__(self, k_p, k_i, k_f=0., k_d=0.):
    self._k_p = k_p
    self._k_i = k_i
    self._k_d = k_d
    self._k_f = k_f

  def reset(self):
    self.p = 0.0
    self.i = 0.0
    self.d = 0.0
    self.f = 0.0
    self.control = 0

  def update(self, error, error_rate=0.0, speed=0.0):
    self.p = float(error) * self._k_p

    self.control = self.p + self.i + self.d + self.f
    return self.control

class LateralPID:
  def __init__(self):
    pass

  def reset(self):
    pass

  def update(self):
    pass
