import numpy as np
from numbers import Number
from collections import deque
from common.numpy_fast import clip, interp


def apply_deadzone(error, deadzone):
  if error > deadzone:
    error -= deadzone
  elif error < - deadzone:
    error += deadzone
  else:
    error = 0.
  return error


class PIDController:
  def __init__(self, k_p=0., k_i=0., k_d=0., k_f=1., k_11=0., k_12=0., k_13=0., k_period=1., pos_limit=None, neg_limit=None, rate=100, sat_limit=0.8, derivative_period=1.):
    self._k_p = k_p  # proportional gain
    self._k_i = k_i  # integral gain
    self._k_d = k_d  # derivative gain
    self.k_f = k_f   # feedforward gain
    if isinstance(self._k_p, Number):
      self._k_p = [[0], [self._k_p]]
    if isinstance(self._k_i, Number):
      self._k_i = [[0], [self._k_i]]
    if isinstance(self._k_d, Number):
      self._k_d = [[0], [self._k_d]]

    self._k_11 = k_11  # proportional gain factor
    self._k_12 = k_12  # integral gain factor
    self._k_13 = k_13  # derivative gain factor
    if isinstance(self._k_11, Number):
      self._k_11 = [[0], [self._k_11]]
    if isinstance(self._k_12, Number):
      self._k_12 = [[0], [self._k_12]]
    if isinstance(self._k_13, Number):
      self._k_13 = [[0], [self._k_13]]

    self.pos_limit = pos_limit
    self.neg_limit = neg_limit

    self.sat_count_rate = 1.0 / rate
    self.i_unwind_rate = 0.3 / rate
    self.i_rate = 1.0 / rate
    self.sat_limit = sat_limit

    if any([k > 0.0 for k in self._k_d[1]]):
      self._d_period = round(derivative_period * rate)  # period of time for derivative calculation (seconds converted to frames)
      self._d_period_recip = 1. / self._d_period
      self.outputs = deque(maxlen=self._d_period)
    else:
      self.outputs = None
    
    if any([k > 0.0 for kk in [self._k_11[1], self._k_12[1], self._k_13[1]] for k in kk]):
      self._k_period = round(k_period * rate)  # period of time for autotune calculation (seconds converted to frames)
      self.output_norms = deque(maxlen=self._k_period)
    else:
      self.output_norms = None

    self.reset()

  @property
  def k_p(self):
    return interp(self.speed, self._k_p[0], self._k_p[1])

  @property
  def k_i(self):
    return interp(self.speed, self._k_i[0], self._k_i[1])

  @property
  def k_d(self):
    return interp(self.speed, self._k_d[0], self._k_d[1])
  
  @property
  def k_11(self):
    return interp(self.speed, self._k_11[0], self._k_11[1])

  @property
  def k_12(self):
    return interp(self.speed, self._k_12[0], self._k_12[1])

  @property
  def k_13(self):
    return interp(self.speed, self._k_13[0], self._k_13[1])

  def _check_saturation(self, control, check_saturation, error):
    saturated = (control < self.neg_limit) or (control > self.pos_limit)

    if saturated and check_saturation and abs(error) > 0.1:
      self.sat_count += self.sat_count_rate
    else:
      self.sat_count -= self.sat_count_rate

    self.sat_count = clip(self.sat_count, 0.0, 1.0)

    return self.sat_count > self.sat_limit

  def reset(self):
    self.p = 0.0
    self.i = 0.0
    self.d = 0.0
    self.f = 0.0
    self.kp = 0.0
    self.ki = 0.0
    self.kd = 0.0
    self.sat_count = 0.0
    self.saturated = False
    self.control = 0
    if self.outputs:
      self.outputs = deque(maxlen=self._d_period)
    if self.output_norms:
      self.output_norms = deque(maxlen=self._k_period)

  def update(self, setpoint, measurement, speed=0.0, check_saturation=True, override=False, feedforward=0., deadzone=0., freeze_integrator=False):
    self.speed = speed

    error = float(apply_deadzone(setpoint - measurement, deadzone))

    self.kp = self.k_p
    self.ki = self.k_i
    self.kd = self.k_d
    
    if self.output_norms is not None and self.outputs is not None and len(self.outputs) > 0:
      abs_sp = setpoint if setpoint > 0. else -setpoint
      self.output_norms.append(self.outputs[-1] / (abs_sp + 1.)) # use the last iteration's output
      if len(self.output_norms) == int(self._k_period):
        delta_error_norm = self.output_norms[-1] - self.output_norms[0]
        gain_update_factor = self.output_norms[-1] * delta_error_norm
        if gain_update_factor != 0.:
          abs_guf = abs(gain_update_factor)
          self.kp *= 1. + min(2., self.k_11 * abs_guf)
          self.ki *= 1. + clip(self.k_12 * gain_update_factor, -1., 2.)
          self.kd *= 1. + min(2., self.k_13 * abs_guf)

      
    
    self.p = error * self.kp
    self.f = feedforward * self.k_f
    
    if self.outputs and len(self.outputs) == int(self._d_period):  # makes sure we have enough history for period
      self.d = clip((self.outputs[-1] - self.outputs[0]) * self._d_period_recip * self.kd, -abs(self.p), abs(self.p))
    else:
      self.d = 0.

    if override:
      self.i -= self.i_unwind_rate * float(np.sign(self.i))
    else:
      i = self.i + error * self.ki * self.i_rate
      control = self.p + self.f + i + self.d

      # Update when changing i will move the control away from the limits
      # or when i will move towards the sign of the error
      if ((error >= 0 and (control <= self.pos_limit or i < 0.0)) or
         (error <= 0 and (control >= self.neg_limit or i > 0.0))) and \
         not freeze_integrator:
        self.i = i

    control = self.p + self.f + self.i + self.d
    self.saturated = self._check_saturation(control, check_saturation, error)
    
    if self.outputs is not None:
      self.outputs.append(control)

    self.control = clip(control, self.neg_limit, self.pos_limit)
    return self.control
