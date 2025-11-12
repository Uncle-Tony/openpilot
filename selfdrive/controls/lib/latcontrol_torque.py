import json
import math
import numpy as np

from cereal import log
from opendbc.car.lateral import FRICTION_THRESHOLD, get_friction
from openpilot.common.constants import ACCELERATION_DUE_TO_GRAVITY
from openpilot.common.params import Params
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.common.pid import PIDController

from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_ext import LatControlTorqueExt

# At higher speeds (25+mph) we can assume:
# Lateral acceleration achieved by a specific car correlates to
# torque applied to the steering rack. It does not correlate to
# wheel slip, or to speed.

# This controller applies torque to achieve desired lateral
# accelerations. To compensate for the low speed effects we
# use a LOW_SPEED_FACTOR in the error. Additionally, there is
# friction in the steering wheel that needs to be overcome to
# move it at all, this is compensated for too.

LOW_SPEED_X = [0, 10, 20, 30]
LOW_SPEED_Y = [15, 13, 10, 5]


class LatControlTorque(LatControl):
  def __init__(self, CP, CP_SP, CI):
    super().__init__(CP, CP_SP, CI)
    self.CP = CP  # Store to compare against original values
    self.torque_params = CP.lateralTuning.torque.as_builder()
    self.torque_from_lateral_accel = CI.torque_from_lateral_accel()
    self.lateral_accel_from_torque = CI.lateral_accel_from_torque()

    # Load custom tuning params at initialization
    self._params = Params()
    self._load_custom_tuning_params()

    self.pid = PIDController(self.torque_params.kp, self.torque_params.ki,
                             k_f=self.torque_params.kf)
    self.update_limits()
    self.steering_angle_deadzone_deg = self.torque_params.steeringAngleDeadzoneDeg

    # Track param check frame counter (check every 300 frames when inactive, like BluePilot)
    self._param_check_counter = 0
    self._last_kp = self.torque_params.kp
    self._last_ki = self.torque_params.ki
    self._last_deadzone = self.torque_params.steeringAngleDeadzoneDeg
    self._last_kf = self.torque_params.kf
    self._last_latAccelFactor = self.torque_params.latAccelFactor
    self._last_latAccelOffset = self.torque_params.latAccelOffset

    self.extension = LatControlTorqueExt(self, CP, CP_SP, CI)

  def _load_custom_tuning_params(self):
    """Load custom tuning parameters from JSON param if it exists, with validation
    Matches BluePilot's approach of storing all tuning params in a single JSON string"""
    tuning_json_str = self._params.get("LateralTuningParams")
    if not tuning_json_str:
      return

    try:
      tuning_data = json.loads(tuning_json_str)

      # Validate and apply kp (valid range: 0.5 to 3.0)
      if "kp" in tuning_data:
        kp_val = float(tuning_data["kp"])
        if 0.5 <= kp_val <= 3.0 and math.isfinite(kp_val):
          self.torque_params.kp = kp_val

      # Validate and apply ki (valid range: 0.1 to 1.0)
      if "ki" in tuning_data:
        ki_val = float(tuning_data["ki"])
        if 0.1 <= ki_val <= 1.0 and math.isfinite(ki_val):
          self.torque_params.ki = ki_val

      # Note: friction is dynamically managed by live torque learning, not tunable via JSON

      # Validate and apply deadzone (valid range: 0.0 to 0.5)
      if "deadzone" in tuning_data:
        deadzone_val = float(tuning_data["deadzone"])
        if 0.0 <= deadzone_val <= 0.5 and math.isfinite(deadzone_val):
          self.torque_params.steeringAngleDeadzoneDeg = deadzone_val

      # Validate and apply kf (valid range: 0.8 to 1.2)
      if "kf" in tuning_data:
        kf_val = float(tuning_data["kf"])
        if 0.8 <= kf_val <= 1.2 and math.isfinite(kf_val):
          self.torque_params.kf = kf_val

      # Validate and apply latAccelFactor (valid range: 2.5 to 3.2)
      if "latAccelFactor" in tuning_data:
        latAccelFactor_val = float(tuning_data["latAccelFactor"])
        if 2.5 <= latAccelFactor_val <= 3.2 and math.isfinite(latAccelFactor_val):
          self.torque_params.latAccelFactor = latAccelFactor_val

      # Validate and apply latAccelOffset (valid range: -0.1 to 0.1)
      if "latAccelOffset" in tuning_data:
        latAccelOffset_val = float(tuning_data["latAccelOffset"])
        if -0.1 <= latAccelOffset_val <= 0.1 and math.isfinite(latAccelOffset_val):
          self.torque_params.latAccelOffset = latAccelOffset_val

    except (json.JSONDecodeError, ValueError, TypeError, KeyError):
      # Invalid JSON or values - use defaults
      pass

  def update_live_torque_params(self, latAccelFactor, latAccelOffset, friction):
    self.torque_params.latAccelFactor = latAccelFactor
    self.torque_params.latAccelOffset = latAccelOffset
    self.torque_params.friction = friction
    self.update_limits()

  def update_limits(self):
    self.pid.set_limits(self.lateral_accel_from_torque(self.steer_max, self.torque_params),
                        self.lateral_accel_from_torque(-self.steer_max, self.torque_params))

  def update(self, active, CS, VM, params, steer_limited_by_safety, desired_curvature, calibrated_pose, curvature_limited):
    # Check for JSON param updates when controls are inactive (matches BluePilot approach)
    # Only update when not active to prevent relay faults from changing gains during active control
    if not active:
      self._param_check_counter += 1
      if self._param_check_counter >= 300:  # Check every 300 frames (like BluePilot)
        old_kp = self._last_kp
        old_ki = self._last_ki
        old_deadzone = self._last_deadzone
        old_kf = self._last_kf
        old_latAccelFactor = self._last_latAccelFactor
        old_latAccelOffset = self._last_latAccelOffset

        # Reload params from JSON
        self._load_custom_tuning_params()

        # If PID gains changed, update PID and reset integrator
        if (self.torque_params.kp != old_kp or self.torque_params.ki != old_ki):
          self.pid._k_p = [[0], [self.torque_params.kp]]
          self.pid._k_i = [[0], [self.torque_params.ki]]
          self.pid.reset()  # Reset integrator to prevent inconsistent state
          self._last_kp = self.torque_params.kp
          self._last_ki = self.torque_params.ki

        # If kf changed, update PID feedforward gain
        if self.torque_params.kf != old_kf:
          self.pid.k_f = self.torque_params.kf
          self._last_kf = self.torque_params.kf

        # Note: friction is dynamically managed by live torque learning, not from JSON

        # If deadzone changed, update it
        if self.torque_params.steeringAngleDeadzoneDeg != old_deadzone:
          self.steering_angle_deadzone_deg = self.torque_params.steeringAngleDeadzoneDeg
          self._last_deadzone = self.torque_params.steeringAngleDeadzoneDeg

        # If latAccelFactor changed, update limits (affects torque conversion)
        if self.torque_params.latAccelFactor != old_latAccelFactor:
          self.update_limits()
          self._last_latAccelFactor = self.torque_params.latAccelFactor

        # If latAccelOffset changed, it's used directly in update() - no action needed
        if self.torque_params.latAccelOffset != old_latAccelOffset:
          self._last_latAccelOffset = self.torque_params.latAccelOffset

        self._param_check_counter = 0

    # Override torque params from extension
    if self.extension.update_override_torque_params(self.torque_params):
      self.update_limits()

    pid_log = log.ControlsState.LateralTorqueState.new_message()
    if not active:
      output_torque = 0.0
      pid_log.active = False
    else:
      actual_curvature = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll)
      roll_compensation = params.roll * ACCELERATION_DUE_TO_GRAVITY
      curvature_deadzone = abs(VM.calc_curvature(math.radians(self.steering_angle_deadzone_deg), CS.vEgo, 0.0))

      desired_lateral_accel = desired_curvature * CS.vEgo ** 2
      actual_lateral_accel = actual_curvature * CS.vEgo ** 2
      lateral_accel_deadzone = curvature_deadzone * CS.vEgo ** 2

      low_speed_factor = np.interp(CS.vEgo, LOW_SPEED_X, LOW_SPEED_Y)**2
      setpoint = desired_lateral_accel + low_speed_factor * desired_curvature
      measurement = actual_lateral_accel + low_speed_factor * actual_curvature
      gravity_adjusted_lateral_accel = desired_lateral_accel - roll_compensation

      # do error correction in lateral acceleration space, convert at end to handle non-linear torque responses correctly
      pid_log.error = float(setpoint - measurement)
      ff = gravity_adjusted_lateral_accel
      # latAccelOffset corrects roll compensation bias from device roll misalignment relative to car roll
      ff -= self.torque_params.latAccelOffset
      ff += get_friction(desired_lateral_accel - actual_lateral_accel, lateral_accel_deadzone, FRICTION_THRESHOLD, self.torque_params)

      freeze_integrator = steer_limited_by_safety or CS.steeringPressed or CS.vEgo < 5
      output_lataccel = self.pid.update(pid_log.error,
                                      feedforward=ff,
                                      speed=CS.vEgo,
                                      freeze_integrator=freeze_integrator)
      output_torque = self.torque_from_lateral_accel(output_lataccel, self.torque_params)

      # Lateral acceleration torque controller extension updates
      # Overrides pid_log.error and output_torque
      pid_log, output_torque = self.extension.update(CS, VM, self.pid, params, ff, pid_log, setpoint, measurement, calibrated_pose, roll_compensation,
                                                     desired_lateral_accel, actual_lateral_accel, lateral_accel_deadzone, gravity_adjusted_lateral_accel,
                                                     desired_curvature, actual_curvature, steer_limited_by_safety, output_torque)

      pid_log.active = True
      pid_log.p = float(self.pid.p)
      pid_log.i = float(self.pid.i)
      pid_log.d = float(self.pid.d)
      pid_log.f = float(self.pid.f)
      pid_log.output = float(-output_torque)  # TODO: log lat accel?
      pid_log.actualLateralAccel = float(actual_lateral_accel)
      pid_log.desiredLateralAccel = float(desired_lateral_accel)
      pid_log.saturated = bool(self._check_saturation(self.steer_max - abs(output_torque) < 1e-3, CS, steer_limited_by_safety, curvature_limited))

    # TODO left is positive in this convention
    return -output_torque, 0.0, pid_log
