"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from cereal import log

from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.drive_helpers import MAX_LATERAL_JERK


class LaneChangeSmoothnessMode:
  OFF = 0
  LIGHT = 1
  DEFAULT = 2
  STRONG = 3


# (initial rate-cap factor f0, ramp seconds T). Factor eases f0→1.0 with smoothstep.
# Default matches the validated highway bake: soft start, still tracks the plan mid-maneuver.
LANE_CHANGE_SMOOTHNESS_PARAMS = {
  LaneChangeSmoothnessMode.OFF: None,
  LaneChangeSmoothnessMode.LIGHT: (0.25, 6.0),
  LaneChangeSmoothnessMode.DEFAULT: (0.12, 8.0),
  LaneChangeSmoothnessMode.STRONG: (0.05, 10.0),
}

LaneChangeState = log.LaneChangeState


class LaneChangeSmoothness:
  """Pre-clamp desired-curvature rate during lane changes (before stock clip_curvature).

  The model plan can demand lateral jerk up to the ISO limit while torque slew is slower,
  causing lag → windup → overshoot. This scales the rate cap with a smoothstep ease-in for
  the duration of the maneuver. Lateral accel ceiling is left to clip_curvature.
  """

  def __init__(self):
    self.params = Params()
    self.mode = LaneChangeSmoothnessMode.OFF
    self._active = False
    self._elapsed = 0.0
    self._prev_in_lc = False
    self.get_params()

  def get_params(self) -> None:
    mode = self.params.get("LaneChangeSmoothness", return_default=True)
    if mode not in LANE_CHANGE_SMOOTHNESS_PARAMS:
      mode = LaneChangeSmoothnessMode.OFF
    self.mode = mode

  def update(self, new_desired_curvature: float, prev_curvature: float, v_ego: float,
             lat_active: bool, lane_change_state: int) -> float:
    params = LANE_CHANGE_SMOOTHNESS_PARAMS.get(self.mode)
    if params is None:
      self._active = False
      self._elapsed = 0.0
      self._prev_in_lc = False
      return new_desired_curvature

    in_lc = lat_active and lane_change_state in (LaneChangeState.laneChangeStarting,
                                                LaneChangeState.laneChangeFinishing)
    if in_lc and not self._prev_in_lc:
      self._active = True
      self._elapsed = 0.0

    if self._active:
      f0, ramp_t = params
      self._elapsed = min(self._elapsed + DT_CTRL, ramp_t)
      if self._elapsed >= ramp_t and not in_lc:
        self._active = False
      else:
        u = self._elapsed / ramp_t
        ease = u * u * (3.0 - 2.0 * u)  # smoothstep
        factor = f0 + (1.0 - f0) * ease
        if factor < 1.0:
          rate = (MAX_LATERAL_JERK * factor) / (max(v_ego, 1.0) ** 2) * DT_CTRL
          new_desired_curvature = min(max(new_desired_curvature, prev_curvature - rate),
                                      prev_curvature + rate)

    self._prev_in_lc = in_lc
    return new_desired_curvature
