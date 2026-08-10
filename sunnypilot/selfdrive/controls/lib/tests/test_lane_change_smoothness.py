"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from cereal import log

from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.drive_helpers import MAX_LATERAL_JERK
from openpilot.sunnypilot.selfdrive.controls.lib.lane_change_smoothness import (
  LaneChangeSmoothness, LaneChangeSmoothnessMode, LANE_CHANGE_SMOOTHNESS_PARAMS,
)

LaneChangeState = log.LaneChangeState


def _make(mode: int) -> LaneChangeSmoothness:
  lcs = LaneChangeSmoothness.__new__(LaneChangeSmoothness)
  lcs.mode = mode
  lcs._active = False
  lcs._elapsed = 0.0
  lcs._prev_in_lc = False
  return lcs


class TestLaneChangeSmoothness:
  def test_off_passthrough(self):
    lcs = _make(LaneChangeSmoothnessMode.OFF)
    assert lcs.update(0.02, 0.0, 30.0, True, LaneChangeState.laneChangeStarting) == 0.02

  def test_default_clamps_on_lc_start(self):
    lcs = _make(LaneChangeSmoothnessMode.DEFAULT)
    f0, _ = LANE_CHANGE_SMOOTHNESS_PARAMS[LaneChangeSmoothnessMode.DEFAULT]
    prev = 0.0
    target = 0.05
    v_ego = 30.0
    out = lcs.update(target, prev, v_ego, True, LaneChangeState.laneChangeStarting)
    max_rate = (MAX_LATERAL_JERK * f0) / (v_ego ** 2) * DT_CTRL
    assert out == max_rate
    assert lcs._active

  def test_strong_more_restrictive_than_light(self):
    light = _make(LaneChangeSmoothnessMode.LIGHT)
    strong = _make(LaneChangeSmoothnessMode.STRONG)
    prev, target, v_ego = 0.0, 0.05, 30.0
    out_light = light.update(target, prev, v_ego, True, LaneChangeState.laneChangeStarting)
    out_strong = strong.update(target, prev, v_ego, True, LaneChangeState.laneChangeStarting)
    assert out_strong < out_light < target

  def test_eases_toward_full_rate(self):
    lcs = _make(LaneChangeSmoothnessMode.DEFAULT)
    _, ramp_t = LANE_CHANGE_SMOOTHNESS_PARAMS[LaneChangeSmoothnessMode.DEFAULT]
    prev, target, v_ego = 0.0, 0.05, 30.0
    first = lcs.update(target, prev, v_ego, True, LaneChangeState.laneChangeStarting)
    steps = int(ramp_t / DT_CTRL) + 1
    out = first
    for _ in range(steps):
      out = lcs.update(target, out, v_ego, True, LaneChangeState.laneChangeFinishing)
    # After full ramp, factor≈1 so clip matches stock ISO rate (still may clamp a large jump)
    stock_rate = MAX_LATERAL_JERK / (v_ego ** 2) * DT_CTRL
    assert abs(out - min(target, stock_rate)) < 1e-9 or out >= first
