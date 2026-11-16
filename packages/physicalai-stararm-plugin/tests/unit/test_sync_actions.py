# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

"""Exercise SDK capability dispatch without opening a device."""

from __future__ import annotations

import sys
from importlib import import_module
from types import SimpleNamespace
from typing import Any
from unittest.mock import MagicMock, call

import numpy as np
import pytest

sys.modules.setdefault("motorbridge_smart_servo", MagicMock())


def _robot(*, leader: bool = False, **kwargs: Any) -> Any:
    name = "stararm102hd" if leader else "stararm102fl"
    cls = "StarArm102HDLeader" if leader else "StarArm102FLFollower"
    return getattr(import_module(f"physicalai_stararm_plugin.{name}"), cls)(**kwargs)


@pytest.mark.parametrize("dtype", [np.float32, np.float64, np.int32])
def test_batch_preserves_order_clipping_duration_and_input(dtype: type) -> None:
    from physicalai_stararm_plugin.constants import STAR_ARM_102_JOINT_ORDER, STAR_ARM_102_JOINT_RANGES_DEG

    robot = _robot()
    bus = robot._bus = MagicMock()
    action = np.array([1000, -1000, 3, 4, 5, 6, 1000], dtype=dtype)
    original = action.copy()
    robot.send_action(action, goal_time=0.1239)
    expected = [
        float(np.clip(float(np.float32(action[i])), *STAR_ARM_102_JOINT_RANGES_DEG[name]))
        for i, name in enumerate(STAR_ARM_102_JOINT_ORDER)
    ]
    assert bus.mock_calls == [call.set_angles(list(range(7)), expected, multi_turn=True, interval_ms=123)]
    np.testing.assert_array_equal(action, original)


@pytest.mark.parametrize("goal_time, expected", [(0.0, 10), (-1.0, 10), (0.001, 10), (0.2, 200)])
def test_batch_preserves_command_duration_floor(goal_time: float, expected: int) -> None:
    robot = _robot()
    bus = robot._bus = MagicMock()
    robot.send_action(np.zeros(7), goal_time=goal_time)
    assert bus.set_angles.call_args.kwargs == {"multi_turn": True, "interval_ms": expected}


@pytest.mark.parametrize("error", [OSError("partial transmission"), ValueError("bad target")])
def test_batch_error_never_retries_with_addressed_writes(error: Exception) -> None:
    robot = _robot()
    bus = robot._bus = MagicMock()
    bus.set_angles.side_effect = error
    with pytest.raises(type(error), match=str(error)):
        robot.send_action(np.zeros(7))
    bus.set_angles.assert_called_once()
    bus.set_angle.assert_not_called()


@pytest.mark.parametrize("noncallable", [False, True])
def test_legacy_sdk_retains_seven_addressed_writes(noncallable: bool) -> None:
    robot = _robot()
    bus = robot._bus = MagicMock(spec=["set_angle"])
    if noncallable:
        bus.set_angles = None
    robot.send_action(np.arange(7))
    assert bus.set_angle.call_args_list == [
        call(i, value, multi_turn=True, interval_ms=100) for i, value in enumerate([0.0, 1.0, 1.0, 3.0, 4.0, 5.0, 6.0])
    ]


def test_repeated_actions_still_transmit_independent_batches() -> None:
    robot = _robot()
    bus = robot._bus = MagicMock()
    for _ in range(2):
        robot.send_action(np.zeros(7))
    assert bus.set_angles.call_count == 2
    assert bus.set_angles.call_args_list[0].args[1] is not bus.set_angles.call_args_list[1].args[1]
    assert len(bus.mock_calls) == 2


def test_invalid_shape_does_not_send_anything() -> None:
    robot = _robot()
    bus = robot._bus = MagicMock()
    with pytest.raises(ValueError, match="Expected action shape"):
        robot.send_action(np.zeros(6))
    assert not bus.mock_calls


def test_passive_leader_sends_no_batch() -> None:
    robot = _robot(leader=True)
    bus = robot._bus = MagicMock()
    robot.send_action(np.zeros(7))
    assert not bus.mock_calls


def test_assist_profile_is_converted_before_batching() -> None:
    robot = _robot(leader=True, control_mode="assist", follower_profile="b601")
    bus = robot._bus = MagicMock()
    robot.send_action(np.array([-1, -2, -3, 4, 5, -6, -60]))
    bus.set_angles.assert_called_once_with(
        list(range(7)), [1.0, 2.0, -3.0, 4.0, 5.0, 6.0, 10.0], multi_turn=True, interval_ms=100
    )
    bus.set_angle.assert_not_called()


def test_hold_batches_native_pose_not_profiled_pose() -> None:
    robot = _robot(leader=True, follower_profile="b601")
    bus = robot._bus = MagicMock()
    native_pose = [10.0, 10.0, -10.0, 10.0, 10.0, 10.0, 10.0]
    bus.sync_monitor.return_value = {
        i: SimpleNamespace(angle_deg=value, reliable=True) for i, value in enumerate(native_pose)
    }
    robot.hold_position()
    bus.set_angles.assert_called_once_with(list(range(7)), native_pose, multi_turn=True, interval_ms=200)
    assert robot.is_holding
    bus.unlock.assert_not_called()
