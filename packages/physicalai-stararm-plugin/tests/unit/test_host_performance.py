# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

"""Parity and ownership checks for scalar host-path clipping; no hardware."""

from __future__ import annotations

import sys
from importlib import import_module
from types import ModuleType, SimpleNamespace
from typing import Any
from unittest.mock import MagicMock

import numpy as np
import pytest

sys.modules.setdefault("motorbridge_smart_servo", MagicMock())


def _base() -> ModuleType:
    return import_module("physicalai_stararm_plugin._stararm102")


def _robot() -> Any:
    return import_module(
        "physicalai_stararm_plugin.stararm102fl"
    ).StarArm102FLFollower()


def _assert_same_float32(actual: np.ndarray, expected: np.ndarray) -> None:
    assert actual.dtype == expected.dtype == np.float32
    np.testing.assert_array_equal(actual, expected)
    np.testing.assert_array_equal(np.signbit(actual), np.signbit(expected))


def test_scalar_clip_matches_numpy() -> None:
    from physicalai_stararm_plugin.constants import (
        STAR_ARM_102_B601_FOLLOWER_RANGES_DEG,
        STAR_ARM_102_JOINT_RANGES_DEG,
    )

    rng = np.random.default_rng(42)
    ranges = [
        *STAR_ARM_102_JOINT_RANGES_DEG.values(),
        *STAR_ARM_102_B601_FOLLOWER_RANGES_DEG.values(),
    ]
    for lower, upper in ranges:
        values = [
            lower,
            upper,
            -0.0,
            0.0,
            float("nan"),
            float("inf"),
            -float("inf"),
            np.nextafter(lower, -np.inf),
            np.nextafter(lower, np.inf),
            np.nextafter(upper, -np.inf),
            np.nextafter(upper, np.inf),
            *rng.uniform(-1e6, 1e6, 1000),
        ]
        for value in values:
            actual = _base()._clip_degrees(float(value), lower, upper)
            expected = float(np.clip(float(value), lower, upper))
            assert isinstance(actual, float)
            if np.isnan(expected):
                assert np.isnan(actual)
            else:
                assert actual == expected
                assert np.signbit(actual) == np.signbit(expected)


@pytest.mark.parametrize("dtype", [np.float32, np.float64, np.int32])
def test_action_matches_original_scalar_clipping(dtype: type) -> None:
    from physicalai_stararm_plugin.constants import (
        STAR_ARM_102_JOINT_ORDER,
        STAR_ARM_102_JOINT_RANGES_DEG,
    )

    robot = _robot()
    bus = MagicMock()
    robot._bus = bus
    rng = np.random.default_rng(123)
    for _ in range(100):
        action = rng.uniform(-1000, 1000, 7).astype(dtype)
        original = action.copy()
        bus.reset_mock()
        robot.send_action(action, goal_time=0.1239)
        converted = np.asarray(action, dtype=np.float32)
        expected = [
            float(np.clip(float(converted[i]), *STAR_ARM_102_JOINT_RANGES_DEG[name]))
            for i, name in enumerate(STAR_ARM_102_JOINT_ORDER)
        ]
        assert [c.args for c in bus.set_angle.call_args_list] == list(
            enumerate(expected)
        )
        assert all(
            c.kwargs == {"multi_turn": True, "interval_ms": 123}
            for c in bus.set_angle.call_args_list
        )
        np.testing.assert_array_equal(action, original)


def test_action_preserves_nonfinite_values_and_signed_zero() -> None:
    robot = _robot()
    robot._bus = MagicMock()
    robot.send_action(
        np.array([np.nan, np.inf, -np.inf, -0.0, 0.0, -0.0, -0.0], dtype=np.float32)
    )
    calls = robot._bus.set_angle.call_args_list
    assert np.isnan(calls[0].args[1])
    assert calls[1].args[1] == 170.0
    assert calls[2].args[1] == -200.0
    assert np.signbit(calls[3].args[1])
    assert np.signbit(calls[6].args[1]) == np.signbit(np.clip(-0.0, -0.0, 270.0))


def test_b601_profile_matches_original_scalar_math() -> None:
    from physicalai_stararm_plugin.constants import (
        STAR_ARM_102_B601_FOLLOWER_DIRECTIONS,
        STAR_ARM_102_B601_FOLLOWER_RANGES_DEG,
        STAR_ARM_102_JOINT_ORDER,
    )

    robot = import_module("physicalai_stararm_plugin.stararm102hd").StarArm102HDLeader(
        follower_profile="b601"
    )
    rng = np.random.default_rng(42)
    samples = [np.zeros(7, dtype=np.float32), -np.zeros(7, dtype=np.float32)]
    samples.extend(rng.uniform(-1000, 1000, (100, 7)).astype(np.float32))
    for positions in samples:
        original = positions.copy()
        expected = positions.copy()
        for i, name in enumerate(STAR_ARM_102_JOINT_ORDER):
            expected[i] = float(
                np.clip(
                    expected[i] * STAR_ARM_102_B601_FOLLOWER_DIRECTIONS[name],
                    *STAR_ARM_102_B601_FOLLOWER_RANGES_DEG[name],
                )
            )
        actual = robot._observation_positions(positions)
        _assert_same_float32(actual, expected)
        np.testing.assert_array_equal(positions, original)
        assert not np.shares_memory(actual, positions)


def test_write_failure_keeps_addressed_prefix_and_propagates() -> None:
    robot = _robot()
    robot._bus = MagicMock()
    robot._bus.set_angle.side_effect = [None, None, OSError("write failed")]
    with pytest.raises(OSError, match="write failed"):
        robot.send_action(np.zeros(7))
    assert [c.args[0] for c in robot._bus.set_angle.call_args_list] == [0, 1, 2]


def test_observation_wrap_clip_float32_and_ownership() -> None:
    from physicalai_stararm_plugin.constants import (
        STAR_ARM_102_JOINT_ORDER,
        STAR_ARM_102_JOINT_RANGES_DEG,
    )

    robot = _robot()
    bus = MagicMock()
    robot._bus = bus
    rng = np.random.default_rng(91)
    prior = None
    for _ in range(100):
        angles = rng.uniform(-3600, 3600, 7)
        bus.sync_monitor.return_value = {
            i: SimpleNamespace(angle_deg=float(angle), reliable=i % 2 == 0)
            for i, angle in enumerate(angles)
        }
        robot._reset_samples()  # Isolate wrapping/clipping from the separately tested glitch filter.
        observation = robot.get_observation()
        expected = np.empty(7, dtype=np.float32)
        for i, name in enumerate(STAR_ARM_102_JOINT_ORDER):
            lower, upper = STAR_ARM_102_JOINT_RANGES_DEG[name]
            unwrapped, _ = robot._round_to_valid_range(float(angles[i]), lower, upper)
            expected[i] = float(np.clip(unwrapped, lower, upper))
        _assert_same_float32(observation.joint_positions, expected)
        _assert_same_float32(
            observation.sensor_data["raw_positions"], angles.astype(np.float32)
        )
        _assert_same_float32(
            observation.sensor_data["reliable"],
            np.array([1, 0, 1, 0, 1, 0, 1], dtype=np.float32),
        )
        if prior is not None:
            previous, snapshot = prior
            np.testing.assert_array_equal(previous.joint_positions, snapshot)
            assert not np.shares_memory(
                previous.joint_positions, observation.joint_positions
            )
        prior = observation, observation.joint_positions.copy()
        assert not np.shares_memory(observation.joint_positions, robot._last_positions)
    assert bus.sync_monitor.call_count == 100


def test_nonfinite_read_keeps_fallback_and_first_sample_error() -> None:
    robot = _robot()
    bus = MagicMock()
    robot._bus = bus
    monitors = {i: SimpleNamespace(angle_deg=0.0, reliable=True) for i in range(7)}
    bus.sync_monitor.return_value = monitors
    monitors[0].angle_deg = np.nan
    with pytest.raises(ConnectionError, match="Failed to read"):
        robot.get_observation()
    monitors[0].angle_deg = 0.0
    good = robot.get_observation()
    good.joint_positions[:] = (
        99  # Public positions must not modify the native fallback cache.
    )
    monitors[0].angle_deg = np.inf
    fallback = robot.get_observation()
    np.testing.assert_array_equal(
        fallback.joint_positions, np.zeros(7, dtype=np.float32)
    )
    np.testing.assert_array_equal(
        fallback.sensor_data["reliable"], np.zeros(7, dtype=np.float32)
    )
