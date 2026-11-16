# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

"""Independent numerical references and fake-transport hot-path contracts."""

from __future__ import annotations

from dataclasses import replace
from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np
import pytest

from physicalai_so101_plugin._conversion import CalibrationTables
from physicalai_so101_plugin.calibration import SO101Calibration
from physicalai_so101_plugin.constants import SO101_JOINT_ORDER


def make_calibration() -> SO101Calibration:
    return SO101Calibration.from_dict({
        name: {"id": i, "drive_mode": i % 2, "homing_offset": 2048, "range_min": i * 117, "range_max": 4095 - i * 53}
        for i, name in enumerate(SO101_JOINT_ORDER, 1)
    })


def reference_normalized(ticks: np.ndarray, tables: CalibrationTables) -> np.ndarray:
    """Freeze the pre-optimization arithmetic, not the convenience wrapper."""
    raw = np.asarray(ticks, dtype=np.float64)
    clipped = np.clip(raw, tables.range_min, tables.range_max)
    span = np.where(tables.valid, tables.span, 1.0)
    body = (clipped - tables.range_min) / span * 200.0 - 100.0
    gripper = (clipped - tables.range_min) / span * 100.0
    values = np.where(tables.gripper_mask, np.clip(gripper, 0.0, 100.0), np.clip(body, -100.0, 100.0))
    return np.where(tables.valid, values, 0.0).astype(np.float32)


@pytest.mark.parametrize("dtype", [np.int32, np.float32, np.float64])
def test_normalized_encoder_domain_exact_parity(dtype: type) -> None:
    tables = CalibrationTables.from_calibration(make_calibration())
    ticks = np.repeat(np.arange(-1, 4097, dtype=dtype)[:, None], 6, axis=1)
    actual = tables.ticks_to_normalized(ticks)
    assert actual.dtype == np.float32
    np.testing.assert_array_equal(actual, reference_normalized(ticks, tables))


@pytest.mark.parametrize("invalid_range", [(1200, 1200), (1500, 1000)])
def test_normalized_special_values_and_invalid_ranges(invalid_range: tuple[int, int]) -> None:
    calibration = make_calibration()
    # Typed construction also supports degenerate legacy calibration entries.
    joints = dict(calibration.joints)
    joints["elbow_flex"] = replace(joints["elbow_flex"], range_min=invalid_range[0], range_max=invalid_range[1])
    tables = CalibrationTables.from_calibration(SO101Calibration(joints))
    rng = np.random.default_rng(42)
    ticks = rng.uniform(-10000, 10000, size=(1000, 12))[:, ::2]
    ticks[0] = [np.nan, np.inf, -np.inf, -0.0, 1e100, -1e100]
    np.testing.assert_array_equal(tables.ticks_to_normalized(ticks), reference_normalized(ticks, tables))


def test_normalized_output_has_independent_storage() -> None:
    tables = CalibrationTables.from_calibration(make_calibration())
    ticks = np.full(6, 2048, dtype=np.int32)
    original = ticks.copy()
    first = tables.ticks_to_normalized(ticks)
    expected = first.copy()
    second = tables.ticks_to_normalized(ticks)
    second[:] = 99
    np.testing.assert_array_equal(first, expected)
    np.testing.assert_array_equal(ticks, original)
    assert not np.shares_memory(first, second)


@pytest.mark.parametrize("unit", ["normalized", "degrees"])
def test_steady_state_control_sdk_calls_and_observation_ownership(unit: str) -> None:
    from physicalai_so101_plugin.so101 import SO101

    robot = SO101(port="unused-fake-port", calibration=make_calibration(), unit=unit)
    read = MagicMock()
    read.txRxPacket.return_value = 0
    read.isAvailable.return_value = True
    read.getData.return_value = 2048
    write = MagicMock()
    write.addParam.return_value = True
    write.txPacket.return_value = 0
    torque = MagicMock()
    robot._connection = SimpleNamespace(
        port_handler=None, group_sync_read=read, group_sync_write=write, group_sync_write_torque=torque
    )
    robot._torque_enabled = True
    first = robot.get_observation()
    expected = first.joint_positions.copy()
    robot.send_action(np.zeros(6, dtype=np.float32))
    read.txRxPacket.assert_called_once_with()
    assert read.isAvailable.call_count == 6
    assert read.getData.call_count == 6
    write.clearParam.assert_called_once_with()
    assert write.addParam.call_count == 6
    write.txPacket.assert_called_once_with()
    assert not torque.mock_calls
    second = robot.get_observation()
    second.joint_positions[:] = 17
    np.testing.assert_array_equal(first.joint_positions, expected)
    assert first.joint_positions.dtype == np.float32
