# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

"""Parity tests for the precomputed conversion tables used by the hot path."""

from __future__ import annotations

import numpy as np
import pytest

from physicalai_so101_plugin._conversion import (
    CalibrationTables,
    degrees_to_ticks,
    ticks_to_degrees,
    ticks_to_normalized,
)
from physicalai_so101_plugin.calibration import SO101Calibration

SAMPLE_CALIBRATION = {
    "shoulder_pan": {"id": 1, "drive_mode": 0, "homing_offset": 2048, "range_min": 707, "range_max": 3439},
    "shoulder_lift": {"id": 2, "drive_mode": 1, "homing_offset": 1024, "range_min": 669, "range_max": 3292},
    "elbow_flex": {"id": 3, "drive_mode": 0, "homing_offset": 2048, "range_min": 846, "range_max": 3069},
    "wrist_flex": {"id": 4, "drive_mode": 0, "homing_offset": 2048, "range_min": 956, "range_max": 3311},
    "wrist_roll": {"id": 5, "drive_mode": 0, "homing_offset": 2048, "range_min": 59, "range_max": 3946},
    "gripper": {"id": 6, "drive_mode": 0, "homing_offset": 2048, "range_min": 2026, "range_max": 3074},
}
TICKS = np.array([0, 100, 707, 2048, 3439, 4095], dtype=np.int32)


@pytest.fixture
def calibration() -> SO101Calibration:
    return SO101Calibration.from_dict(SAMPLE_CALIBRATION)


@pytest.fixture
def tables(calibration: SO101Calibration) -> CalibrationTables:
    return CalibrationTables.from_calibration(calibration)


def test_normalized_matches_legacy(
    tables: CalibrationTables, calibration: SO101Calibration
) -> None:
    np.testing.assert_allclose(
        tables.ticks_to_normalized(TICKS),
        ticks_to_normalized(TICKS, calibration),
        rtol=0,
        atol=1e-6,
    )


def test_degrees_matches_legacy(
    tables: CalibrationTables, calibration: SO101Calibration
) -> None:
    np.testing.assert_allclose(
        tables.ticks_to_degrees(TICKS),
        ticks_to_degrees(TICKS, calibration),
        rtol=0,
        atol=1e-5,
    )


def test_degree_actions_match_legacy(
    tables: CalibrationTables, calibration: SO101Calibration
) -> None:
    values = np.array([-180.0, -90.0, 0.0, 90.0, 180.0, 50.0], dtype=np.float64)

    np.testing.assert_array_equal(
        tables.degrees_to_ticks(values),
        degrees_to_ticks(values, calibration),
    )


def test_normalized_round_trip_within_one_tick(tables: CalibrationTables) -> None:
    ranges = list(zip(tables.range_min.tolist(), tables.range_max.tolist(), strict=True))
    ticks = np.array(
        [(low + high) / 2 for low, high in ranges],
        dtype=np.int32,
    )

    recovered = tables.normalized_to_ticks(tables.ticks_to_normalized(ticks))

    assert np.max(np.abs(recovered - ticks)) <= 1


def test_normalized_rejects_non_finite(tables: CalibrationTables) -> None:
    values = np.zeros(6, dtype=np.float64)
    values[5] = np.nan

    with pytest.raises(ValueError, match="finite"):
        tables.normalized_to_ticks(values)


def test_degrees_reject_non_finite(tables: CalibrationTables) -> None:
    values = np.zeros(6, dtype=np.float64)
    values[0] = np.inf

    with pytest.raises(ValueError, match="finite"):
        tables.degrees_to_ticks(values)
