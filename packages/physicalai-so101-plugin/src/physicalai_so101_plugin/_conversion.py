# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

"""Shared already-homed tick conversion; hardware offsets are not applied twice."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

from physicalai_so101_plugin.constants import SO101_JOINT_ORDER, TICKS_PER_REVOLUTION

if TYPE_CHECKING:
    from physicalai_so101_plugin.calibration import SO101Calibration

_DEGREES_PER_TICK = 360.0 / (TICKS_PER_REVOLUTION - 1)
_INT32_MIN = float(np.iinfo(np.int32).min)
_INT32_MAX = float(np.iinfo(np.int32).max)


@dataclass(frozen=True, slots=True)
class CalibrationTables:
    """Per-joint conversion constants in canonical order.

    Built once per driver instead of looking up calibration entries and doing
    scalar NumPy operations for every joint on every read/write.
    """

    range_min: np.ndarray
    range_max: np.ndarray
    span: np.ndarray
    midpoint: np.ndarray
    homing_offset: np.ndarray
    gripper_mask: np.ndarray
    valid: np.ndarray

    @classmethod
    def from_calibration(cls, calibration: SO101Calibration) -> CalibrationTables:
        """Precompute conversion vectors in :data:`SO101_JOINT_ORDER` order.

        Returns:
            Calibration tables in canonical joint order.
        """
        joints = [calibration.joints[name] for name in SO101_JOINT_ORDER]
        range_min = np.array([joint.range_min for joint in joints], dtype=np.float64)
        range_max = np.array([joint.range_max for joint in joints], dtype=np.float64)
        span = range_max - range_min
        return cls(
            range_min=range_min,
            range_max=range_max,
            span=span,
            midpoint=(range_min + range_max) / 2.0,
            homing_offset=np.array([joint.homing_offset for joint in joints], dtype=np.float64),
            gripper_mask=np.array([name == "gripper" for name in SO101_JOINT_ORDER], dtype=bool),
            valid=span > 0,
        )

    def _require_valid(self) -> None:
        if np.all(self.valid):
            return
        for name, valid in zip(SO101_JOINT_ORDER, self.valid, strict=True):
            if not valid:
                msg = f"Motor {name} requires an ordered, nonzero calibration range for degrees"
                raise ValueError(msg)

    def ticks_to_normalized(self, ticks: np.ndarray) -> np.ndarray:
        """Canonical float32 normalized positions; matches the legacy helper.

        Returns:
            Float32 normalized positions in canonical order.
        """
        raw = np.asarray(ticks, dtype=np.float64)
        clipped = np.clip(raw, self.range_min, self.range_max)
        span = np.where(self.valid, self.span, 1.0)
        body = (clipped - self.range_min) / span * 200.0 - 100.0
        gripper = (clipped - self.range_min) / span * 100.0
        values = np.where(
            self.gripper_mask,
            np.clip(gripper, 0.0, 100.0),
            np.clip(body, -100.0, 100.0),
        )
        return np.where(self.valid, values, 0.0).astype(np.float32)

    def ticks_to_degrees(self, ticks: np.ndarray) -> np.ndarray:
        """Canonical float32 body degrees with gripper percentage.

        Returns:
            Float32 body degrees and gripper percentage in canonical order.
        """
        self._require_valid()
        raw = np.asarray(ticks, dtype=np.float64)
        clipped = np.clip(raw, self.range_min, self.range_max)
        gripper = (clipped - self.range_min) / self.span * 100.0
        body = (raw - self.midpoint) * _DEGREES_PER_TICK
        return np.where(self.gripper_mask, gripper, body).astype(np.float32)

    def normalized_to_ticks(self, values: np.ndarray) -> np.ndarray:
        """Convert normalized commands to int32 ticks with the legacy rounding.

        Returns:
            Int32 tick commands in canonical order.

        Raises:
            ValueError: If any command is not finite.
        """
        raw = np.asarray(values, dtype=np.float64)
        if not np.all(np.isfinite(raw)):
            msg = "Normalized actions must contain only finite values"
            raise ValueError(msg)
        lower = np.where(self.gripper_mask, 0.0, -100.0)
        upper = np.where(self.gripper_mask, 100.0, 100.0)
        clamped = np.clip(raw, lower, upper)
        span = np.where(self.valid, self.span, 1.0)
        fraction = np.where(self.gripper_mask, clamped / 100.0, (clamped + 100.0) / 200.0)
        ticks = np.round(self.range_min + fraction * span)
        ticks = np.clip(ticks, self.range_min, self.range_max)
        fallback = np.clip(self.homing_offset, self.range_min, self.range_max)
        return np.where(self.valid, ticks, fallback).astype(np.int32)

    def degrees_to_ticks(self, values: np.ndarray) -> np.ndarray:
        """Convert degree commands to int32 ticks with the legacy truncation.

        Returns:
            Int32 tick commands in canonical order.

        Raises:
            ValueError: If a command is not finite or a joint has no positive span.
        """
        raw = np.asarray(values, dtype=np.float64)
        if not np.all(np.isfinite(raw)):
            msg = "Degree-mode actions must contain only finite values"
            raise ValueError(msg)
        self._require_valid()
        gripper_fraction = np.clip(raw, 0.0, 100.0) / 100.0
        gripper_ticks = gripper_fraction * self.span + self.range_min
        with np.errstate(over="ignore", invalid="ignore"):
            body_ticks = raw * (TICKS_PER_REVOLUTION - 1) / 360.0 + self.midpoint
        ticks = np.where(self.gripper_mask, gripper_ticks, body_ticks)
        return np.clip(ticks, _INT32_MIN, _INT32_MAX).astype(np.int32)


def ticks_to_normalized(ticks: np.ndarray, calibration: SO101Calibration) -> np.ndarray:
    """Convert positions with the normal driver's ordering, clipping and dtype.

    Returns:
        Canonically ordered float32 normalized positions.
    """
    return CalibrationTables.from_calibration(calibration).ticks_to_normalized(ticks)


def ticks_to_degrees(ticks: np.ndarray, calibration: SO101Calibration) -> np.ndarray:
    """Use LeRobot's body degree convention, keeping gripper percentage units.

    Body zero is the recorded range midpoint, not the EEPROM homing offset.
    The encoder denominator is 4095 and body reads are NOT range-clipped.

    Returns:
        Float32 body degrees and gripper percentage in canonical order.

    Raises:
        ValueError: If a recorded range has no positive span.
    """
    return CalibrationTables.from_calibration(calibration).ticks_to_degrees(ticks)


def degrees_to_ticks(values: np.ndarray, calibration: SO101Calibration) -> np.ndarray:
    """Convert body degrees and gripper percentage with LeRobot's truncation.

    Do not clamp body commands to recorded ranges. The driver's existing final
    encoder safety clamp still applies before hardware writes.

    Returns:
        Int32 servo ticks in canonical order.

    Raises:
        ValueError: If a recorded range has no positive span or inputs are not finite.
    """
    return CalibrationTables.from_calibration(calibration).degrees_to_ticks(values)
