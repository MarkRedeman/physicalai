# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

"""Studio configuration for bimanual Star Arm 102 variants."""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
from typing import Generic, TypeVar

from physicalai_studio_plugin import (
    CatalogRobotFactory,
    PayloadContainer,
    PortScanner,
    RobotCatalogDefinition,
    RobotZeroCalibration,
    SerialPortInfo,
)
from pydantic import BaseModel, Field

from physicalai_stararm_plugin import BimanualStarArm102
from physicalai_stararm_plugin.studio_catalog import (
    _STAR_ARM_PROBE,
    StarArm102FLPayload,
    StarArm102HDPayload,
    StarArm102LDPayload,
    _build_stararm_102_fl_driver,
    _build_stararm_102_hd_driver,
    _build_stararm_102_ld_driver,
)

_PayloadT = TypeVar("_PayloadT", bound=BaseModel)


@dataclass
class _Arm(Generic[_PayloadT]):
    payload: _PayloadT


class BimanualStarArm102LDPayload(BaseModel):
    """Independent connection and follower-profile settings for two LD leaders."""

    left: StarArm102LDPayload = Field(title="Left arm")
    right: StarArm102LDPayload = Field(title="Right arm")


class BimanualStarArm102HDPayload(BaseModel):
    """Independent connection and control settings for two HD leaders."""

    left: StarArm102HDPayload = Field(title="Left arm")
    right: StarArm102HDPayload = Field(title="Right arm")


class BimanualStarArm102FLPayload(BaseModel):
    """Independent connection and control settings for two FL followers."""

    left: StarArm102FLPayload = Field(title="Left arm")
    right: StarArm102FLPayload = Field(title="Right arm")


BimanualStarArmPayload = BimanualStarArm102LDPayload | BimanualStarArm102HDPayload | BimanualStarArm102FLPayload


class BimanualStarArmProbe:
    """Reuse serial discovery and require both arms to be present."""

    async def discover(self, manager: PortScanner) -> list[SerialPortInfo]:
        """Discover available serial devices.

        Returns:
            Available serial devices from the scanner.
        """
        _ = self
        return await _STAR_ARM_PROBE.discover(manager)

    async def identify(
        self,
        payload: BimanualStarArmPayload,
        manager: PortScanner | None = None,
        joint: str | None = None,
    ) -> None:
        """Do not move arms: visual identification is not supported."""
        _ = self, payload, manager, joint

    async def is_online(self, payload: BimanualStarArmPayload, manager: PortScanner | None = None) -> bool:
        """Check both configured UART connections without opening them.

        Returns:
            Whether both single-arm probes report online.
        """
        _ = self
        online = await asyncio.gather(
            _STAR_ARM_PROBE.is_online(payload.left, manager), _STAR_ARM_PROBE.is_online(payload.right, manager)
        )
        return all(online)


_BIMANUAL_PROBE = BimanualStarArmProbe()


async def _build_bimanual_ld(
    robot: PayloadContainer[BimanualStarArm102LDPayload], factory: CatalogRobotFactory
) -> BimanualStarArm102:
    payload = BimanualStarArm102LDPayload.model_validate(robot.payload)
    left = await _build_stararm_102_ld_driver(_Arm(payload.left), factory)
    right = await _build_stararm_102_ld_driver(_Arm(payload.right), factory)
    return BimanualStarArm102(left=left, right=right)


async def _build_bimanual_hd(
    robot: PayloadContainer[BimanualStarArm102HDPayload], factory: CatalogRobotFactory
) -> BimanualStarArm102:
    payload = BimanualStarArm102HDPayload.model_validate(robot.payload)
    left = await _build_stararm_102_hd_driver(_Arm(payload.left), factory)
    right = await _build_stararm_102_hd_driver(_Arm(payload.right), factory)
    return BimanualStarArm102(left=left, right=right)


async def _build_bimanual_fl(
    robot: PayloadContainer[BimanualStarArm102FLPayload], factory: CatalogRobotFactory
) -> BimanualStarArm102:
    payload = BimanualStarArm102FLPayload.model_validate(robot.payload)
    left = await _build_stararm_102_fl_driver(_Arm(payload.left), factory)
    right = await _build_stararm_102_fl_driver(_Arm(payload.right), factory)
    return BimanualStarArm102(left=left, right=right)


async def _release(robot: BimanualStarArm102) -> None:
    await asyncio.to_thread(robot.disable_torque)


async def _set_zero(robot: BimanualStarArm102) -> None:
    await asyncio.to_thread(robot.set_zero_position)


_LEADER_ZERO_CALIBRATION = RobotZeroCalibration[BimanualStarArm102](
    instructions=(
        "Move both leaders into their folded rest (zero) poses with both grippers fully closed. "
        "Hold them still, then set zero for both arms."
    ),
    release=_release,
    set_zero=_set_zero,
)


def get_bimanual_definitions() -> list[RobotCatalogDefinition]:
    """Return LD/HD leader and FL follower bimanual definitions."""
    return [
        RobotCatalogDefinition(
            type="StarArm_Bimanual_102_LD_Leader",
            display_name="Star Arm Bimanual 102-LD Leader",
            role="leader",
            robot_builder=_build_bimanual_ld,
            robot_payload=BimanualStarArm102LDPayload,
            probe=_BIMANUAL_PROBE,
            zero_calibration=_LEADER_ZERO_CALIBRATION,
        ),
        RobotCatalogDefinition(
            type="StarArm_Bimanual_102_HD_Leader",
            display_name="Star Arm Bimanual 102-HD Leader",
            role="leader",
            robot_builder=_build_bimanual_hd,
            robot_payload=BimanualStarArm102HDPayload,
            probe=_BIMANUAL_PROBE,
            zero_calibration=_LEADER_ZERO_CALIBRATION,
        ),
        RobotCatalogDefinition(
            type="StarArm_Bimanual_102_FL_Follower",
            display_name="Star Arm Bimanual 102-FL Follower",
            role="follower",
            robot_builder=_build_bimanual_fl,
            robot_payload=BimanualStarArm102FLPayload,
            probe=_BIMANUAL_PROBE,
        ),
    ]
