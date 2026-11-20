# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

"""Studio configuration for two independently connected ReBot B601 arms."""

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

from physicalai_rebot_b601_plugin import BimanualReBotB601
from physicalai_rebot_b601_plugin.studio_catalog import (
    _REBOT_PROBE,
    _REBOT_RS_PROBE,
    ReBotB601DMPayload,
    ReBotB601RSPayload,
    _build_rebot_b601_dm_driver,
    _build_rebot_b601_rs_driver,
)

_PayloadT = TypeVar("_PayloadT", bound=BaseModel)


@dataclass
class _Arm(Generic[_PayloadT]):
    payload: _PayloadT


class BimanualReBotB601DMPayload(BaseModel):
    """Independent connection and control settings for two DM followers."""

    left: ReBotB601DMPayload = Field(title="Left arm")
    right: ReBotB601DMPayload = Field(title="Right arm")


class BimanualReBotB601RSPayload(BaseModel):
    """Independent SocketCAN and control settings for two RS followers."""

    left: ReBotB601RSPayload = Field(title="Left arm")
    right: ReBotB601RSPayload = Field(title="Right arm")


class BimanualReBotProbe:
    """Reuse single-arm discovery and require both connections to be online."""

    def __init__(self, *, socketcan: bool = False) -> None:
        """Select serial discovery or SocketCAN interface checks."""
        self._socketcan = socketcan

    async def discover(self, manager: PortScanner) -> list[SerialPortInfo]:
        """Discover serial adapters; SocketCAN interfaces are configured manually.

        Returns:
            Available serial adapters, or an empty list for SocketCAN.
        """
        if self._socketcan:
            return []
        return await _REBOT_PROBE.discover(manager)

    async def identify(
        self,
        payload: BimanualReBotB601DMPayload | BimanualReBotB601RSPayload,
        manager: PortScanner | None = None,
        joint: str | None = None,
    ) -> None:
        """Do not move arms: visual identification is not supported."""
        _ = self, payload, manager, joint

    async def is_online(
        self,
        payload: BimanualReBotB601DMPayload | BimanualReBotB601RSPayload,
        manager: PortScanner | None = None,
    ) -> bool:
        """Check both adapters or CAN interfaces without connecting to hardware.

        Returns:
            Whether both single-arm probes report online.
        """
        _ = self
        if isinstance(payload, BimanualReBotB601DMPayload):
            online = await asyncio.gather(
                _REBOT_PROBE.is_online(payload.left, manager), _REBOT_PROBE.is_online(payload.right, manager)
            )
        else:
            online = await asyncio.gather(
                _REBOT_RS_PROBE.is_online(payload.left, manager), _REBOT_RS_PROBE.is_online(payload.right, manager)
            )
        return all(online)


async def _build_bimanual_dm(
    robot: PayloadContainer[BimanualReBotB601DMPayload], factory: CatalogRobotFactory
) -> BimanualReBotB601:
    payload = BimanualReBotB601DMPayload.model_validate(robot.payload)
    left = await _build_rebot_b601_dm_driver(_Arm(payload.left), factory)
    right = await _build_rebot_b601_dm_driver(_Arm(payload.right), factory)
    return BimanualReBotB601(left=left, right=right)


async def _build_bimanual_rs(
    robot: PayloadContainer[BimanualReBotB601RSPayload], factory: CatalogRobotFactory
) -> BimanualReBotB601:
    payload = BimanualReBotB601RSPayload.model_validate(robot.payload)
    left = await _build_rebot_b601_rs_driver(_Arm(payload.left), factory)
    right = await _build_rebot_b601_rs_driver(_Arm(payload.right), factory)
    return BimanualReBotB601(left=left, right=right)


async def _release(robot: BimanualReBotB601) -> None:
    await asyncio.to_thread(robot.disable_torque)


async def _set_zero(robot: BimanualReBotB601) -> None:
    await asyncio.to_thread(robot.set_zero_position)


_ZERO_CALIBRATION = RobotZeroCalibration[BimanualReBotB601](
    instructions=(
        "Support both arms while torque is off. Move both into their folded rest (zero) poses "
        "with both grippers fully closed. Hold them still, then set zero for both arms."
    ),
    release=_release,
    set_zero=_set_zero,
)


def get_bimanual_definitions() -> list[RobotCatalogDefinition]:
    """Return DM and RS bimanual follower definitions."""
    return [
        RobotCatalogDefinition(
            type="ReBot_Bimanual_B601_DM_Follower",
            display_name="ReBot Bimanual B601 DM Follower",
            role="follower",
            robot_builder=_build_bimanual_dm,
            robot_payload=BimanualReBotB601DMPayload,
            probe=BimanualReBotProbe(),
            zero_calibration=_ZERO_CALIBRATION,
        ),
        RobotCatalogDefinition(
            type="ReBot_Bimanual_B601_RS_Follower",
            display_name="ReBot Bimanual B601 RS Follower",
            role="follower",
            robot_builder=_build_bimanual_rs,
            robot_payload=BimanualReBotB601RSPayload,
            probe=BimanualReBotProbe(socketcan=True),
            zero_calibration=_ZERO_CALIBRATION,
        ),
    ]
