# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

"""Bimanual ReBot B601 follower driver."""

from __future__ import annotations

from typing import ClassVar

import numpy as np

from physicalai.config import export_config
from physicalai.robot import BimanualRobot, CompositeRobotObservation
from physicalai_rebot_b601_plugin.constants import REBOT_B601_DM_JOINT_ORDER
from physicalai_rebot_b601_plugin.dm import ReBotB601DM

BimanualReBotB601Observation = CompositeRobotObservation


@export_config(class_path="physicalai_rebot_b601_plugin.BimanualReBotB601")
class BimanualReBotB601(BimanualRobot[ReBotB601DM, ReBotB601DM]):
    """Two matching DM or RS followers, with joint vectors ordered left then right.

    Each arm needs its own serial adapter or CAN interface: the motors on both
    arms use the same bus addresses. Per-arm control and parking settings remain
    on the single-arm drivers, available through ``left`` and ``right``.
    """

    JOINT_ORDER: ClassVar[tuple[str, ...]] = tuple(
        f"{side}_{joint}" for side in ("left", "right") for joint in REBOT_B601_DM_JOINT_ORDER
    )
    NUM_JOINTS: ClassVar[int] = len(JOINT_ORDER)

    def __init__(self, left: ReBotB601DM, right: ReBotB601DM) -> None:
        """Initialize two independently connected followers.

        Raises:
            ValueError: If driver types, roles, or transport connections are incompatible.
        """
        if type(left) is not type(right):
            msg = "Both arms must be the same driver type (two DM or two RS arms)."
            raise ValueError(msg)
        if left.role != right.role:
            msg = "Both arms must have the same role."
            raise ValueError(msg)
        if left.port == right.port:
            msg = "Both arms must use separate transport connections."
            raise ValueError(msg)
        super().__init__(left=left, right=right)

    @property
    def role(self) -> str:
        """Shared follower role."""
        return self.left.role

    def send_action(self, action: np.ndarray, *, goal_time: float = 0.1) -> None:
        """Send exactly 14 finite joint targets, in degrees, left then right.

        Raises:
            ValueError: If any target is non-finite.
        """
        # Validate the whole vector before either arm receives a command.
        if not np.all(np.isfinite(action)):
            msg = "Action must be finite."
            raise ValueError(msg)
        super().send_action(action, goal_time=goal_time)

    def disable_torque(self) -> None:
        """Release both arms for manual positioning; support them before calling."""
        try:
            self.left.disable_torque()
        finally:
            self.right.disable_torque()

    def set_zero_position(self) -> None:
        """Store both arms' current poses as zero, disabling torque first."""
        self.disable_torque()
        self.left.set_zero_position()
        self.right.set_zero_position()
