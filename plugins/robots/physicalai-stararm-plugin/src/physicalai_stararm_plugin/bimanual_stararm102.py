# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

"""Bimanual Star Arm 102 leader and follower driver."""

from __future__ import annotations

from typing import ClassVar

from physicalai.config import export_config
from physicalai.robot import BimanualRobot, CompositeRobotObservation
from physicalai_stararm_plugin.constants import STAR_ARM_102_JOINT_ORDER
from physicalai_stararm_plugin.stararm102fl import StarArm102FLFollower
from physicalai_stararm_plugin.stararm102hd import StarArm102HDLeader

StarArm102 = StarArm102HDLeader | StarArm102FLFollower
BimanualStarArm102Observation = CompositeRobotObservation


@export_config(class_path="physicalai_stararm_plugin.BimanualStarArm102")
class BimanualStarArm102(BimanualRobot[StarArm102, StarArm102]):
    """Two matching LD leaders, HD leaders, or FL followers on separate UART ports.

    Joint vectors contain 14 values, left then right. Leaders retain their
    single-arm passive/assist behavior and follower-frame conversion. Both
    leaders must use the same control mode and follower profile.
    """

    JOINT_ORDER: ClassVar[tuple[str, ...]] = tuple(
        f"{side}_{joint}" for side in ("left", "right") for joint in STAR_ARM_102_JOINT_ORDER
    )
    NUM_JOINTS: ClassVar[int] = len(JOINT_ORDER)

    def __init__(self, left: StarArm102, right: StarArm102) -> None:
        """Initialize two arms of the same concrete variant.

        Raises:
            ValueError: If variants, ports, leader modes, or follower profiles differ.
        """
        if type(left) is not type(right):
            msg = "Both arms must be the same driver type (two LD, HD, or FL arms)."
            raise ValueError(msg)
        if left.port == right.port:
            msg = "Both arms must use separate transport connections."
            raise ValueError(msg)
        if isinstance(left, StarArm102HDLeader) and isinstance(right, StarArm102HDLeader):
            if left.control_mode != right.control_mode:
                msg = "Both leaders must have the same control mode."
                raise ValueError(msg)
            if left.follower_profile != right.follower_profile:
                msg = "Both leaders must have the same follower profile."
                raise ValueError(msg)
        super().__init__(left=left, right=right)

    @property
    def role(self) -> str:
        """Shared leader or follower role."""
        return "leader" if isinstance(self.left, StarArm102HDLeader) else "follower"

    def disable_torque(self) -> None:
        """Unlock both arms so they can be positioned by hand."""
        try:
            self.left.disable_torque()
        finally:
            self.right.disable_torque()

    def set_zero_position(self) -> None:
        """Store both arms' current poses as servo origins, unlocking them first."""
        self.disable_torque()
        self.left.set_zero_position()
        self.right.set_zero_position()
