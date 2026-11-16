# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

"""Parity coverage for the optimized gravity kernel, independent of hardware/SDKs."""

from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
from typing import TYPE_CHECKING

import numpy as np
import pytest
from physicalai_rebot_b601_plugin import get_urdf_path
from physicalai_rebot_b601_plugin.constants import (
    REBOT_B601_DM_GRAVITY_URDF,
    REBOT_B601_RS_GRAVITY_URDF,
    REBOT_B601_URDF_ARM_JOINTS,
)
from physicalai_rebot_b601_plugin.gravity import GravityModel

if TYPE_CHECKING:
    from pathlib import Path

    from physicalai_rebot_b601_plugin.gravity import _ActiveJoint


class ReferenceGravityModel(GravityModel):
    """The original NumPy cross/dot kernel, retaining its accumulation order."""

    def _add_holding_torque(
        self,
        torques: np.ndarray,
        link: str,
        com_world: np.ndarray,
        above: tuple[_ActiveJoint, ...],
    ) -> None:
        weight = self._links[link][0] * self._gravity
        for joint in above:
            torques[joint.index] -= float(np.dot(np.cross(joint.axis, com_world - joint.anchor), weight))


@pytest.mark.parametrize("urdf", [REBOT_B601_DM_GRAVITY_URDF, REBOT_B601_RS_GRAVITY_URDF])
@pytest.mark.parametrize("gravity", [(0.0, 0.0, -9.80665), (1.2, -3.4, 5.6), (0.0, 0.0, 0.0)])
def test_random_poses_match_original_kernel(urdf: str, gravity: tuple[float, ...]) -> None:
    path = get_urdf_path() / urdf
    model = GravityModel(path, REBOT_B601_URDF_ARM_JOINTS, gravity=gravity)
    reference = ReferenceGravityModel(path, REBOT_B601_URDF_ARM_JOINTS, gravity=gravity)
    rng = np.random.default_rng(601)
    poses = np.vstack([np.zeros(6), np.full(6, np.pi), rng.uniform(-np.pi, np.pi, (100, 6))])
    for q in poses:
        np.testing.assert_allclose(model.torques(q), reference.torques(q), rtol=1e-13, atol=1e-13)


def test_branched_non_axial_tree_and_reordered_joint_subset(tmp_path: Path) -> None:
    # A branch, a fixed payload and an unactuated revolute joint exercise more than the serial B601.
    path = tmp_path / "branch.urdf"
    path.write_text("""<robot name="branch">
      <link name="base"/>
      <link name="upper"><inertial><mass value="2"/><origin xyz=".2 .1 -.1"/></inertial></link>
      <link name="lower"><inertial><mass value="1"/><origin xyz="-.1 .3 .2"/></inertial></link>
      <link name="side"><inertial><mass value=".5"/><origin xyz=".4 -.2 .1"/></inertial></link>
      <link name="tool"><inertial><mass value=".2"/><origin xyz=".1 .2 .3"/></inertial></link>
      <joint name="root" type="continuous"><parent link="base"/><child link="upper"/>
        <origin xyz=".1 -.2 .3" rpy=".3 .4 -.2"/><axis xyz="1 2 -3"/></joint>
      <joint name="tip" type="revolute"><parent link="upper"/><child link="lower"/>
        <origin xyz=".2 .3 -.1" rpy="-.2 .7 .1"/><axis xyz="-2 1 1"/></joint>
      <joint name="branch" type="revolute"><parent link="upper"/><child link="side"/>
        <origin xyz="-.1 .4 .2" rpy=".5 -.2 .8"/><axis xyz="0 1 0"/></joint>
      <joint name="fixed" type="fixed"><parent link="lower"/><child link="tool"/>
        <origin xyz=".3 .1 .2" rpy=".4 -.3 .5"/></joint>
    </robot>""")
    gravity = (2.5, 1.7, -8.0)
    model = GravityModel(path, ["tip", "root"], gravity=gravity)
    reference = ReferenceGravityModel(path, ["tip", "root"], gravity=gravity)
    for q in np.random.default_rng(42).uniform(-3.0, 3.0, (100, 2)):
        np.testing.assert_allclose(model.torques(q), reference.torques(q), rtol=1e-13, atol=1e-13)


def test_results_remain_owned_and_model_is_reentrant() -> None:
    model = GravityModel(get_urdf_path() / REBOT_B601_RS_GRAVITY_URDF, REBOT_B601_URDF_ARM_JOINTS)
    poses = np.random.default_rng(10).uniform(-2.0, 2.0, (20, 6))
    original = poses.copy()
    expected = [model.torques(q) for q in poses]
    with ThreadPoolExecutor(max_workers=2) as pool:
        actual = list(pool.map(model.torques, poses))
    np.testing.assert_array_equal(poses, original)
    np.testing.assert_allclose(actual, expected, rtol=0.0, atol=0.0)
    actual[0][:] = 123.0
    np.testing.assert_array_equal(model.torques(poses[0]), expected[0])


@pytest.mark.parametrize("gravity", [(1.0, 2.0), (1.0, 2.0, 3.0, 4.0)])
def test_invalid_gravity_length_still_raises(gravity: tuple[float, ...]) -> None:
    # Do not silently drop extra components when expanding the scalar triple product.
    for cls in (GravityModel, ReferenceGravityModel):
        model = cls(get_urdf_path() / REBOT_B601_DM_GRAVITY_URDF, REBOT_B601_URDF_ARM_JOINTS, gravity=gravity)
        with pytest.raises(ValueError):
            model.torques(np.zeros(6))
