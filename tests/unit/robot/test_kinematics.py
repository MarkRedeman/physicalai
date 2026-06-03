# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

"""Tests for kinematics-based robot adapters."""

from __future__ import annotations

import sys
from dataclasses import dataclass
from types import SimpleNamespace
from typing import TYPE_CHECKING
from unittest.mock import MagicMock, patch

import numpy as np
import pytest

from physicalai.robot.interface import Robot
from physicalai.robot.kinematics import InverseKinematicsOptions, KinematicEndEffectorRobot

if TYPE_CHECKING:
    from physicalai.capture.frame import Frame


@dataclass
class _Obs:
    joint_positions: np.ndarray
    timestamp: float
    sensor_data: dict[str, np.ndarray] | None = None
    images: dict[str, Frame] | None = None


class _FakeInternalRobot:
    def __init__(self) -> None:
        self._connected = False
        self._positions = np.array([0.0, 0.5], dtype=np.float32)
        self.sent_action: np.ndarray | None = None
        self.sent_goal_time: float | None = None

    @property
    def joint_names(self) -> list[str]:
        return ["joint1", "gripper"]

    def connect(self) -> None:
        self._connected = True

    def disconnect(self) -> None:
        self._connected = False

    def is_connected(self) -> bool:
        return self._connected

    def get_observation(self) -> _Obs:
        return _Obs(
            joint_positions=self._positions.copy(),
            timestamp=1.25,
            sensor_data={"velocities": np.array([0.0, 0.0], dtype=np.float32)},
        )

    def send_action(self, action: np.ndarray, *, goal_time: float = 0.1) -> None:
        self.sent_action = action.copy()
        self.sent_goal_time = goal_time
        self._positions = action.astype(np.float32)


class _FakeJoint:
    def __init__(self, idx_q: int, idx_v: int) -> None:
        self.idx_q = idx_q
        self.idx_v = idx_v
        self.nq = 1
        self.nv = 1


class _FakeSE3:
    def __init__(self, rotation: np.ndarray, translation: np.ndarray) -> None:
        self.rotation = rotation
        self.translation = np.asarray(translation, dtype=np.float64)

    def actInv(self, target: _FakeSE3) -> SimpleNamespace:  # noqa: N802
        error = np.zeros(6, dtype=np.float64)
        error[:3] = target.translation - self.translation
        return SimpleNamespace(vector=error)


class _FakeQuaternion:
    def __init__(self, _rotation: np.ndarray) -> None:
        pass

    def coeffs(self) -> np.ndarray:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)


def _make_fake_pinocchio() -> SimpleNamespace:
    model = SimpleNamespace(
        names=["universe", "joint1"],
        joints=[_FakeJoint(-1, -1), _FakeJoint(0, 0)],
        nframes=1,
        nv=1,
        lowerPositionLimit=np.array([-np.pi], dtype=np.float64),
        upperPositionLimit=np.array([np.pi], dtype=np.float64),
    )
    model.getFrameId = lambda name: 0 if name == "ee_link" else model.nframes
    model.getJointId = lambda name: 1 if name == "joint1" else len(model.joints)
    model.createData = lambda: SimpleNamespace(oMf=[_FakeSE3(np.eye(3), np.zeros(3))], q=np.zeros(1))

    def forward_kinematics(_model: object, data: SimpleNamespace, q: np.ndarray) -> None:
        data.q = q.copy()

    def update_frame_placements(_model: object, data: SimpleNamespace) -> None:
        data.oMf[0] = _FakeSE3(np.eye(3), np.array([data.q[0], 0.0, 0.0], dtype=np.float64))

    def compute_frame_jacobian(
        _model: object,
        _data: object,
        _q: np.ndarray,
        _frame_id: int,
        _reference_frame: object,
    ) -> np.ndarray:
        jacobian = np.zeros((6, 1), dtype=np.float64)
        jacobian[0, 0] = 1.0
        return jacobian

    return SimpleNamespace(
        buildModelFromUrdf=lambda _path: model,
        neutral=lambda _model: np.zeros(1, dtype=np.float64),
        forwardKinematics=forward_kinematics,
        updateFramePlacements=update_frame_placements,
        computeFrameJacobian=compute_frame_jacobian,
        integrate=lambda _model, q, delta: q + delta,
        Quaternion=_FakeQuaternion,
        SE3=_FakeSE3,
        log=lambda err: err,
        ReferenceFrame=SimpleNamespace(LOCAL=object()),
    )


@pytest.fixture
def fake_pinocchio() -> SimpleNamespace:
    return _make_fake_pinocchio()


def _create_adapter(fake_pinocchio: SimpleNamespace, internal: _FakeInternalRobot | None = None) -> KinematicEndEffectorRobot:
    if internal is None:
        internal = _FakeInternalRobot()
    with patch.dict(sys.modules, {"pinocchio": fake_pinocchio}):
        return KinematicEndEffectorRobot(
            internal_robot=internal,
            urdf_path="/tmp/fake.urdf",
            end_effector_frame="ee_link",
            controlled_joint_names=["joint1"],
        )


def test_adapter_satisfies_robot_protocol(fake_pinocchio: SimpleNamespace) -> None:
    adapter = _create_adapter(fake_pinocchio)

    assert isinstance(adapter, Robot)


def test_lifecycle_delegates_to_internal_robot(fake_pinocchio: SimpleNamespace) -> None:
    internal = _FakeInternalRobot()
    adapter = _create_adapter(fake_pinocchio, internal)

    adapter.connect()
    assert adapter.is_connected()

    adapter.disconnect()
    assert not adapter.is_connected()


def test_get_observation_returns_end_effector_pose(fake_pinocchio: SimpleNamespace) -> None:
    internal = _FakeInternalRobot()
    internal._positions = np.array([90.0, 0.5], dtype=np.float32)  # noqa: SLF001
    adapter = _create_adapter(fake_pinocchio, internal)

    obs = adapter.get_observation()

    assert adapter.joint_names == ["x", "y", "z", "qx", "qy", "qz", "qw"]
    assert obs.joint_positions.shape == (7,)
    assert obs.joint_positions[0] == pytest.approx(np.pi / 2, abs=1e-6)
    np.testing.assert_allclose(obs.joint_positions[3:], np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32))
    assert obs.sensor_data is not None
    np.testing.assert_allclose(obs.sensor_data["internal_joint_positions"], np.array([90.0, 0.5], dtype=np.float32))
    assert obs.internal_observation is not None
    np.testing.assert_allclose(obs.internal_observation.joint_positions, np.array([90.0, 0.5], dtype=np.float32))


def test_send_action_solves_ik_and_preserves_gripper(fake_pinocchio: SimpleNamespace) -> None:
    internal = _FakeInternalRobot()
    adapter = _create_adapter(fake_pinocchio, internal)
    target = np.array([np.pi / 2, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0], dtype=np.float32)

    adapter.send_action(target, goal_time=0.2)

    assert internal.sent_action is not None
    assert internal.sent_action[0] == pytest.approx(90.0, abs=1e-3)
    assert internal.sent_action[1] == pytest.approx(0.5, abs=1e-6)
    assert internal.sent_goal_time == pytest.approx(0.2)


def test_send_action_rejects_short_action(fake_pinocchio: SimpleNamespace) -> None:
    adapter = _create_adapter(fake_pinocchio)

    with pytest.raises(ValueError, match="Expected at least 7 action dims"):
        adapter.send_action(np.zeros(6, dtype=np.float32))


def test_send_action_rejects_zero_quaternion(fake_pinocchio: SimpleNamespace) -> None:
    adapter = _create_adapter(fake_pinocchio)

    with pytest.raises(ValueError, match="Quaternion"):
        adapter.send_action(np.zeros(7, dtype=np.float32))


def test_inverse_kinematics_options_validate_values() -> None:
    with pytest.raises(ValueError, match="max_iterations"):
        InverseKinematicsOptions(max_iterations=0)

    with pytest.raises(ValueError, match="tolerance"):
        InverseKinematicsOptions(tolerance=0.0)

    with pytest.raises(ValueError, match="damping"):
        InverseKinematicsOptions(damping=-1.0)


def test_missing_pinocchio_has_helpful_error() -> None:
    real_import_module = __import__("importlib").import_module

    def import_module(name: str) -> object:
        if name == "pinocchio":
            raise ImportError("missing")
        return real_import_module(name)

    with (
        patch("physicalai.robot.kinematics.importlib.import_module", side_effect=import_module),
        pytest.raises(ImportError, match=r"physicalai\[kinematics\]"),
    ):
        KinematicEndEffectorRobot(
            internal_robot=_FakeInternalRobot(),
            urdf_path="/tmp/fake.urdf",
            end_effector_frame="ee_link",
        )


def test_invalid_controlled_joint_raises(fake_pinocchio: SimpleNamespace) -> None:
    with patch.dict(sys.modules, {"pinocchio": fake_pinocchio}), pytest.raises(ValueError, match="missing from internal"):
        KinematicEndEffectorRobot(
            internal_robot=_FakeInternalRobot(),
            urdf_path="/tmp/fake.urdf",
            end_effector_frame="ee_link",
            controlled_joint_names=["missing_joint"],
        )


def test_public_lazy_export(fake_pinocchio: SimpleNamespace) -> None:
    with patch.dict(sys.modules, {"pinocchio": fake_pinocchio}):
        from physicalai.robot import KinematicEndEffectorRobot as ExportedRobot

    assert ExportedRobot is KinematicEndEffectorRobot
