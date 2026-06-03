# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

"""Kinematics-based robot adapters."""

from __future__ import annotations

import importlib
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Literal

import numpy as np

from physicalai.robot.interface import Robot

if TYPE_CHECKING:
    from types import ModuleType

    from physicalai.capture.frame import Frame
    from physicalai.robot.interface import RobotObservation


POSE_JOINT_NAMES = ["x", "y", "z", "qx", "qy", "qz", "qw"]


@dataclass
class InverseKinematicsOptions:
    """Options for iterative inverse kinematics.

    Attributes:
        max_iterations: Maximum number of damped least-squares IK updates.
        tolerance: Pose error norm threshold for IK convergence.
        damping: Damping coefficient for least-squares IK.
    """

    max_iterations: int = 100
    tolerance: float = 1e-4
    damping: float = 1e-6

    def __post_init__(self) -> None:
        """Validate IK option values.

        Raises:
            ValueError: If any option value is invalid.
        """
        if self.max_iterations <= 0:
            msg = "max_iterations must be positive."
            raise ValueError(msg)
        if self.tolerance <= 0:
            msg = "tolerance must be positive."
            raise ValueError(msg)
        if self.damping < 0:
            msg = "damping must be non-negative."
            raise ValueError(msg)


@dataclass
class KinematicSafetyOptions:
    """Safety guardrails for Cartesian IK actions.

    Attributes:
        max_cartesian_delta_m: Maximum target translation distance from the
            current end-effector pose. ``None`` disables this check.
        max_orientation_delta_rad: Maximum target orientation distance from the
            current end-effector orientation. ``None`` disables this check.
        max_joint_delta_deg: Maximum solved controlled-joint motion, measured
            in degrees from the current joint configuration. ``None`` disables
            this check.
        require_finite_action: Reject actions containing NaN or infinity.
    """

    max_cartesian_delta_m: float | None = 0.05
    max_orientation_delta_rad: float | None = 0.25
    max_joint_delta_deg: float | None = 10.0
    require_finite_action: bool = True

    def __post_init__(self) -> None:
        """Validate safety option values.

        Raises:
            ValueError: If any option value is invalid.
        """
        if self.max_cartesian_delta_m is not None and self.max_cartesian_delta_m <= 0:
            msg = "max_cartesian_delta_m must be positive or None."
            raise ValueError(msg)
        if self.max_orientation_delta_rad is not None and self.max_orientation_delta_rad <= 0:
            msg = "max_orientation_delta_rad must be positive or None."
            raise ValueError(msg)
        if self.max_joint_delta_deg is not None and self.max_joint_delta_deg <= 0:
            msg = "max_joint_delta_deg must be positive or None."
            raise ValueError(msg)


@dataclass
class KinematicEndEffectorObservation:
    """End-effector pose observation from a kinematics adapter.

    Attributes:
        joint_positions: Array of shape ``(7,)`` containing
            ``x, y, z, qx, qy, qz, qw``. Quaternion order is ``xyzw``.
        timestamp: Timestamp copied from the wrapped robot observation.
        sensor_data: Wrapped robot sensor data plus internal joint positions.
        images: Wrapped robot images, if any.
        internal_observation: Original observation returned by the wrapped robot.
    """

    joint_positions: np.ndarray
    timestamp: float
    sensor_data: dict[str, np.ndarray] | None = None
    images: dict[str, Frame] | None = None
    internal_observation: RobotObservation | None = None

    @property
    def state(self) -> np.ndarray:
        """State vector for inference: end-effector pose only."""
        return self.joint_positions


class KinematicEndEffectorRobot(Robot):
    """Expose a joint-space robot as a single end-effector pose robot.

    The public ``joint_positions``/action vector is ``x, y, z, qx, qy, qz, qw``.
    Internally, this adapter reads/writes the wrapped robot's joint-space API
    and uses a URDF model for forward/inverse kinematics.

    Args:
        internal_robot: Wrapped joint-space robot.
        urdf_path: URDF file used by the kinematics backend.
        end_effector_frame: URDF frame name for the end effector.
        controlled_joint_names: Joint names to solve/control. If omitted, all
            wrapped robot joints that also exist as 1-DoF URDF joints are used.
        joint_units: Units used by the wrapped robot for controlled joints.
            ``"degrees"`` also covers SO101 ``unit="normalized"`` for now.
        ik_options: Options for iterative inverse kinematics.
        safety_options: Guardrails applied before and after IK.

    Raises:
        ImportError: If Pinocchio is not installed.
        ValueError: If the frame or joint mapping is invalid.
    """

    def __init__(
        self,
        internal_robot: Robot,
        urdf_path: str | Path,
        end_effector_frame: str,
        controlled_joint_names: list[str] | None = None,
        *,
        joint_units: Literal["degrees", "radians"] = "degrees",
        ik_options: InverseKinematicsOptions | None = None,
        safety_options: KinematicSafetyOptions | None = None,
    ) -> None:
        """Initialize the kinematic end-effector adapter.

        Raises:
            ValueError: If adapter settings, frame, or joint mapping are invalid.
        """
        if joint_units not in {"degrees", "radians"}:
            msg = f"Invalid joint_units {joint_units!r}. Must be 'degrees' or 'radians'."
            raise ValueError(msg)

        self._pin = self._load_pinocchio()
        self._internal_robot = internal_robot
        self._urdf_path = Path(urdf_path)
        self._end_effector_frame = end_effector_frame
        self._joint_units = joint_units
        self._ik_options = ik_options or InverseKinematicsOptions()
        self._safety_options = safety_options or KinematicSafetyOptions()

        self._model = self._pin.buildModelFromUrdf(str(self._urdf_path))
        self._data = self._model.createData()
        self._frame_id = self._resolve_frame_id(end_effector_frame)
        self._controlled_joint_names = self._resolve_controlled_joint_names(controlled_joint_names)
        self._internal_indices = [self._internal_robot.joint_names.index(name) for name in self._controlled_joint_names]
        self._joint_q_indices = [self._joint_q_index(name) for name in self._controlled_joint_names]
        self._joint_v_indices = [self._joint_v_index(name) for name in self._controlled_joint_names]

    @property
    def internal_robot(self) -> Robot:
        """Wrapped joint-space robot."""
        return self._internal_robot

    @property
    def end_effector_frame(self) -> str:
        """URDF frame used as the end-effector pose."""
        return self._end_effector_frame

    @property
    def controlled_joint_names(self) -> list[str]:
        """Wrapped robot joints controlled by IK."""
        return list(self._controlled_joint_names)

    @property
    def joint_names(self) -> list[str]:
        """Pose component names matching the public state/action vector."""
        return list(POSE_JOINT_NAMES)

    @staticmethod
    def _load_pinocchio() -> ModuleType:
        try:
            return importlib.import_module("pinocchio")
        except ImportError as exc:
            msg = (
                "KinematicEndEffectorRobot requires Pinocchio. "
                "Install it with: pip install physicalai[kinematics]"
            )
            raise ImportError(msg) from exc

    def _resolve_frame_id(self, frame_name: str) -> int:
        frame_id = int(self._model.getFrameId(frame_name))
        if frame_id >= int(self._model.nframes):
            msg = f"End-effector frame {frame_name!r} was not found in URDF {self._urdf_path}."
            raise ValueError(msg)
        return frame_id

    def _resolve_controlled_joint_names(self, controlled_joint_names: list[str] | None) -> list[str]:
        internal_names = set(self._internal_robot.joint_names)
        model_names = set(self._model.names)
        if controlled_joint_names is None:
            names = [
                name for name in self._internal_robot.joint_names if name in model_names and self._joint_nq(name) == 1
            ]
        else:
            names = list(controlled_joint_names)

        if not names:
            msg = "No controlled joints resolved. Provide controlled_joint_names matching the URDF and internal robot."
            raise ValueError(msg)

        missing_internal = [name for name in names if name not in internal_names]
        if missing_internal:
            msg = f"Controlled joints missing from internal robot: {missing_internal}"
            raise ValueError(msg)

        missing_urdf = [name for name in names if name not in model_names]
        if missing_urdf:
            msg = f"Controlled joints missing from URDF: {missing_urdf}"
            raise ValueError(msg)

        unsupported = [name for name in names if self._joint_nq(name) != 1 or self._joint_nv(name) != 1]
        if unsupported:
            msg = f"Only 1-DoF controlled joints are supported, got: {unsupported}"
            raise ValueError(msg)

        return names

    def _joint_id(self, joint_name: str) -> int:
        return int(self._model.getJointId(joint_name))

    def _joint_nq(self, joint_name: str) -> int:
        return int(self._model.joints[self._joint_id(joint_name)].nq)

    def _joint_nv(self, joint_name: str) -> int:
        return int(self._model.joints[self._joint_id(joint_name)].nv)

    def _joint_q_index(self, joint_name: str) -> int:
        return int(self._model.joints[self._joint_id(joint_name)].idx_q)

    def _joint_v_index(self, joint_name: str) -> int:
        return int(self._model.joints[self._joint_id(joint_name)].idx_v)

    def connect(self) -> None:
        """Connect the wrapped robot."""
        self._internal_robot.connect()

    def disconnect(self) -> None:
        """Disconnect the wrapped robot."""
        self._internal_robot.disconnect()

    def is_connected(self) -> bool:
        """Return whether the wrapped robot is connected."""
        return self._internal_robot.is_connected()

    def get_observation(self) -> RobotObservation:
        """Return current end-effector pose from forward kinematics."""
        internal_obs = self._internal_robot.get_observation()
        q = self._configuration_from_internal_positions(internal_obs.joint_positions)
        pose = self._forward_pose(q)

        sensor_data = dict(internal_obs.sensor_data) if internal_obs.sensor_data is not None else {}
        sensor_data["internal_joint_positions"] = internal_obs.joint_positions.copy()

        return KinematicEndEffectorObservation(
            joint_positions=pose.astype(np.float32),
            timestamp=internal_obs.timestamp,
            sensor_data=sensor_data,
            images=internal_obs.images,
            internal_observation=internal_obs,
        )

    def send_action(self, action: np.ndarray, *, goal_time: float = 0.1) -> None:
        """Solve IK for an end-effector pose action and command the wrapped robot."""
        pose = self._validate_pose_action(action)
        target_translation = pose[:3]
        target_quaternion = self._normalize_quaternion_xyzw(pose[3:])
        target = self._pin.SE3(self._quaternion_xyzw_to_rotation(target_quaternion), target_translation)

        internal_obs = self._internal_robot.get_observation()
        seed = self._configuration_from_internal_positions(internal_obs.joint_positions)
        current_pose = self._forward_pose(seed)
        self._validate_target_delta(current_pose, pose)

        solution = self._inverse_kinematics(target, seed)
        self._validate_solution_delta(seed, solution)
        internal_action = internal_obs.joint_positions.copy()
        solved_positions = self._internal_positions_from_configuration(solution)
        for index, position in zip(self._internal_indices, solved_positions, strict=True):
            internal_action[index] = position

        self._internal_robot.send_action(internal_action, goal_time=goal_time)

    def _validate_pose_action(self, action: np.ndarray) -> np.ndarray:
        action = np.asarray(action)
        if action.ndim != 1:
            msg = f"Expected 1D action array, got shape {action.shape}."
            raise ValueError(msg)
        if action.shape[0] < len(POSE_JOINT_NAMES):
            msg = f"Expected at least {len(POSE_JOINT_NAMES)} action dims, got {action.shape}."
            raise ValueError(msg)

        pose = np.asarray(action[: len(POSE_JOINT_NAMES)], dtype=np.float64)
        if self._safety_options.require_finite_action and not np.all(np.isfinite(pose)):
            msg = "Action pose must contain only finite values."
            raise ValueError(msg)
        return pose

    def _validate_target_delta(self, current_pose: np.ndarray, target_pose: np.ndarray) -> None:
        max_cartesian_delta = self._safety_options.max_cartesian_delta_m
        if max_cartesian_delta is not None:
            delta = float(np.linalg.norm(target_pose[:3] - current_pose[:3]))
            if delta > max_cartesian_delta:
                msg = f"Cartesian target delta {delta:.6f} m exceeds limit {max_cartesian_delta:.6f} m."
                raise ValueError(msg)

        max_orientation_delta = self._safety_options.max_orientation_delta_rad
        if max_orientation_delta is not None:
            delta = self._orientation_delta_rad(current_pose[3:], target_pose[3:])
            if delta > max_orientation_delta:
                msg = f"Orientation target delta {delta:.6f} rad exceeds limit {max_orientation_delta:.6f} rad."
                raise ValueError(msg)

    def _validate_solution_delta(self, seed: np.ndarray, solution: np.ndarray) -> None:
        max_joint_delta_deg = self._safety_options.max_joint_delta_deg
        if max_joint_delta_deg is None:
            return

        deltas_rad = np.abs(solution[self._joint_q_indices] - seed[self._joint_q_indices])
        deltas_deg = np.rad2deg(deltas_rad)
        max_delta_deg = float(np.max(deltas_deg)) if deltas_deg.size else 0.0
        if max_delta_deg > max_joint_delta_deg:
            msg = f"IK joint delta {max_delta_deg:.6f} deg exceeds limit {max_joint_delta_deg:.6f} deg."
            raise ValueError(msg)

    def _configuration_from_internal_positions(self, positions: np.ndarray) -> np.ndarray:
        if positions.shape[0] < len(self._internal_robot.joint_names):
            msg = f"Internal observation has fewer positions than joint_names: {positions.shape}"
            raise ValueError(msg)

        q = self._pin.neutral(self._model)
        values = np.asarray(positions, dtype=np.float64)
        for internal_index, q_index in zip(self._internal_indices, self._joint_q_indices, strict=True):
            q[q_index] = self._to_kinematics_units(values[internal_index])
        return q

    def _internal_positions_from_configuration(self, q: np.ndarray) -> np.ndarray:
        values = np.empty(len(self._joint_q_indices), dtype=np.float64)
        for i, q_index in enumerate(self._joint_q_indices):
            values[i] = self._from_kinematics_units(float(q[q_index]))
        return values

    def _to_kinematics_units(self, value: float) -> float:
        if self._joint_units == "degrees":
            return float(np.deg2rad(value))
        return float(value)

    def _from_kinematics_units(self, value: float) -> float:
        if self._joint_units == "degrees":
            return float(np.rad2deg(value))
        return float(value)

    def _forward_pose(self, q: np.ndarray) -> np.ndarray:
        self._pin.forwardKinematics(self._model, self._data, q)
        self._pin.updateFramePlacements(self._model, self._data)
        placement = self._data.oMf[self._frame_id]
        quaternion = np.asarray(self._pin.Quaternion(placement.rotation).coeffs(), dtype=np.float64)
        quaternion = self._normalize_quaternion_xyzw(quaternion)
        return np.concatenate([np.asarray(placement.translation, dtype=np.float64), quaternion])

    def _inverse_kinematics(self, target: object, seed: np.ndarray) -> np.ndarray:
        q = seed.copy()
        reference_frame = self._pin.ReferenceFrame.LOCAL
        damping_matrix = self._ik_options.damping * np.eye(6)
        final_error_norm = float("inf")

        for _ in range(self._ik_options.max_iterations):
            self._pin.forwardKinematics(self._model, self._data, q)
            self._pin.updateFramePlacements(self._model, self._data)

            current = self._data.oMf[self._frame_id]
            error = np.asarray(self._pin.log(current.actInv(target)).vector, dtype=np.float64)
            final_error_norm = float(np.linalg.norm(error))
            if final_error_norm < self._ik_options.tolerance:
                return q

            full_jacobian = np.asarray(
                self._pin.computeFrameJacobian(self._model, self._data, q, self._frame_id, reference_frame),
                dtype=np.float64,
            )
            jacobian = full_jacobian[:, self._joint_v_indices]
            delta = jacobian.T @ np.linalg.solve(jacobian @ jacobian.T + damping_matrix, error)
            full_delta = np.zeros(int(self._model.nv), dtype=np.float64)
            for v_index, value in zip(self._joint_v_indices, delta, strict=True):
                full_delta[v_index] = value
            q = self._pin.integrate(self._model, q, full_delta)
            q = self._clamp_to_position_limits(q)

        msg = (
            f"IK failed to converge for frame {self._end_effector_frame!r}; "
            f"final_error_norm={final_error_norm:.6g}, max_iterations={self._ik_options.max_iterations}."
        )
        raise ValueError(msg)

    def _clamp_to_position_limits(self, q: np.ndarray) -> np.ndarray:
        lower = np.asarray(self._model.lowerPositionLimit, dtype=np.float64)
        upper = np.asarray(self._model.upperPositionLimit, dtype=np.float64)
        if lower.shape == q.shape and upper.shape == q.shape:
            return np.minimum(np.maximum(q, lower), upper)
        return q

    @staticmethod
    def _normalize_quaternion_xyzw(quaternion: np.ndarray) -> np.ndarray:
        norm = float(np.linalg.norm(quaternion))
        if norm == 0 or not np.isfinite(norm):
            msg = "Quaternion must have non-zero finite norm."
            raise ValueError(msg)
        return np.asarray(quaternion, dtype=np.float64) / norm

    @classmethod
    def _orientation_delta_rad(cls, first: np.ndarray, second: np.ndarray) -> float:
        first = cls._normalize_quaternion_xyzw(first)
        second = cls._normalize_quaternion_xyzw(second)
        dot = abs(float(np.dot(first, second)))
        dot = float(np.clip(dot, -1.0, 1.0))
        return 2.0 * float(np.arccos(dot))

    @staticmethod
    def _quaternion_xyzw_to_rotation(quaternion: np.ndarray) -> np.ndarray:
        x, y, z, w = quaternion
        xx = x * x
        yy = y * y
        zz = z * z
        xy = x * y
        xz = x * z
        yz = y * z
        wx = w * x
        wy = w * y
        wz = w * z
        return np.array(
            [
                [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
                [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
                [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
            ],
            dtype=np.float64,
        )
