# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

"""Read and optionally jog an end-effector pose through URDF kinematics.

Examples:
    python examples/kinematics/end_effector_pose.py \
      --robot so101 --port /dev/ttyUSB0 --calibration ./calibration.json \
      --urdf ./so101.urdf --end-effector-frame gripper_link \
      --controlled-joint shoulder_pan --controlled-joint shoulder_lift \
      --controlled-joint elbow_flex --controlled-joint wrist_flex \
      --controlled-joint wrist_roll

    python examples/kinematics/end_effector_pose.py \
      --robot widowxai --ip 192.168.1.2 \
      --urdf ./widowxai.urdf --end-effector-frame ee_link \
      --controlled-joint shoulder_pan --controlled-joint shoulder_lift \
      --controlled-joint elbow_flex --controlled-joint wrist_flex \
      --controlled-joint wrist_yaw --controlled-joint wrist_roll \
      --jog-x 0.01 --execute
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np

from physicalai.robot import KinematicEndEffectorRobot
from physicalai.robot.kinematics import InverseKinematicsOptions

if TYPE_CHECKING:
    from physicalai.robot.interface import Robot
    from physicalai.robot.kinematics import KinematicEndEffectorObservation


def _build_internal_robot(args: argparse.Namespace) -> Robot:
    """Build the wrapped joint-space robot from CLI arguments.

    Returns:
        Joint-space robot to wrap with kinematics.
    """
    if args.robot == "so101":
        from physicalai.robot import SO101  # noqa: PLC0415

        if not args.port:
            sys.exit("error: --port is required for so101")
        if not args.calibration:
            sys.exit("error: --calibration is required for so101")
        return SO101(port=args.port, calibration=args.calibration, baudrate=args.baudrate, role="follower")

    if args.robot == "widowxai":
        from physicalai.robot import WidowXAI  # noqa: PLC0415

        if not args.ip:
            sys.exit("error: --ip is required for widowxai")
        return WidowXAI(ip=args.ip, role="follower")

    sys.exit(f"error: unknown robot type: {args.robot}")


def _format_vector(values: np.ndarray) -> str:
    """Format a vector for compact terminal output.

    Returns:
        Comma-separated vector values.
    """
    return ", ".join(f"{value:.6f}" for value in values)


def _print_observation(obs: KinematicEndEffectorObservation) -> None:
    """Print pose-space and internal robot-space observations."""
    pose = obs.joint_positions
    print("End-effector pose:")  # noqa: T201
    print(f"  xyz:        {_format_vector(pose[:3])}")  # noqa: T201
    print(f"  quat xyzw:  {_format_vector(pose[3:])}")  # noqa: T201

    if obs.internal_observation is not None:
        internal = obs.internal_observation
        print("Internal joint positions:")  # noqa: T201
        print(f"  {_format_vector(internal.joint_positions)}")  # noqa: T201


def _build_parser() -> argparse.ArgumentParser:
    """Build the command-line parser.

    Returns:
        Configured argument parser.
    """
    parser = argparse.ArgumentParser(
        description="Read and optionally jog an end-effector pose through URDF kinematics.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    robot_group = parser.add_argument_group("robot")
    robot_group.add_argument("--robot", required=True, choices=("so101", "widowxai"))
    robot_group.add_argument("--port", help="Serial port for SO101, e.g. /dev/ttyUSB0")
    robot_group.add_argument("--baudrate", type=int, default=1_000_000, help="SO101 baudrate (default: 1000000)")
    robot_group.add_argument("--calibration", help="SO101 calibration JSON path")
    robot_group.add_argument("--ip", help="WidowXAI robot IP address")

    kinematics_group = parser.add_argument_group("kinematics")
    kinematics_group.add_argument("--urdf", required=True, type=Path, help="URDF file path")
    kinematics_group.add_argument("--end-effector-frame", required=True, help="URDF end-effector frame name")
    kinematics_group.add_argument(
        "--controlled-joint",
        action="append",
        dest="controlled_joints",
        required=True,
        help="Controlled URDF/internal robot joint name. Repeat for each arm joint.",
    )
    kinematics_group.add_argument(
        "--joint-units",
        choices=("degrees", "radians"),
        default="degrees",
        help="Internal robot units for controlled joints (default: degrees). SO101 normalized is treated as degrees.",
    )
    kinematics_group.add_argument("--ik-max-iterations", type=int, default=100, help="IK max iterations")
    kinematics_group.add_argument("--ik-tolerance", type=float, default=1e-4, help="IK convergence tolerance")
    kinematics_group.add_argument("--ik-damping", type=float, default=1e-6, help="IK damping coefficient")

    jog_group = parser.add_argument_group("optional Cartesian jog")
    jog_group.add_argument("--jog-x", type=float, default=0.0, help="Relative x movement in meters")
    jog_group.add_argument("--jog-y", type=float, default=0.0, help="Relative y movement in meters")
    jog_group.add_argument("--jog-z", type=float, default=0.0, help="Relative z movement in meters")
    jog_group.add_argument("--execute", action="store_true", help="Actually send the jog action to the robot")
    jog_group.add_argument("--goal-time", type=float, default=0.5, help="Goal time for jog action in seconds")
    jog_group.add_argument(
        "--settle-time",
        type=float,
        default=0.5,
        help="Seconds to wait after jog before reading again",
    )

    return parser


def _read_and_optionally_jog(robot: KinematicEndEffectorRobot, args: argparse.Namespace) -> None:
    """Read the current pose and optionally execute one Cartesian jog."""
    obs = robot.get_observation()
    _print_observation(obs)

    jog = np.array([args.jog_x, args.jog_y, args.jog_z], dtype=np.float32)
    if not np.any(jog):
        print("No jog requested. Exiting without sending an action.")  # noqa: T201
        return

    target = obs.joint_positions.copy()
    target[:3] += jog
    print(f"Requested jog xyz: {_format_vector(jog)}")  # noqa: T201
    print(f"Target pose:       {_format_vector(target)}")  # noqa: T201

    if not args.execute:
        print("Dry run only. Pass --execute to solve IK and send the target pose.")  # noqa: T201
        return

    print("Sending Cartesian jog through IK...")  # noqa: T201
    robot.send_action(target, goal_time=args.goal_time)
    time.sleep(args.settle_time)
    print("Observation after jog:")  # noqa: T201
    _print_observation(robot.get_observation())


def main(argv: list[str] | None = None) -> None:
    """Run the end-effector pose example."""
    parser = _build_parser()
    args = parser.parse_args(argv)

    internal_robot = _build_internal_robot(args)
    robot = KinematicEndEffectorRobot(
        internal_robot=internal_robot,
        urdf_path=args.urdf,
        end_effector_frame=args.end_effector_frame,
        controlled_joint_names=args.controlled_joints,
        joint_units=args.joint_units,
        ik_options=InverseKinematicsOptions(
            max_iterations=args.ik_max_iterations,
            tolerance=args.ik_tolerance,
            damping=args.ik_damping,
        ),
    )

    print(f"Connecting {args.robot}...")  # noqa: T201
    robot.connect()
    try:
        _read_and_optionally_jog(robot, args)
    finally:
        robot.disconnect()
        print("Disconnected.")  # noqa: T201


if __name__ == "__main__":
    main()
