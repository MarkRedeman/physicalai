# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

"""Measure asset-free control ticks; no rendering, downloads, network or hardware."""

from __future__ import annotations

import argparse
import json
import platform
import time
from types import SimpleNamespace

import mujoco
import numpy as np
from physicalai_mujoco_plugin.channels import ArmChannels
from physicalai_mujoco_plugin.profiles import RobotProfile
from physicalai_mujoco_plugin.profiles.derive import derive_profile
from physicalai_mujoco_plugin.robot import MuJoCoRobot


def make_model(kind: str, joints: int = 6) -> mujoco.MjModel:
    """Build independent pendulums with the requested actuator kind.

    Returns:
        An asset-free compiled model.
    """
    bodies = "".join(
        f'<body pos="{index} 0 1"><joint name="j{index}" axis="0 1 0" range="-2 2"/>'
        '<geom type="capsule" fromto="0 0 0 0 0 0.2" size="0.02" mass="1"/></body>'
        for index in range(joints)
    )
    actuators = "".join(f'<{kind} joint="j{index}"/>' for index in range(joints))
    return mujoco.MjModel.from_xml_string(
        '<mujoco><compiler angle="radian"/><option timestep="0.002"/>'
        f"<worldbody>{bodies}</worldbody><actuator>{actuators}</actuator></mujoco>"
    )


def _noop(*_args: object) -> None:
    pass


def make_robot(kind: str, substeps: int, joints: int = 6) -> MuJoCoRobot:
    """Use the real tick and channels, replacing only unrelated scene services.

    Returns:
        A benchmark-only driver with real physics and channels, but no scene services.
    """
    model = make_model(kind, joints)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    channels = ArmChannels(model, data, derive_profile(model), RobotProfile("bench", "Bench", "unused"))
    channels.write(np.full(joints, 10.0))
    robot = MuJoCoRobot.__new__(MuJoCoRobot)
    robot._sim = SimpleNamespace(model=model, data=data, channels=[channels], bases=())  # ruff: ignore[private-member-access]
    robot._substeps = substeps  # ruff: ignore[private-member-access]
    robot._ease = robot._watcher = robot._replay = robot._viewer = robot._episode_auto_reset = None  # ruff: ignore[private-member-access]
    robot._automation = SimpleNamespace(tick=_noop, take_reset_request=lambda: False)  # ruff: ignore[private-member-access]
    robot._objects = SimpleNamespace(apply_held=_noop, publish=_noop)  # ruff: ignore[private-member-access]
    robot._cameras = SimpleNamespace(publish=_noop)  # ruff: ignore[private-member-access]
    robot._drain_commands = robot._record_tick = _noop  # ruff: ignore[private-member-access]
    return robot


def main() -> None:
    """Report repeated wall/CPU tick latency and per-call wall percentiles in microseconds."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--calls", type=int, default=10000)
    parser.add_argument("--repeats", type=int, default=5)
    args = parser.parse_args()
    print(json.dumps({"python": platform.python_version(), "mujoco": mujoco.__version__, "numpy": np.__version__}))
    for kind in ("position", "motor", "velocity"):
        for substeps in (1, 10, 40):
            robot = make_robot(kind, substeps)
            for _ in range(1000):
                robot._step_and_sync()  # ruff: ignore[private-member-access]
            wall_means, cpu_means, samples = [], [], []
            for _ in range(args.repeats):
                start_cpu = time.process_time_ns()
                start_wall = time.perf_counter_ns()
                for _ in range(args.calls):
                    start = time.perf_counter_ns()
                    robot._step_and_sync()  # ruff: ignore[private-member-access]
                    samples.append((time.perf_counter_ns() - start) / 1000)
                wall_means.append((time.perf_counter_ns() - start_wall) / args.calls / 1000)
                cpu_means.append((time.process_time_ns() - start_cpu) / args.calls / 1000)
            print(
                json.dumps({
                    "kind": kind,
                    "substeps": substeps,
                    "calls": len(samples),
                    "wall_us": wall_means,
                    "cpu_us": cpu_means,
                    "p50_p95_p99_max_us": np.percentile(samples, [50, 95, 99, 100]).tolist(),
                }),
                flush=True,
            )


if __name__ == "__main__":
    main()
