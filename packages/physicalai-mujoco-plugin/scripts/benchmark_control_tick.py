# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

"""Measure asset-free control ticks; no rendering, downloads, network or hardware."""

# ruff: noqa: INP001, SLF001
# Standalone benchmark CLI: deliberately assemble and exercise the driver's private tick.

from __future__ import annotations

import argparse
import json
import platform
import sys
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
    robot._sim = SimpleNamespace(model=model, data=data, channels=[channels], bases=())
    robot._substeps = substeps
    robot._ease = robot._watcher = robot._replay = robot._viewer = robot._episode_auto_reset = None
    robot._automation = SimpleNamespace(tick=_noop, take_reset_request=lambda: False)
    robot._objects = SimpleNamespace(apply_held=_noop, publish=_noop)
    robot._cameras = SimpleNamespace(publish=_noop)
    robot._drain_commands = robot._record_tick = _noop
    return robot


def main() -> None:
    """Report repeated wall/CPU tick latency and per-call wall percentiles in microseconds.

    Raises:
        RuntimeError: If physics ends with non-finite joint positions.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--calls", type=int, default=10000)
    parser.add_argument("--repeats", type=int, default=5)
    parser.add_argument(
        "--paired", action="store_true", help="Alternate current dispatch with the original Python loop"
    )
    args = parser.parse_args()
    sys.stdout.write(
        json.dumps({"python": platform.python_version(), "mujoco": mujoco.__version__, "numpy": np.__version__}) + "\n"
    )
    for kind in ("position", "motor", "velocity"):
        for substeps in (1, 10, 40):
            variants = {"current": make_robot(kind, substeps)}
            if args.paired:
                reference = make_robot(kind, substeps)
                # Force the original Python loop without duplicating production integration code.
                # The reference also pays the new dispatch check: this is not the pre-change binary.
                reference._sim.channels = [
                    SimpleNamespace(
                        requires_step_control=True, apply_pd=channel.apply_pd, model_targets=channel.model_targets
                    )
                    for channel in reference._sim.channels
                ]
                variants["python_loop"] = reference
            results = {name: {"wall_us": [], "cpu_us": [], "samples": []} for name in variants}
            for robot in variants.values():
                for _ in range(1000):
                    robot._step_and_sync()
            for repeat in range(args.repeats):
                order = list(variants) if repeat % 2 == 0 else list(reversed(variants))
                for name in order:
                    robot, result = variants[name], results[name]
                    start_cpu = time.process_time_ns()
                    start_wall = time.perf_counter_ns()
                    for _ in range(args.calls):
                        start = time.perf_counter_ns()
                        robot._step_and_sync()
                        result["samples"].append((time.perf_counter_ns() - start) / 1000)
                    result["wall_us"].append((time.perf_counter_ns() - start_wall) / args.calls / 1000)
                    result["cpu_us"].append((time.process_time_ns() - start_cpu) / args.calls / 1000)
            for robot in variants.values():
                if not np.isfinite(robot._sim.data.qpos).all():
                    msg = "Benchmark ended with non-finite joint positions"
                    raise RuntimeError(msg)
            if args.paired:
                np.testing.assert_array_equal(
                    variants["current"]._sim.data.qpos, variants["python_loop"]._sim.data.qpos
                )
            for name, result in results.items():
                samples = result.pop("samples")
                sys.stdout.write(
                    json.dumps({
                        "variant": name,
                        "kind": kind,
                        "substeps": substeps,
                        "calls": len(samples),
                        **result,
                        "p50_p95_p99_max_us": np.percentile(samples, [50, 95, 99, 100]).tolist(),
                    })
                    + "\n"
                )
                sys.stdout.flush()


if __name__ == "__main__":
    main()
