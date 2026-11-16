# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

"""Hardware-free gravity and public API benchmark; never constructs a real controller."""

from __future__ import annotations

import argparse
import json
import sys
import time
from collections import Counter
from collections.abc import Callable
from functools import partial
from types import ModuleType, SimpleNamespace
from typing import TYPE_CHECKING, Any

import numpy as np

# Install a deliberately inert SDK before importing the drivers. There is no hardware path.
sdk = ModuleType("motorbridge")
sdk.Controller = None
sdk.Mode = None
sys.modules["motorbridge"] = sdk

from physicalai_rebot_b601_plugin import get_urdf_path  # noqa: E402
from physicalai_rebot_b601_plugin.constants import (  # noqa: E402
    REBOT_B601_DM_GRAVITY_URDF,
    REBOT_B601_RS_GRAVITY_URDF,
    REBOT_B601_URDF_ARM_JOINTS,
)
from physicalai_rebot_b601_plugin.dm import ReBotB601DM  # noqa: E402
from physicalai_rebot_b601_plugin.gravity import GravityModel  # noqa: E402
from physicalai_rebot_b601_plugin.rs import ReBotB601RS  # noqa: E402

if TYPE_CHECKING:
    from physicalai_rebot_b601_plugin.gravity import _ActiveJoint


class ReferenceGravityModel(GravityModel):
    """Original NumPy kernel for paired comparisons in the same process."""

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


class FakeMotor:
    """Record SDK calls without a device or SDK dependency."""

    def __init__(self, counts: Counter[str]) -> None:
        self.counts = counts
        self.state = SimpleNamespace(pos=0.0, vel=0.0, torq=1.0, t_mos=25.0, t_rotor=26.0, status_code=0)

    def get_state(self) -> SimpleNamespace:
        self.counts["get_state"] += 1
        return self.state

    def request_feedback(self) -> None:
        self.counts["request_feedback"] += 1

    def send_mit(self, *args: float) -> None:
        self.counts["send_mit"] += 1

    def send_pos_vel(self, *args: float) -> None:
        self.counts["send_pos_vel"] += 1

    def send_force_pos(self, *args: float) -> None:
        self.counts["send_force_pos"] += 1

    def robstride_write_param_f32(self, *args: float) -> None:
        self.counts["write_torque_limit"] += 1


class FakeController:
    def __init__(self, counts: Counter[str]) -> None:
        self.counts = counts

    def poll_feedback_once(self) -> None:
        self.counts["poll_feedback_once"] += 1


def make_robot(variant: str, *, reference: bool = False) -> tuple[Any, Counter[str]]:
    """Attach only inert fakes; skip connect, enable, park, and disconnect entirely."""
    counts: Counter[str] = Counter()
    cls = ReBotB601RS if variant == "rs" else ReBotB601DM
    robot = cls(approach_speed=None, park_on_disconnect=False)
    if variant == "dm-mit":
        robot = ReBotB601DM(control_mode="mit", approach_speed=None, park_on_disconnect=False)
    robot._controller = FakeController(counts)
    robot._motors = {name: FakeMotor(counts) for name in robot.JOINT_ORDER}
    robot._gravity()  # Parse outside timing, as connect does for MIT mode.
    if reference:
        robot._gravity_model = ReferenceGravityModel(get_urdf_path() / robot.GRAVITY_URDF, REBOT_B601_URDF_ARM_JOINTS)
    return robot, counts


def measure(operations: dict[str, Callable[[], Any]], iterations: int, repeats: int) -> dict[str, Any]:
    for operation in operations.values():
        for _ in range(100):
            operation()
    samples: dict[str, list[float]] = {name: [] for name in operations}
    means: dict[str, list[float]] = {name: [] for name in operations}
    for repeat in range(repeats):
        order = list(operations) if repeat % 2 == 0 else list(reversed(operations))
        for name in order:
            operation = operations[name]
            start = time.perf_counter_ns()
            for _ in range(iterations):
                before = time.perf_counter_ns()
                operation()
                samples[name].append((time.perf_counter_ns() - before) / 1000.0)
            means[name].append((time.perf_counter_ns() - start) / iterations / 1000.0)
    return {name: summarize(means[name], samples[name]) for name in operations}


def summarize(means: list[float], samples: list[float]) -> dict[str, float]:
    return {
        "mean_us": float(np.mean(means)),
        "repeat_std_us": float(np.std(means, ddof=1)),
        **{f"p{p}_us": float(np.percentile(samples, p)) for p in (50, 95, 99)},
        "max_us": max(samples),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--iterations", type=int, default=1000)
    parser.add_argument("--repeats", type=int, default=7)
    args = parser.parse_args()
    if args.iterations < 1 or args.repeats < 2:
        parser.error("--iterations must be positive and --repeats must be at least 2")
    results = {}
    q = np.array([0.3, -1.5, -0.5, 0.4, 0.2, 0.1])
    for variant, urdf in (("dm", REBOT_B601_DM_GRAVITY_URDF), ("rs", REBOT_B601_RS_GRAVITY_URDF)):
        model = GravityModel(get_urdf_path() / urdf, REBOT_B601_URDF_ARM_JOINTS)
        reference = ReferenceGravityModel(get_urdf_path() / urdf, REBOT_B601_URDF_ARM_JOINTS)
        results[f"gravity-{variant}"] = measure(
            {"original": partial(reference.torques, q), "optimized": partial(model.torques, q)},
            args.iterations,
            args.repeats,
        )
    for variant in ("dm", "dm-mit", "rs"):
        original, original_counts = make_robot(variant, reference=True)
        optimized, optimized_counts = make_robot(variant)
        action = np.zeros(7)
        for scenario in ("read", "write"):
            operations = (
                {"original": original.get_observation, "optimized": optimized.get_observation}
                if scenario == "read"
                else {
                    "original": partial(original.send_action, action),
                    "optimized": partial(optimized.send_action, action),
                }
            )
            result = measure(operations, args.iterations, args.repeats)
            for name, robot, counts in (
                ("original", original, original_counts),
                ("optimized", optimized, optimized_counts),
            ):
                # Exclude setup/refresh counts from this steady-state single-call snapshot.
                if variant == "rs" and scenario == "write":
                    robot._gripper_limit_written_at = time.monotonic()
                counts.clear()
                operations[name]()
                result[name]["fake_calls"] = dict(counts)
            if result["original"]["fake_calls"] != result["optimized"]["fake_calls"]:
                msg = f"Changed SDK call budget for {variant}-{scenario}"
                raise RuntimeError(msg)
            results[f"{variant}-{scenario}"] = result
    print(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()
