# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
# ruff: noqa: INP001, T201
# This standalone benchmark is a CLI, not an importable library package.

"""Measure adapter CPU overhead with in-memory devices, never hardware.

An optional baseline is an operator-selected, trusted Python source snapshot.
Run from the repository root with this package's src and runtime src on PYTHONPATH.
"""

from __future__ import annotations

import argparse
import gc
import importlib.util
import platform
import statistics
import sys
import time
from pathlib import Path
from typing import TYPE_CHECKING, Any

import numpy as np
from physicalai_lerobot_plugin import lerobot_adapter

if TYPE_CHECKING:
    from collections.abc import Callable
    from types import ModuleType


class FakeDevice:
    """Return prebuilt values without SDK imports, bus traffic, or mock overhead."""

    is_connected = True

    def __init__(self, joints: int, *, auxiliary: bool = False) -> None:
        """Prepare immutable input values outside the timed region."""
        self.observation_features = {f"joint_{i:02}.pos": float(i) for i in range(joints)}
        self.action_features = self.observation_features
        self.values = dict(self.observation_features)
        if auxiliary:
            self.values.update(
                temperature=25.0,
                velocity=np.zeros(joints, dtype=np.float32),
                camera=np.zeros((8, 8, 3), dtype=np.uint8),
            )

    def get_observation(self) -> dict[str, Any]:
        """Return the prebuilt observation.

        Returns:
            The fake device values.
        """
        return self.values

    def get_action(self) -> dict[str, Any]:
        """Return the prebuilt teleoperator action.

        Returns:
            The fake device values.
        """
        return self.values

    def send_action(self, action: dict[str, Any]) -> None:
        """Discard the outgoing command without I/O."""

    def send_feedback(self, action: dict[str, Any]) -> None:
        """Discard the outgoing feedback without I/O."""


def _load_baseline(path: Path) -> ModuleType:
    spec = importlib.util.spec_from_file_location("_lerobot_benchmark_baseline", path)
    if spec is None or spec.loader is None:
        msg = f"Cannot load baseline {path}"
        raise ValueError(msg)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _make_operation(module: ModuleType, scenario: str, joints: int) -> Callable[[], object]:
    device = FakeDevice(joints, auxiliary=scenario.endswith("aux"))
    if scenario.startswith("robot"):
        adapter = module.LeRobotAdapter("fake", {}, _robot=device)
    else:
        adapter = module.LeRobotTeleoperatorAdapter("fake", {}, role="follower", _teleoperator=device)
    if "write" in scenario:
        action = np.arange(joints, dtype=np.float32)
        return lambda: adapter.send_action(action)
    return adapter.get_observation


def _measure(operation: Callable[[], object], iterations: int) -> float:
    start = time.perf_counter_ns()
    for _ in range(iterations):
        operation()
    return (time.perf_counter_ns() - start) / iterations / 1000


def main() -> None:
    """Print repeat-level CPU timings in microseconds per public API call."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", type=Path)
    parser.add_argument("--iterations", type=int, default=50_000)
    parser.add_argument("--repeats", type=int, default=9)
    args = parser.parse_args()
    minimum_repeats = 2
    if args.iterations < 1 or args.repeats < minimum_repeats:
        parser.error("iterations must be positive and repeats must be at least two")
    modules = {"current": lerobot_adapter}
    if args.baseline:
        modules = {"baseline": _load_baseline(args.baseline), **modules}
    print(f"Python {platform.python_version()}, NumPy {np.__version__}, {platform.platform()}")
    print(f"iterations={args.iterations}, repeats={args.repeats}, warmup=2000, GC disabled while timing")
    print("scenario,joints,version,mean_us,stdev_us,min_us,max_us")
    for joints in (6, 14):
        for scenario in (
            "robot_read",
            "teleop_read",
            "robot_read_aux",
            "teleop_read_aux",
            "robot_write",
            "teleop_write",
        ):
            operations = {label: _make_operation(module, scenario, joints) for label, module in modules.items()}
            for operation in operations.values():
                for _ in range(2000):
                    operation()
            samples: dict[str, list[float]] = {label: [] for label in modules}
            gc.disable()
            try:
                for repeat in range(args.repeats):
                    labels = list(modules)
                    if repeat % 2:
                        labels.reverse()
                    for label in labels:
                        samples[label].append(_measure(operations[label], args.iterations))
            finally:
                gc.enable()
            for label, timings in samples.items():
                print(
                    f"{scenario},{joints},{label},{statistics.mean(timings):.3f},"
                    f"{statistics.stdev(timings):.3f},{min(timings):.3f},{max(timings):.3f}"
                )


if __name__ == "__main__":
    main()
