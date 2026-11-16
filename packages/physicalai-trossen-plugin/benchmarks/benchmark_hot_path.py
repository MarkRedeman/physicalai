# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

"""Measure host CPU work with a fake SDK; never construct a hardware driver."""

from __future__ import annotations

import platform
import statistics
import sys
import timeit
from collections import Counter
from types import ModuleType

import numpy as np

# Replace even an installed SDK before importing the driver. This script cannot
# configure a network connection or instantiate a real vendor driver.
sdk = ModuleType("trossen_arm")
sdk.VectorDouble = list
sys.modules["trossen_arm"] = sdk

from physicalai_trossen_plugin import WidowXAI  # noqa: E402


class FakeDriver:
    """Return deterministic state and count SDK calls outside timed samples."""

    def __init__(self) -> None:
        self.calls: Counter[str] = Counter()
        self.count = False
        self.positions = [0.01, -0.02, 0.03, -0.04, 0.05, -0.06, 0.02]

    def _record(self, name: str) -> None:
        if self.count:
            self.calls[name] += 1

    def get_all_positions(self) -> list[float]:
        self._record("positions")
        return self.positions

    def get_all_velocities(self) -> list[float]:
        self._record("velocities")
        return self.positions

    def get_all_external_efforts(self) -> list[float]:
        self._record("efforts")
        return self.positions

    def set_all_positions(self, values: list[float], goal_time: float, blocking: bool) -> None:
        self._record("set_positions")


def main() -> None:
    """Print repeat statistics in microseconds per public API call."""
    print(f"Python {platform.python_version()}, NumPy {np.__version__}, {platform.platform()}")
    for role in ("follower", "leader"):
        robot = WidowXAI("benchmark.invalid", role=role)
        driver = FakeDriver()
        robot._driver = driver
        action = np.array([1.0, -2.0, 3.0, -4.0, 5.0, -6.0, 0.03], dtype=np.float32)
        operations = {"observation": robot.get_observation}
        if role == "follower":
            operations["action"] = lambda: robot.send_action(action)
        for name, operation in operations.items():
            driver.calls.clear()
            driver.count = True
            operation()
            driver.count = False
            calls = dict(driver.calls)
            for _ in range(1000):
                operation()
            samples = [seconds * 1e6 / 20000 for seconds in timeit.repeat(operation, number=20000, repeat=7)]
            print(
                f"{role} {name}: median={statistics.median(samples):.3f} us, "
                f"min={min(samples):.3f}, max={max(samples):.3f}; calls={calls}"
            )


if __name__ == "__main__":
    main()
