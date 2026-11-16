# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

"""Hardware-free CPU benchmark; never opens a serial port.

Run with the package src and Runtime src on PYTHONPATH. Transport timings are
intentionally excluded; call counts describe driver-to-SDK calls, not packets
observed on a real bus.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import platform
import statistics
import sys
import timeit
from collections import Counter
from collections.abc import Callable
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from physicalai_so101_plugin.calibration import SO101Calibration
from physicalai_so101_plugin.constants import SO101_JOINT_ORDER
from physicalai_so101_plugin.so101 import SO101


class FakeGroup:
    """Count SDK entry points without serial communication or mock overhead."""

    def __init__(self) -> None:
        self.calls: Counter[str] = Counter()

    def clearParam(self) -> None:
        self.calls["clearParam"] += 1

    def addParam(self, servo_id: int, data: list[int]) -> bool:
        self.calls["addParam"] += 1
        return True

    def txPacket(self) -> int:
        self.calls["txPacket"] += 1
        return 0

    def txRxPacket(self) -> int:
        self.calls["txRxPacket"] += 1
        return 0

    def isAvailable(self, servo_id: int, address: int, length: int) -> bool:
        self.calls["isAvailable"] += 1
        return True

    def getData(self, servo_id: int, address: int, length: int) -> int:
        self.calls["getData"] += 1
        return 2048


def make_robot(unit: str) -> SO101:
    """Build a connected-looking driver with no port object or connect call."""
    calibration = SO101Calibration.from_dict(
        {
            name: {"id": i, "drive_mode": 0, "homing_offset": 2048, "range_min": 500 + i, "range_max": 3500 - i}
            for i, name in enumerate(SO101_JOINT_ORDER, 1)
        }
    )
    robot = SO101(port="unused-fake-port", calibration=calibration, unit=unit)
    robot._connection = SimpleNamespace(
        port_handler=None,
        group_sync_read=FakeGroup(),
        group_sync_write=FakeGroup(),
        group_sync_write_torque=FakeGroup(),
    )
    robot._torque_enabled = True
    return robot


def make_cases(robot: SO101) -> dict[str, Callable[[], object]]:
    ticks = np.full(6, 2048, dtype=np.int32)
    action = np.array([-50, -25, 0, 25, 50, 50], dtype=np.float32)

    def control() -> None:
        robot.get_observation()
        robot.send_action(action)

    return {
        "ticks_to_unit": lambda: robot._ticks_to_unit(ticks),
        "unit_to_ticks": lambda: robot._unit_to_ticks(action),
        "observation": robot.get_observation,
        "action": lambda: robot.send_action(action),
        "control": control,
    }


def main() -> None:
    """Alternate reviewed baseline/current conversions on identical fake drivers."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--number", type=int, default=10000)
    parser.add_argument("--repeat", type=int, default=7)
    parser.add_argument(
        "--baseline-conversion", type=Path, help="Trusted local _conversion.py exported from the baseline commit"
    )
    args = parser.parse_args()
    baseline = None
    if args.baseline_conversion is not None:
        spec = importlib.util.spec_from_file_location("_so101_benchmark_baseline", args.baseline_conversion)
        if spec is None or spec.loader is None:
            parser.error("Cannot load the baseline conversion module")
        baseline = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = baseline
        spec.loader.exec_module(baseline)
    print(json.dumps({"python": platform.python_version(), "numpy": np.__version__, "platform": platform.platform()}))
    for unit in ("normalized", "degrees"):
        robot = make_robot(unit)
        variants = {"current": make_cases(robot)}
        if baseline is not None:
            old_robot = make_robot(unit)
            old_robot._tables = baseline.CalibrationTables.from_calibration(old_robot._calibration)
            variants["baseline"] = make_cases(old_robot)
        for name in variants["current"]:
            samples = {variant: [] for variant in variants}
            for cases in variants.values():
                timeit.timeit(cases[name], number=1000)
            for repeat in range(args.repeat):
                order = list(variants) if repeat % 2 == 0 else list(reversed(variants))
                for variant in order:
                    seconds = timeit.timeit(variants[variant][name], number=args.number)
                    samples[variant].append(seconds / args.number * 1e6)
            result = {
                "unit": unit,
                "case": name,
                "median_us": {variant: statistics.median(values) for variant, values in samples.items()},
                "batch_us": samples,
            }
            if baseline is not None:
                result["paired_reduction_percent"] = statistics.median(
                    [(1 - new / old) * 100 for old, new in zip(samples["baseline"], samples["current"], strict=True)]
                )
            print(json.dumps(result))
        groups = {
            name: getattr(robot._connection, name)
            for name in ("group_sync_read", "group_sync_write", "group_sync_write_torque")
        }
        for group in groups.values():
            group.calls.clear()
        variants["current"]["control"]()
        print(json.dumps({"unit": unit, "control_calls": {name: dict(group.calls) for name, group in groups.items()}}))


if __name__ == "__main__":
    main()
