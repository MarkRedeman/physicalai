# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

"""Hardware-free CPU benchmark; never opens a serial port.

Run with the package src and Runtime src on PYTHONPATH. Transport timings are
intentionally excluded; call counts describe driver-to-SDK calls, not packets
observed on a real bus.
"""

from __future__ import annotations

import argparse
import json
import platform
import statistics
import timeit
from collections import Counter
from collections.abc import Callable
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
    calibration = SO101Calibration.from_dict({
        name: {"id": i, "drive_mode": 0, "homing_offset": 2048, "range_min": 500 + i, "range_max": 3500 - i}
        for i, name in enumerate(SO101_JOINT_ORDER, 1)
    })
    robot = SO101(port="unused-fake-port", calibration=calibration, unit=unit)
    robot._connection = SimpleNamespace(
        port_handler=None,
        group_sync_read=FakeGroup(),
        group_sync_write=FakeGroup(),
        group_sync_write_torque=FakeGroup(),
    )
    robot._torque_enabled = True
    return robot


def main() -> None:
    """Report repeated batch medians and a single control cycle's call counts."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--number", type=int, default=10000)
    parser.add_argument("--repeat", type=int, default=7)
    args = parser.parse_args()
    print(json.dumps({"python": platform.python_version(), "numpy": np.__version__, "platform": platform.platform()}))
    for unit in ("normalized", "degrees"):
        robot = make_robot(unit)
        ticks = np.full(6, 2048, dtype=np.int32)
        action = np.array([-50, -25, 0, 25, 50, 50], dtype=np.float32)

        def control() -> None:
            robot.get_observation()
            robot.send_action(action)

        cases: dict[str, Callable[[], object]] = {
            "ticks_to_unit": lambda: robot._ticks_to_unit(ticks),
            "unit_to_ticks": lambda: robot._unit_to_ticks(action),
            "observation": robot.get_observation,
            "action": lambda: robot.send_action(action),
            "control": control,
        }
        for name, function in cases.items():
            timeit.timeit(function, number=1000)
            samples = [seconds / args.number * 1e6 for seconds in timeit.repeat(function, number=args.number, repeat=args.repeat)]
            print(json.dumps({"unit": unit, "case": name, "median_us": statistics.median(samples), "batch_us": samples}))
        groups = {name: getattr(robot._connection, name) for name in (
            "group_sync_read", "group_sync_write", "group_sync_write_torque"
        )}
        for group in groups.values():
            group.calls.clear()
        control()
        print(json.dumps({"unit": unit, "control_calls": {name: dict(group.calls) for name, group in groups.items()}}))


if __name__ == "__main__":
    main()
