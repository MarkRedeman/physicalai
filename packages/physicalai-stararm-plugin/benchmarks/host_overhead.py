# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

"""Measure host overhead with an in-memory bus; never import the vendor SDK."""

from __future__ import annotations

import argparse
import json
import platform
import statistics
import sys
import time
from collections import Counter
from types import ModuleType, SimpleNamespace

import numpy as np
from loguru import logger


class FakeBus:
    """Count SDK calls without opening a device or sleeping."""

    def __init__(self, *_args: object, **_kwargs: object) -> None:
        self.calls: Counter[str] = Counter()
        self.monitors = {
            i: SimpleNamespace(angle_deg=0.0, reliable=True) for i in range(7)
        }

    def ping(self, _servo_id: int) -> bool:
        self.calls["ping"] += 1
        return True

    def unlock(self, _servo_id: int) -> None:
        self.calls["unlock"] += 1

    def reset_multi_turn(self, _servo_id: int) -> None:
        self.calls["reset_multi_turn"] += 1

    def sync_monitor(self, _servo_ids: list[int]) -> dict[int, SimpleNamespace]:
        self.calls["sync_monitor"] += 1
        return self.monitors

    def set_angle(self, *_args: object, **_kwargs: object) -> None:
        self.calls["set_angle"] += 1


def main() -> None:
    """Print per-call CPU-only latencies and fake SDK counts as JSON."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--iterations", type=int, default=5000)
    parser.add_argument("--repeats", type=int, default=5)
    args = parser.parse_args()
    # Install before importing the driver: this harness cannot connect to hardware.
    sdk = ModuleType("motorbridge_smart_servo")
    sdk.FashionStarServo = FakeBus
    sys.modules["motorbridge_smart_servo"] = sdk
    from physicalai_stararm_plugin import StarArm102FLFollower, StarArm102HDLeader

    logger.disable("physicalai_stararm_plugin")
    follower = StarArm102FLFollower()
    follower.connect()
    bus = follower._bus
    connect_counts = dict(bus.calls)
    bus.calls.clear()
    follower.get_observation()
    follower.send_action(np.zeros(7, dtype=np.float32))
    control_counts = dict(bus.calls)
    leader = StarArm102HDLeader(follower_profile="b601", control_mode="assist")
    leader.connect()
    action = np.array([12, -30, -40, 20, -50, 60, -120], dtype=np.float32)
    cases = {
        "follower_observation": follower.get_observation,
        "follower_action": lambda: follower.send_action(action),
        "b601_observation": leader.get_observation,
        "b601_action": lambda: leader.send_action(action),
    }
    results = {}
    for name, operation in cases.items():
        for _ in range(1000):
            operation()
        samples = []
        means = []
        for _ in range(args.repeats):
            repeat = []
            for _ in range(args.iterations):
                start = time.perf_counter_ns()
                operation()
                repeat.append((time.perf_counter_ns() - start) / 1000)
            means.append(statistics.mean(repeat))
            samples.extend(repeat)
        results[name] = {
            "repeat_mean_us": means,
            "mean_us": statistics.mean(samples),
            "p50_p95_p99_max_us": [
                float(x) for x in np.percentile(samples, [50, 95, 99, 100])
            ],
        }
    print(
        json.dumps(
            {
                "python": platform.python_version(),
                "numpy": np.__version__,
                "platform": platform.platform(),
                "iterations": args.iterations,
                "repeats": args.repeats,
                "connect_counts": connect_counts,
                "read_plus_action_counts": control_counts,
                "results": results,
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
