# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

"""Measure SDK startup and simulation CLI preparation without Studio or hardware."""

from __future__ import annotations

import argparse
import importlib
import json
import platform
import statistics
import subprocess
import sys
import time
from collections.abc import Callable
from pathlib import Path
from typing import Any


def _summary(samples: list[float]) -> dict[str, float]:
    ordered = sorted(samples)
    return {
        "mean": statistics.mean(samples),
        "p50": statistics.median(samples),
        "p95": ordered[min(len(ordered) - 1, int(len(ordered) * 0.95))],
        "p99": ordered[min(len(ordered) - 1, int(len(ordered) * 0.99))],
        "max": max(samples),
    }


def _worker(scenario: str) -> None:
    start = time.perf_counter_ns()
    sdk = importlib.import_module("physicalai_studio_plugin")
    if scenario != "sdk":
        getattr(sdk, scenario)
    elapsed = (time.perf_counter_ns() - start) / 1e6
    print(json.dumps({"ms": elapsed, "modules": len(sys.modules), "numpy": "numpy" in sys.modules}))


def _time_calls(operation: Callable[[], object], count: int) -> list[float]:
    for _ in range(100):
        operation()
    samples = []
    for _ in range(count):
        start = time.perf_counter_ns()
        operation()
        samples.append((time.perf_counter_ns() - start) / 1e3)
    return samples


def main() -> None:
    """Print repeatable cold-import and warm CLI timings as JSON."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--worker", choices=["sdk", "shared_robot_name", "SimulationLaunch", "RobotCatalogDefinition"])
    parser.add_argument("--repeats", type=int, default=15)
    parser.add_argument("--calls", type=int, default=1000)
    args = parser.parse_args()
    if args.worker:
        _worker(args.worker)
        return
    if args.repeats < 1 or args.calls < 1:
        parser.error("repeats and calls must be positive")

    scenarios = ["sdk", "shared_robot_name", "SimulationLaunch", "RobotCatalogDefinition"]
    samples: dict[str, list[dict[str, Any]]] = {name: [] for name in scenarios}
    for repeat in range(args.repeats):
        for scenario in scenarios if repeat % 2 == 0 else reversed(scenarios):
            result = subprocess.run(
                [sys.executable, str(Path(__file__).resolve()), "--worker", scenario],
                check=True,
                capture_output=True,
                text=True,
            )
            samples[scenario].append(json.loads(result.stdout))

    from physicalai_studio_plugin import SimulationLaunch, SimulationScene

    simulation = {}
    for size in (2, 32, 256):
        scenes = tuple(SimulationScene(id=f"scene-{i}", display_name=f"Scene {i}") for i in range(size))

        def construct() -> SimulationLaunch:
            return SimulationLaunch(
                scenes=scenes,
                default_scene=scenes[0].id,
                payload_owner_name=lambda _: "benchmark",
                build_argv=lambda **_: ["simulation", "start"],
            )

        launch = construct()
        simulation[str(size)] = {
            "construct_us": _summary(_time_calls(construct, args.calls)),
            "argv_us": _summary(_time_calls(lambda: launch.argv(scenes[0].id, None), args.calls)),
        }

    # Collect provenance after measurement, without warming child imports.
    from importlib.metadata import version

    revision = subprocess.run(
        ["git", "-C", str(Path(__file__).resolve().parent), "rev-parse", "HEAD"],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()
    print(
        json.dumps(
            {
                "revision": revision,
                "python": sys.version,
                "platform": platform.platform(),
                "dependencies": {name: version(name) for name in ("physicalai", "pydantic", "numpy")},
                "repeats": args.repeats,
                "calls": args.calls,
                "errors": 0,
                "cold": {
                    name: {
                        "ms": _summary([sample["ms"] for sample in values]),
                        "modules": values[-1]["modules"],
                        "numpy": values[-1]["numpy"],
                    }
                    for name, values in samples.items()
                },
                "simulation": simulation,
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
