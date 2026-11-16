# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

"""Capture real SDK writes on a pseudo-terminal; never accept a physical port."""

from __future__ import annotations

import argparse
import json
import os
import select
import sys
import time
from importlib.metadata import version


def _read(fd: int, length: int) -> bytes:
    result = bytearray()
    deadline = time.monotonic() + 1.0
    while len(result) < length:
        remaining = deadline - time.monotonic()
        if remaining <= 0 or not select.select([fd], [], [], remaining)[0]:
            raise RuntimeError("Timed out waiting for SDK output on the pseudo-terminal")
        result.extend(os.read(fd, length - len(result)))
    return bytes(result)


def _count_packets(data: bytes) -> int:
    offset = count = 0
    while offset < len(data):
        if data[offset : offset + 2] != b"\x12\x4c":
            raise RuntimeError("Unexpected request header")
        end = offset + 5 + data[offset + 3]
        packet = data[offset:end]
        if len(packet) != end - offset or sum(packet[:-1]) & 0xFF != packet[-1]:
            raise RuntimeError("Invalid SDK packet length or checksum")
        count += 1
        offset = end
    return count


def main() -> None:
    """Capture addressed or sync writes and report byte/packet counts, not hardware timings."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=["addressed", "sync"], required=True)
    parser.add_argument("--calls", type=int, default=1)
    args = parser.parse_args()
    if sys.platform != "linux" or args.calls < 1:
        parser.error("Requires Linux and --calls >= 1")

    from motorbridge_smart_servo import FashionStarServo

    master, slave = os.openpty()
    bus = None
    captured_bytes = captured_packets = 0
    try:
        bus = FashionStarServo(os.ttyname(slave), baudrate=1_000_000)
        for _ in range(args.calls):
            if args.mode == "sync":
                bus.set_angles(list(range(7)), [0.0] * 7, multi_turn=True, interval_ms=100)
                data = _read(master, 113)
            else:
                for servo_id in range(7):
                    bus.set_angle(servo_id, 0.0, multi_turn=True, interval_ms=100)
                data = _read(master, 140)
            captured_bytes += len(data)
            captured_packets += _count_packets(data)
        if select.select([master], [], [], 0)[0]:
            raise RuntimeError("Unexpected extra SDK output")
    finally:
        if bus is not None:
            bus.close()
        os.close(slave)
        os.close(master)

    print(
        json.dumps(
            {
                "sdk_version": version("motorbridge-smart-servo"),
                "mode": args.mode,
                "actions": args.calls,
                "captured_packets": captured_packets,
                "captured_bytes": captured_bytes,
                "ideal_wire_us_per_action_at_1mbps_8n1": captured_bytes / args.calls * 10,
                "hardware_accessed": False,
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
