# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

"""Opt-in release-wheel tests on OS-created pseudo-terminals, never physical ports."""

from __future__ import annotations

import os
import select
import struct
import sys
import time
from collections.abc import Iterator
from typing import Any

import pytest

pytestmark = pytest.mark.skipif(
    os.environ.get("STARARM_TEST_NATIVE_SDK") != "1" or sys.platform != "linux",
    reason="Requires the patched SDK and STARARM_TEST_NATIVE_SDK=1 on Linux; uses only pseudo-terminals",
)


@pytest.fixture
def native_bus() -> Iterator[tuple[Any, int]]:
    from motorbridge_smart_servo import FashionStarServo

    assert callable(getattr(FashionStarServo, "set_angles", None)), "Install the patched native wheel first"
    master, slave = os.openpty()
    bus = None
    try:
        bus = FashionStarServo(os.ttyname(slave), baudrate=1_000_000)
        yield bus, master
    finally:
        if bus is not None:
            bus.close()
        os.close(slave)
        os.close(master)


def _read(fd: int, length: int) -> bytes:
    data = bytearray()
    deadline = time.monotonic() + 1.0
    while len(data) < length:
        remaining = deadline - time.monotonic()
        assert remaining > 0 and select.select([fd], [], [], remaining)[0], "Missing native SDK output"
        data.extend(os.read(fd, length - len(data)))
    return bytes(data)


def _packet(params: bytes) -> bytes:
    header = bytes([0x12, 0x4C, 25, len(params)])
    return header + params + bytes([(sum(header) + sum(params)) & 0xFF])


def test_seven_targets_are_one_vendor_sync_packet(native_bus: tuple[Any, int]) -> None:
    bus, fd = native_bus
    ids = list(range(7))
    angles = [12.5, -12.5, 0.05, -0.05, 0.0, 180.0, 270.0]
    bus.set_angles(ids, angles, multi_turn=True, interval_ms=100)
    # Explicit reference follows the vendor's [subcommand, record length, count]
    # framing. Rust rounds half away from zero, including the +/-0.05 targets.
    raw_angles = [125, -125, 1, -1, 0, 1800, 2700]
    records = b"".join(struct.pack("<BiIHHH", i, raw, 100, 20, 20, 0) for i, raw in zip(ids, raw_angles))
    expected = _packet(bytes([14, 15, 7]) + records)
    assert len(expected) == 113
    assert _read(fd, len(expected)) == expected
    assert not select.select([fd], [], [], 0)[0]


def test_plugin_uses_the_real_native_batch(native_bus: tuple[Any, int]) -> None:
    import numpy as np
    from physicalai_stararm_plugin import StarArm102FLFollower

    bus, fd = native_bus
    robot = StarArm102FLFollower()
    # Attach only the OS-created PTY bus. Do not call connect: it would ping
    # nonexistent servos rather than test this output-only transport boundary.
    robot._bus = bus
    action = np.zeros(7, dtype=np.float32)
    robot.send_action(action, goal_time=0.1)
    records = b"".join(struct.pack("<BiIHHH", i, 0, 100, 20, 20, 0) for i in range(7))
    expected = _packet(bytes([14, 15, 7]) + records)
    assert _read(fd, len(expected)) == expected
    assert not select.select([fd], [], [], 0)[0]
    np.testing.assert_array_equal(action, np.zeros(7, dtype=np.float32))


@pytest.mark.parametrize("multi_turn, interval", [(True, 0), (True, 123), (False, 0), (False, 123)])
def test_batch_records_match_real_addressed_encoder(
    native_bus: tuple[Any, int], multi_turn: bool, interval: int
) -> None:
    bus, fd = native_bus
    ids, angles = [0, 2, 253], [0.05, -0.05, 270.0]
    records = []
    command = None
    for servo_id, angle in zip(ids, angles):
        bus.set_angle(servo_id, angle, multi_turn=multi_turn, interval_ms=interval)
        header = _read(fd, 4)
        payload_and_checksum = _read(fd, header[3] + 1)
        command = header[2]
        records.append(payload_and_checksum[:-1])
    expected = _packet(bytes([command, len(records[0]), 3]) + b"".join(records))
    bus.set_angles(ids, angles, multi_turn=multi_turn, interval_ms=interval)
    assert _read(fd, len(expected)) == expected


@pytest.mark.parametrize(
    "ids, angles, interval",
    [
        ([], [], 100),
        ([0], [], 100),
        ([0, 0], [1.0, 2.0], 100),
        ([254], [0.0], 100),
        ([-1], [0.0], 100),
        ([0, 1], [0.0, float("nan")], 100),
        ([0, 1], [0.0, float("inf")], 100),
        ([0], [0.0], -1),
        ([0], [0.0], 4_096_001),
        (list(range(17)), [0.0] * 17, 100),
    ],
)
def test_validation_rejects_complete_batch_without_writing(
    native_bus: tuple[Any, int], ids: list[int], angles: list[float], interval: int
) -> None:
    bus, fd = native_bus
    with pytest.raises((ValueError, RuntimeError)):
        bus.set_angles(ids, angles, multi_turn=True, interval_ms=interval)
    assert not select.select([fd], [], [], 0)[0]


def test_closed_native_bus_propagates_error(native_bus: tuple[Any, int]) -> None:
    bus, _fd = native_bus
    bus.close()
    with pytest.raises(RuntimeError, match="closed"):
        bus.set_angles([0], [0.0], multi_turn=True, interval_ms=100)
    # No read is attempted: a closed slave can make the PTY master report EOF.


def test_native_boundary_validates_ids_without_python_wrapper(native_bus: tuple[Any, int]) -> None:
    bus, fd = native_bus
    with pytest.raises(RuntimeError, match="servo IDs"):
        bus._inner.set_angles([0, 255], [0.0, 0.0], True, 100)
    assert not select.select([fd], [], [], 0)[0]
