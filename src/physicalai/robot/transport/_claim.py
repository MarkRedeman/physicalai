# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

"""Exclusive host-local hardware claims for direct robot connections."""

from __future__ import annotations

from contextlib import contextmanager
from typing import TYPE_CHECKING

from physicalai.robot.errors import RobotDeviceAlreadyOwned

from ._lock import DEVICE_KIND, NamedLock

if TYPE_CHECKING:
    from collections.abc import Generator, Sequence


@contextmanager
def claim_devices(device_ids: Sequence[str]) -> Generator[None, None, None]:
    """Claim physical devices using the shared owner's exclusivity locks.

    Acquire before connecting a direct driver and retain until disconnect has
    completed. This only claims hardware: it never connects, moves, or stops a
    robot. Empty identities are valid for virtual/subscriber robots.

    Args:
        device_ids: Canonical physical identities from ``Robot.device_ids``.

    Yields:
        None while the device claims are held.

    Raises:
        RobotDeviceAlreadyOwned: When another local owner holds a device.
    """
    locks: list[NamedLock] = []
    try:
        # Direct connections have no Zenoh metadata endpoint. Claim devices only,
        # avoiding phantom owner names in shared-robot discovery.
        for device_id in sorted(set(device_ids)):
            lock = NamedLock(DEVICE_KIND, device_id)
            if not lock.acquire():
                msg = "robot devices are already in use"
                raise RobotDeviceAlreadyOwned(msg, phase="device_lock_contention", device_ids=(device_id,))
            locks.append(lock)
        yield
    finally:
        for lock in reversed(locks):
            lock.release()
