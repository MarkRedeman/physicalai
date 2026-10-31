# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

from typing import TYPE_CHECKING
from uuid import uuid4

import pytest

from physicalai.robot.errors import RobotDeviceAlreadyOwned
from physicalai.robot.transport import claim_devices
from physicalai.robot.transport._lock import acquire_locks, registered_owner_names

if TYPE_CHECKING:
    from pathlib import Path


def test_claims_conflict_and_release() -> None:
    device = f"fake:{uuid4().hex}"
    with claim_devices([device]):
        with pytest.raises(RobotDeviceAlreadyOwned) as caught:
            with claim_devices([device]):
                pytest.fail("device claimed twice")
        assert caught.value.device_ids == (device,)
    with claim_devices([device]):
        pass


def test_partial_composite_claim_rolls_back() -> None:
    prefix = uuid4().hex
    left, right = f"fake:{prefix}:a", f"fake:{prefix}:b"
    with claim_devices([right]):
        with pytest.raises(RobotDeviceAlreadyOwned):
            with claim_devices([left, right]):
                pytest.fail("composite claim should fail")
        with claim_devices([left]):
            pass


def test_claim_releases_on_exception() -> None:
    device = f"fake:{uuid4().hex}"
    with pytest.raises(RuntimeError, match="connection failed"):
        with claim_devices([device]):
            raise RuntimeError("connection failed")
    with claim_devices([device]):
        pass


def test_virtual_robot_has_no_device_claims() -> None:
    with claim_devices([]), claim_devices([]):
        pass


def test_direct_claim_conflicts_with_shared_owner_locks(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("XDG_RUNTIME_DIR", str(tmp_path))
    device = f"fake:{uuid4().hex}"
    locks = acquire_locks(f"test-{uuid4().hex}", [device])
    try:
        with pytest.raises(RobotDeviceAlreadyOwned):
            with claim_devices([device]):
                pytest.fail("shared owner and direct claim overlap")
    finally:
        locks.release_all()
    with claim_devices([device]):
        pass


def test_claim_does_not_advertise_a_phantom_shared_owner(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("XDG_RUNTIME_DIR", str(tmp_path))
    with claim_devices([f"fake:{uuid4().hex}"]):
        assert registered_owner_names() == []
