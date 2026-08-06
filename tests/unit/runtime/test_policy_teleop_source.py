# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
# ruff:file-ignore[assert, magic-value-comparison, no-self-use, undocumented-public-class, undocumented-public-method, unused-method-argument]

"""Tests for guarded policy/teleoperation handoff."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any
from unittest.mock import MagicMock, patch

import numpy as np
import pytest

from tests.unit.runtime.conftest import FakeRobotObservation

from physicalai.runtime import ActionMode, PolicyTeleopSource


@dataclass
class _Source:
    action: np.ndarray
    updates: int = 0
    connected: bool = False
    disconnected: bool = False
    action_queue: MagicMock = field(default_factory=MagicMock)

    def connect(self, *, bus: Any, session_id: str) -> None:  # noqa: ANN401
        self.connected = True

    def update(self, robot_state: Any, camera_frames: Any, step: int) -> np.ndarray:  # noqa: ANN401
        self.updates += 1
        return self.action

    def disconnect(self) -> None:
        self.disconnected = True


def _source(
    *, policy: np.ndarray, teleop: np.ndarray, stable_duration_s: float = 0.25
) -> tuple[PolicyTeleopSource, _Source, _Source]:
    policy_child = _Source(policy)
    teleop_child = _Source(teleop)
    source = PolicyTeleopSource(
        policy=policy_child,  # type: ignore[arg-type]
        teleop=teleop_child,  # type: ignore[arg-type]
        position_tolerance=0.05,
        stable_duration_s=stable_duration_s,
    )
    source._keyboard = MagicMock()
    return source, policy_child, teleop_child


class TestPolicyTeleopSource:
    def test_connects_and_disconnects_both_children(self) -> None:
        source, policy, teleop = _source(policy=np.array([1.0]), teleop=np.array([1.0]))
        bus = MagicMock()

        source.connect(bus=bus, session_id="session")
        source.disconnect()

        assert policy.connected
        assert teleop.connected
        assert policy.disconnected
        assert teleop.disconnected

    def test_teleop_requires_alignment_then_second_toggle(self) -> None:
        source, policy, teleop = _source(
            policy=np.array([10.0, 11.0]), teleop=np.array([1.01, 2.01]), stable_duration_s=0.25
        )
        keyboard = source._keyboard
        keyboard.read_key.side_effect = ["t", None, "t"]
        observation = FakeRobotObservation(joint_positions=np.array([1.0, 2.0]))
        source.connect(bus=MagicMock(), session_id="session")

        with patch("physicalai.runtime.action_sources.policy_teleop.time.monotonic", side_effect=[0.0, 0.3]):
            assert np.array_equal(source.update(observation, {}, 0), policy.action)
            assert source.mode is ActionMode.ARMING
            assert np.array_equal(source.update(observation, {}, 1), policy.action)
            assert source.mode is ActionMode.HOLD

        # Hold emits the follower position until an explicit second toggle.
        assert np.array_equal(source.update(observation, {}, 2), teleop.action)
        assert source.mode is ActionMode.TELEOP
        assert policy.updates == 3
        assert teleop.updates == 3  # Two alignment reads plus active teleop.

    def test_alignment_must_remain_within_tolerance_for_full_duration(self) -> None:
        source, policy, _teleop = _source(
            policy=np.array([10.0]), teleop=np.array([1.0]), stable_duration_s=0.25
        )
        keyboard = source._keyboard
        keyboard.read_key.side_effect = ["t", None, None, None]
        observation = FakeRobotObservation(joint_positions=np.array([1.0]))
        source.connect(bus=MagicMock(), session_id="session")

        with patch("physicalai.runtime.action_sources.policy_teleop.time.monotonic", side_effect=[0.0, 0.2, 0.6]):
            source.update(observation, {}, 0)
            source._teleop.action = np.array([2.0])
            source.update(observation, {}, 1)
            source._teleop.action = np.array([1.0])
            source.update(observation, {}, 2)
            source.update(observation, {}, 3)

        assert source.mode is ActionMode.HOLD
        assert policy.updates == 4

    def test_toggle_from_teleop_returns_to_policy_and_clears_queue(self) -> None:
        source, policy, _teleop = _source(
            policy=np.array([10.0]), teleop=np.array([1.0]), stable_duration_s=0.0
        )
        keyboard = source._keyboard
        keyboard.read_key.side_effect = ["t", "t", "t"]
        observation = FakeRobotObservation(joint_positions=np.array([1.0]))
        source.connect(bus=MagicMock(), session_id="session")

        source.update(observation, {}, 0)
        source.update(observation, {}, 1)
        result = source.update(observation, {}, 2)

        assert source.mode is ActionMode.POLICY
        assert np.array_equal(result, policy.action)
        policy.action_queue.clear.assert_called_once()

    @pytest.mark.parametrize(
        ("kwargs", "message"),
        [
            ({"position_tolerance": -0.1}, "position_tolerance"),
            ({"stable_duration_s": -0.1}, "stable_duration_s"),
            ({"toggle_key": "toggle"}, "toggle_key"),
        ],
    )
    def test_rejects_invalid_configuration(self, kwargs: dict[str, object], message: str) -> None:
        policy = _Source(np.array([1.0]))
        teleop = _Source(np.array([1.0]))

        with pytest.raises(ValueError, match=message):
            PolicyTeleopSource(policy=policy, teleop=teleop, **kwargs)  # type: ignore[arg-type]
