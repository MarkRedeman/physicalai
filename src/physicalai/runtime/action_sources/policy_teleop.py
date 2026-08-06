# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

"""Keyboard-controlled switching between policy and teleoperation."""

from __future__ import annotations

import contextlib
import os
import select
import sys
import termios
import time
import tty
from enum import StrEnum
from typing import TYPE_CHECKING, Any

import numpy as np

from physicalai.config import export_config
from physicalai.runtime.action_sources.base import ActionSource
from physicalai.runtime.events import MetricsEvent

if TYPE_CHECKING:
    from collections.abc import Mapping

    from physicalai.capture.frame import Frame
    from physicalai.robot.interface import RobotObservation
    from physicalai.runtime._callback_bus import _CallbackBus
    from physicalai.runtime.action_sources.policy import PolicySource
    from physicalai.runtime.action_sources.teleop import TeleopSource


class ActionMode(StrEnum):
    """Control state of :class:`PolicyTeleopSource`."""

    POLICY = "policy"
    ARMING = "arming"
    HOLD = "hold"
    TELEOP = "teleop"


class _TerminalKeyReader:
    """Read one key without blocking the runtime loop when stdin is a terminal."""

    def __init__(self) -> None:
        self._fd: int | None = None
        self._settings: Any = None

    def enable(self) -> None:
        """Put stdin into cbreak mode when it is safe to do so."""
        with contextlib.suppress(OSError, ValueError):
            fd = sys.stdin.fileno()
            if not os.isatty(fd):
                return
            self._fd = fd
            self._settings = termios.tcgetattr(fd)
            tty.setcbreak(fd)

    def read_key(self) -> str | None:
        """Return one pending key, if any."""
        if self._fd is None:
            return None
        readable, _, _ = select.select([self._fd], [], [], 0)
        if not readable:
            return None
        return os.read(self._fd, 1).decode(errors="ignore") or None

    def disable(self) -> None:
        """Restore terminal settings changed by :meth:`enable`."""
        if self._fd is not None and self._settings is not None:
            with contextlib.suppress(OSError):
                termios.tcsetattr(self._fd, termios.TCSADRAIN, self._settings)
        self._fd = None
        self._settings = None


_MODE_METRICS = {
    ActionMode.POLICY: 0.0,
    ActionMode.ARMING: 1.0,
    ActionMode.HOLD: 2.0,
    ActionMode.TELEOP: 3.0,
}


@export_config(class_path="physicalai.runtime.PolicyTeleopSource")
class PolicyTeleopSource(ActionSource):
    """Use a policy normally and switch to an aligned teleoperator on demand.

    Press ``toggle_key`` once to arm teleoperation. The policy remains in
    control until the mapped leader position is within ``position_tolerance``
    of the follower for ``stable_duration_s``. The follower then holds its
    current position until a second press activates teleoperation. Pressing the
    key again while teleoperating returns to policy control.

    The policy continues inference in all states, so an intervention does not
    require restarting its execution worker. Its queued actions are discarded
    whenever policy control resumes to prevent stale actions being sent.

    Args:
        policy: Policy-backed source kept running throughout the session.
        teleop: Teleoperation source used after the guarded handoff.
        position_tolerance: Maximum per-joint absolute leader/follower error
            accepted before teleoperation can be armed.
        stable_duration_s: Time the leader must stay within tolerance before
            entering the hold state.
        toggle_key: Single keyboard key used to arm, engage, and exit teleop.
    """

    def __init__(
        self,
        policy: PolicySource,
        teleop: TeleopSource,
        *,
        position_tolerance: float = 0.05,
        stable_duration_s: float = 0.25,
        toggle_key: str = "t",
    ) -> None:
        """Initialize the policy/teleoperation handoff source.

        Raises:
            ValueError: If a tolerance or duration is negative, or the toggle
                key is not one character.
        """
        if position_tolerance < 0:
            msg = f"position_tolerance must be non-negative, got {position_tolerance}"
            raise ValueError(msg)
        if stable_duration_s < 0:
            msg = f"stable_duration_s must be non-negative, got {stable_duration_s}"
            raise ValueError(msg)
        if len(toggle_key) != 1:
            msg = "toggle_key must be exactly one character"
            raise ValueError(msg)

        self._policy = policy
        self._teleop = teleop
        self._position_tolerance = position_tolerance
        self._stable_duration_s = stable_duration_s
        self._toggle_key = toggle_key
        self._keyboard = _TerminalKeyReader()
        self._mode = ActionMode.POLICY
        self._within_tolerance_since: float | None = None
        self._hold_action: np.ndarray | None = None
        self._bus: _CallbackBus | None = None
        self._session_id = ""

    @property
    def mode(self) -> ActionMode:
        """Current control mode, useful for recording and user interfaces."""
        return self._mode

    def connect(self, *, bus: _CallbackBus, session_id: str) -> None:
        """Connect both child sources and prepare keyboard input."""
        self._bus = bus
        self._session_id = session_id
        self._mode = ActionMode.POLICY
        self._within_tolerance_since = None
        self._hold_action = None
        self._policy.connect(bus=bus, session_id=session_id)
        try:
            self._teleop.connect(bus=bus, session_id=session_id)
            self._keyboard.enable()
        except Exception:
            self._policy.disconnect()
            raise

    def update(self, robot_state: RobotObservation, camera_frames: Mapping[str, Frame], step: int) -> np.ndarray:
        """Update the warm policy and return an action from the active state.

        Returns:
            Action to send to the follower this tick.

        Raises:
            RuntimeError: If the hold state has no preserved follower action.
        """
        key = self._keyboard.read_key()
        if key == self._toggle_key:
            self._toggle()

        # Always update policy so asynchronous execution stays warm while the
        # operator aligns or controls the leader arm.
        policy_action = self._policy.update(robot_state, camera_frames, step)

        if self._mode is ActionMode.POLICY:
            action = policy_action
        elif self._mode is ActionMode.ARMING:
            action = policy_action
            self._update_arming(robot_state)
        elif self._mode is ActionMode.HOLD:
            if self._hold_action is None:  # Defensive: HOLD is only entered with an action.
                msg = "Teleop hold has no follower action"
                raise RuntimeError(msg)
            action = self._hold_action
        else:
            action = self._teleop.update(robot_state, camera_frames, step)

        self._emit_mode(step)
        return action

    def disconnect(self) -> None:
        """Restore terminal input and disconnect both child sources."""
        self._keyboard.disable()
        try:
            self._teleop.disconnect()
        finally:
            self._policy.disconnect()

    def _toggle(self) -> None:
        if self._mode is ActionMode.POLICY:
            self._mode = ActionMode.ARMING
            self._within_tolerance_since = None
        elif self._mode is ActionMode.HOLD:
            self._mode = ActionMode.TELEOP
        elif self._mode is ActionMode.TELEOP:
            self._mode = ActionMode.POLICY
            self._policy.action_queue.clear()

    def _update_arming(self, robot_state: RobotObservation) -> None:
        leader_action = self._teleop.update(robot_state, {}, 0)
        follower_action = np.asarray(robot_state.joint_positions)
        if leader_action.shape != follower_action.shape:
            msg = f"Leader action shape {leader_action.shape} does not match follower shape {follower_action.shape}"
            raise ValueError(msg)

        if np.all(np.abs(leader_action - follower_action) <= self._position_tolerance):
            now = time.monotonic()
            if self._within_tolerance_since is None:
                self._within_tolerance_since = now
            if now - self._within_tolerance_since >= self._stable_duration_s:
                self._mode = ActionMode.HOLD
                self._hold_action = follower_action.copy()
        else:
            self._within_tolerance_since = None

    def _emit_mode(self, step: int) -> None:
        if self._bus is not None:
            self._bus.emit_metrics(
                MetricsEvent(
                    session_id=self._session_id,
                    step=step,
                    timestamp=time.time(),
                    values={"action_mode": _MODE_METRICS[self._mode]},
                )
            )
