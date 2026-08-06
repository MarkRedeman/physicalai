# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

"""Keyboard-controlled switching between policy and teleoperation."""

from __future__ import annotations

import contextlib
import logging
import os
import select
import shutil
import subprocess  # ruff: ignore[suspicious-subprocess-import] # nosec: B404
import sys
import termios
import threading
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

logger = logging.getLogger(__name__)


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


class _AudioCuePlayer:
    """Play speech cues in a daemon thread so control ticks never block."""

    def __init__(self) -> None:
        self._warned = False

    def play(self, message: str) -> None:
        """Start playback of *message* without waiting for it to complete."""
        try:
            threading.Thread(target=self._play, args=(message,), daemon=True).start()
        except RuntimeError:
            self._warn_once("Could not start audio cue thread")

    def _play(self, message: str) -> None:
        speaker_path = shutil.which("espeak")
        player_path = shutil.which("aplay")
        if speaker_path is None or player_path is None:
            self._warn_once("Audio cues unavailable: espeak and aplay must be installed")
            return

        speaker: subprocess.Popen[bytes] | None = None
        try:
            speaker = subprocess.Popen(  # ruff: ignore[subprocess-without-shell-equals-true] # nosec: B603
                [speaker_path, "--stdout", message],
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
            )
            assert speaker.stdout is not None  # ruff: ignore[assert]
            player = subprocess.Popen(  # ruff: ignore[subprocess-without-shell-equals-true] # nosec: B603
                [player_path, "-q"],
                stdin=speaker.stdout,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except OSError as exc:
            if speaker is not None:
                speaker.terminate()
                speaker.wait()
            self._warn_once(f"Audio cues unavailable: {exc}")
            return

        speaker.stdout.close()
        speaker.wait()
        player.wait()

    def _warn_once(self, message: str) -> None:
        if not self._warned:
            logger.warning(message)
            self._warned = True


_MODE_METRICS = {
    ActionMode.POLICY: 0.0,
    ActionMode.ARMING: 1.0,
    ActionMode.HOLD: 2.0,
    ActionMode.TELEOP: 3.0,
}

_AUDIO_CUES = {
    ActionMode.ARMING: "Teleop armed. Align leader.",
    ActionMode.HOLD: "Leader aligned. Teleop ready.",
    ActionMode.TELEOP: "Teleop enabled.",
    ActionMode.POLICY: "Policy enabled.",
}

_TERMINAL_CUES = {
    ActionMode.POLICY: "Policy active. Press 't' to arm teleop.",
    ActionMode.ARMING: "Teleop armed. Press 't' to cancel.",
    ActionMode.HOLD: "Leader aligned. Follower holding. Press 't' to enable teleop.",
    ActionMode.TELEOP: "Teleop active. Press 't' to resume policy.",
}

_ALIGNMENT_STATUS_INTERVAL_S = 1.0


@export_config(class_path="physicalai.runtime.PolicyTeleopSource")
class PolicyTeleopSource(ActionSource):
    """Use a policy normally and switch to an aligned teleoperator on demand.

    Press ``toggle_key`` once to arm teleoperation. The policy remains in
    inference remains active, but the follower holds its position until the
    mapped leader position is within ``position_tolerance`` for
    ``stable_duration_s``. The follower remains held until a second press
    activates teleoperation. Pressing the key again while teleoperating returns
    to policy control.

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
        audio_cues: Whether to announce state changes through ``espeak`` and
            ``aplay``. Enabled by default.
        leader_follows_follower: Whether an actuated, same-morphology leader
            should track follower positions while policy control is active.
        leader_goal_time: Requested seconds for leader tracking commands.
        auto_teleop_delay_s: When positive, command an actuated tracking leader
            to the held follower pose, then automatically enable teleop after
            this delay. Requires ``leader_follows_follower=True``.
    """

    def __init__(
        self,
        policy: PolicySource,
        teleop: TeleopSource,
        *,
        position_tolerance: float = 0.05,
        stable_duration_s: float = 0.25,
        toggle_key: str = "t",
        audio_cues: bool = True,
        leader_follows_follower: bool = False,
        leader_goal_time: float = 0.1,
        auto_teleop_delay_s: float = 0.0,
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
        if leader_goal_time <= 0:
            msg = f"leader_goal_time must be positive, got {leader_goal_time}"
            raise ValueError(msg)
        if auto_teleop_delay_s < 0:
            msg = f"auto_teleop_delay_s must be non-negative, got {auto_teleop_delay_s}"
            raise ValueError(msg)
        if auto_teleop_delay_s > 0 and not leader_follows_follower:
            msg = "auto_teleop_delay_s requires leader_follows_follower=True"
            raise ValueError(msg)

        self._policy = policy
        self._teleop = teleop
        self._position_tolerance = position_tolerance
        self._stable_duration_s = stable_duration_s
        self._toggle_key = toggle_key
        self._audio_cues = audio_cues
        self._leader_follows_follower = leader_follows_follower
        self._leader_goal_time = leader_goal_time
        self._auto_teleop_delay_s = auto_teleop_delay_s
        self._keyboard = _TerminalKeyReader()
        self._audio = _AudioCuePlayer()
        self._mode = ActionMode.POLICY
        self._within_tolerance_since: float | None = None
        self._last_alignment_status: float | None = None
        self._hold_action: np.ndarray | None = None
        self._auto_teleop_started_at: float | None = None
        self._last_countdown: int | None = None
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
        self._last_alignment_status = None
        self._hold_action = None
        self._auto_teleop_started_at = None
        self._last_countdown = None
        self._policy.connect(bus=bus, session_id=session_id)
        try:
            self._teleop.connect(bus=bus, session_id=session_id)
            self._keyboard.enable()
            self._print_mode()
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
            self._toggle(robot_state)

        # Always update policy so asynchronous execution stays warm while the
        # operator aligns or controls the leader arm.
        policy_action = self._policy.update(robot_state, camera_frames, step)

        if self._mode is ActionMode.POLICY:
            if self._leader_follows_follower:
                self._teleop.follow_follower(robot_state.joint_positions, goal_time=self._leader_goal_time)
            action = policy_action
        elif self._mode is ActionMode.ARMING:
            if self._hold_action is None:  # Defensive: ARMING is only entered with an action.
                msg = "Teleop arming has no follower action"
                raise RuntimeError(msg)
            action = self._hold_action
            if self._auto_teleop_delay_s > 0:
                self._update_auto_teleop()
            else:
                self._update_arming(robot_state)
        elif self._mode is ActionMode.HOLD:
            if self._hold_action is None:  # Defensive: HOLD is only entered with an action.
                msg = "Teleop hold has no follower action"
                raise RuntimeError(msg)
            action = self._hold_action
        elif self._mode is ActionMode.TELEOP:
            action = self._teleop.update(robot_state, camera_frames, step)
        else:
            msg = f"Unsupported action mode {self._mode!r}"
            raise RuntimeError(msg)

        self._emit_mode(step)
        return action

    def disconnect(self) -> None:
        """Restore terminal input and disconnect both child sources."""
        self._keyboard.disable()
        try:
            self._teleop.disconnect()
        finally:
            self._policy.disconnect()

    def _toggle(self, robot_state: RobotObservation) -> None:
        if self._mode is ActionMode.POLICY:
            self._hold_action = np.asarray(robot_state.joint_positions).copy()
            self._set_mode(ActionMode.ARMING)
            self._within_tolerance_since = None
            if self._auto_teleop_delay_s > 0:
                self._teleop.follow_follower(self._hold_action, goal_time=self._leader_goal_time)
                self._auto_teleop_started_at = time.monotonic()
                self._last_countdown = None
                self._announce_countdown()
            else:
                self._print("Align leader with follower. Press 't' to cancel.")
        elif self._mode is ActionMode.ARMING:
            self._cancel_arming()
        elif self._mode is ActionMode.HOLD:
            self._set_mode(ActionMode.TELEOP)
        elif self._mode is ActionMode.TELEOP:
            self._set_mode(ActionMode.POLICY)
            self._policy.action_queue.clear()

    def _cancel_arming(self) -> None:
        self._auto_teleop_started_at = None
        self._last_countdown = None
        self._set_mode(ActionMode.POLICY)
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
                self._set_mode(ActionMode.HOLD)
                self._hold_action = follower_action.copy()
        else:
            self._within_tolerance_since = None
            self._print_alignment_status(leader_action, follower_action)

    def _update_auto_teleop(self) -> None:
        if self._auto_teleop_started_at is None:
            msg = "Automatic teleop handoff has no start time"
            raise RuntimeError(msg)
        now = time.monotonic()
        self._announce_countdown(now)
        if now - self._auto_teleop_started_at >= self._auto_teleop_delay_s:
            self._auto_teleop_started_at = None
            self._set_mode(ActionMode.TELEOP)

    def _announce_countdown(self, now: float | None = None) -> None:
        if self._auto_teleop_started_at is None:
            return
        timestamp = time.monotonic() if now is None else now
        remaining = max(0, int(np.ceil(self._auto_teleop_delay_s - (timestamp - self._auto_teleop_started_at))))
        if remaining == self._last_countdown:
            return
        self._last_countdown = remaining
        message = f"Teleop starts in {remaining}. Press '{self._toggle_key}' to cancel."
        self._print(message)
        if self._audio_cues:
            self._audio.play(message)

    def _print_alignment_status(self, leader_action: np.ndarray, follower_action: np.ndarray) -> None:
        now = time.monotonic()
        if self._last_alignment_status is not None and now - self._last_alignment_status < _ALIGNMENT_STATUS_INTERVAL_S:
            return

        errors = follower_action - leader_action
        joints = np.flatnonzero(np.abs(errors) > self._position_tolerance)
        names = getattr(self._teleop, "joint_names", [])
        guidance = ", ".join(
            f"{names[index] if index < len(names) else f'joint {index + 1}'} "
            f"{'increase' if errors[index] > 0 else 'decrease'} ({abs(errors[index]):.1f})"
            for index in joints
        )
        self._print(f"Align leader: {guidance}")
        self._last_alignment_status = now

    def _set_mode(self, mode: ActionMode) -> None:
        if self._leader_follows_follower:
            if (mode is ActionMode.ARMING and self._auto_teleop_delay_s == 0) or mode is ActionMode.TELEOP:
                self._teleop.set_leader_torque(enabled=False)
            elif mode is ActionMode.POLICY:
                self._teleop.set_leader_torque(enabled=True)
        self._mode = mode
        self._print_mode()
        if self._audio_cues:
            self._audio.play(_AUDIO_CUES[mode])

    def _print_mode(self) -> None:
        self._print(_TERMINAL_CUES[self._mode])

    @staticmethod
    def _print(message: str) -> None:
        print(f"[physicalai] {message}", flush=True)  # ruff: ignore[print]

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
