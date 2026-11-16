# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

"""Unit tests for the WidowXAI robot driver.

All hardware communication is mocked — these tests verify the driver's logic
without requiring a physical robot or the trossen-arm package.
"""

from __future__ import annotations

from collections.abc import Generator
from unittest.mock import MagicMock, call, patch

import numpy as np
import pytest

pytest.importorskip("trossen_arm", reason="trossen_arm SDK is not installed")

from physicalai_trossen_plugin.constants import HOME_POSITION

# ---------------------------------------------------------------------------
# Helpers: build the mock trossen_arm SDK module
# ---------------------------------------------------------------------------


def _make_mock_trossen_arm() -> MagicMock:
    """Build a mock ``trossen_arm`` module with minimal behaviour."""
    module = MagicMock()

    driver = MagicMock()
    driver.get_is_configured.return_value = True
    driver.get_all_positions.return_value = [0.0] * 7
    driver.get_all_velocities.return_value = [0.0] * 7
    driver.get_all_external_efforts.return_value = [0.0] * 7
    driver.configure.return_value = None
    driver.cleanup.return_value = None
    driver.set_all_positions.return_value = None
    driver.set_all_modes.return_value = None
    driver.set_all_external_efforts.return_value = None

    module.TrossenArmDriver.return_value = driver
    module.VectorDouble.side_effect = lambda values: list(values)

    module.Model.wxai_v0 = MagicMock(name="Model.wxai_v0")
    module.StandardEndEffector.wxai_v0_follower = MagicMock(name="wxai_v0_follower")
    module.StandardEndEffector.wxai_v0_leader = MagicMock(name="wxai_v0_leader")
    module.Mode.position = MagicMock(name="Mode.position")
    module.Mode.external_effort = MagicMock(name="Mode.external_effort")

    return module


@pytest.fixture
def mock_trossen_arm() -> Generator[MagicMock, None, None]:
    """Inject a mock trossen_arm into sys.modules and into the widowxai module.

    The widowxai module binds ``trossen_arm`` at import time.  If the real
    ``trossen_arm`` package is installed, that binding happens before the test
    fixture runs (triggered by ``from physicalai_trossen_plugin.constants ...``
    importing ``__init__.py`` which eagerly imports ``widowxai``).  We must
    therefore also patch the module-level name inside ``widowxai`` so that
    ``connect()`` / ``disconnect()`` see the mock, not the real SDK.
    """
    mock_module = _make_mock_trossen_arm()
    with (
        patch.dict("sys.modules", {"trossen_arm": mock_module}),
        patch("physicalai_trossen_plugin.widowxai.trossen_arm", mock_module),
    ):
        yield mock_module


def _create_robot(mock_module: MagicMock, role: str = "follower") -> object:
    """Instantiate a WidowXAI and call connect() with the mock SDK active."""
    from physicalai_trossen_plugin import WidowXAI

    robot = WidowXAI(ip="192.168.1.1", role=role)  # type: ignore[arg-type]
    robot.connect()
    return robot


# ---------------------------------------------------------------------------
# Tests: connect lifecycle
# ---------------------------------------------------------------------------


class TestWidowXAIConnect:
    """Tests for connect()."""

    def test_device_ids_scheme_qualified_ip(self, mock_trossen_arm: MagicMock) -> None:
        """device_ids is a single scheme-qualified network address."""
        from physicalai_trossen_plugin import WidowXAI

        robot = WidowXAI(ip="192.168.1.42", role="follower")  # type: ignore[arg-type]
        assert robot.device_ids == ("tcp:192.168.1.42",)

    def test_connect_follower(self, mock_trossen_arm: MagicMock) -> None:
        """Follower connect: configure called with follower end effector, modes set, home sent."""
        driver = mock_trossen_arm.TrossenArmDriver.return_value
        _create_robot(mock_trossen_arm, role="follower")

        configure_call = driver.configure.call_args
        assert configure_call is not None
        assert mock_trossen_arm.StandardEndEffector.wxai_v0_follower in configure_call.args

        # set_all_modes(Mode.position) called exactly once (no duplicated call bug)
        mode_calls = driver.set_all_modes.call_args_list
        position_calls = [c for c in mode_calls if c == call(mock_trossen_arm.Mode.position)]
        assert len(position_calls) == 1

        # set_all_positions called with HOME_POSITION
        driver.set_all_positions.assert_called_once_with(list(HOME_POSITION), 2.0, True)  # noqa: FBT003

    def test_connect_leader(self, mock_trossen_arm: MagicMock) -> None:
        """Leader connect: effort mode first, then position mode with home."""
        driver = mock_trossen_arm.TrossenArmDriver.return_value
        _create_robot(mock_trossen_arm, role="leader")

        configure_call = driver.configure.call_args
        assert configure_call is not None
        assert mock_trossen_arm.StandardEndEffector.wxai_v0_leader in configure_call.args

        # Call order: external_effort mode → zero efforts → position mode → home
        mode_calls = driver.set_all_modes.call_args_list
        assert len(mode_calls) == 2
        assert mode_calls[0] == call(mock_trossen_arm.Mode.position)
        assert mode_calls[1] == call(mock_trossen_arm.Mode.external_effort)

        driver.set_all_external_efforts.assert_called_once_with(list(HOME_POSITION), 0.0, False)  # noqa: FBT003
        driver.set_all_positions.assert_called_once_with(list(HOME_POSITION), 2.0, True)  # noqa: FBT003

    def test_connect_failure_cleans_up(self, mock_trossen_arm: MagicMock) -> None:
        """If configure raises, cleanup is called and driver is reset to None."""
        from physicalai_trossen_plugin import WidowXAI

        driver = mock_trossen_arm.TrossenArmDriver.return_value
        driver.configure.side_effect = RuntimeError("hardware error")

        robot = WidowXAI(ip="192.168.1.1", role="follower")

        with pytest.raises(RuntimeError, match="hardware error"):
            robot.connect()

        driver.cleanup.assert_called_once()
        assert getattr(robot, "_driver") is None


# ---------------------------------------------------------------------------
# Tests: disconnect lifecycle
# ---------------------------------------------------------------------------


class TestWidowXAIDisconnect:
    """Tests for disconnect()."""

    def test_disconnect_homes_and_cleans(self, mock_trossen_arm: MagicMock) -> None:
        """disconnect() sets position mode, homes the arm, and calls cleanup."""
        driver = mock_trossen_arm.TrossenArmDriver.return_value
        robot = _create_robot(mock_trossen_arm, role="follower")

        # Reset call counts accumulated during connect
        driver.set_all_modes.reset_mock()
        driver.set_all_positions.reset_mock()
        driver.cleanup.reset_mock()

        robot.disconnect()  # type: ignore[union-attr]

        driver.set_all_modes.assert_called_once_with(mock_trossen_arm.Mode.position)
        driver.set_all_positions.assert_called_once_with(list(HOME_POSITION), 2.0, True)  # noqa: FBT003
        driver.cleanup.assert_called_once()
        assert getattr(robot, "_driver") is None

    def test_disconnect_cleans_on_home_failure(self, mock_trossen_arm: MagicMock) -> None:
        """Even if homing fails during disconnect, cleanup is still called."""
        driver = mock_trossen_arm.TrossenArmDriver.return_value
        robot = _create_robot(mock_trossen_arm, role="follower")

        # Make set_all_modes fail during disconnect
        driver.set_all_modes.side_effect = RuntimeError("motor fault")

        robot.disconnect()  # type: ignore[union-attr]

        # cleanup must still be called (finally block)
        driver.cleanup.assert_called()
        assert getattr(robot, "_driver") is None


# ---------------------------------------------------------------------------
# Tests: connect/disconnect idempotency
# ---------------------------------------------------------------------------


class TestWidowXAIIdempotency:
    """Tests for connect/disconnect idempotency."""

    def test_connect_and_disconnect_idempotent(self, mock_trossen_arm: MagicMock) -> None:
        """Double connect creates driver once; disconnect when not connected is a no-op."""
        from physicalai_trossen_plugin import WidowXAI

        robot = WidowXAI(ip="192.168.1.1", role="follower")

        # disconnect before connect is a no-op
        robot.disconnect()

        # double connect only creates driver once
        robot.connect()
        robot.connect()
        assert mock_trossen_arm.TrossenArmDriver.call_count == 1


# ---------------------------------------------------------------------------
# Tests: observation
# ---------------------------------------------------------------------------


class TestWidowXAIObservation:
    """Tests for get_observation()."""

    def test_get_observation_follower(self, mock_trossen_arm: MagicMock) -> None:
        """Follower state contains positions while velocities remain auxiliary data."""
        driver = mock_trossen_arm.TrossenArmDriver.return_value
        driver.get_all_positions.return_value = [1.0, 0.5, -0.5, 1.5, -1.0, 0.3, 0.02]
        driver.get_all_velocities.return_value = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]
        robot = _create_robot(mock_trossen_arm, role="follower")
        obs = robot.get_observation()  # type: ignore[union-attr]

        assert isinstance(obs.joint_positions, np.ndarray)
        assert obs.joint_positions.shape == (7,)
        assert obs.joint_positions.dtype == np.float32
        assert isinstance(obs.timestamp, float)
        assert obs.sensor_data is not None
        assert "velocities" in obs.sensor_data
        assert "efforts" in obs.sensor_data
        assert obs.sensor_data["velocities"].shape == (7,)
        assert obs.state.shape == (7,)
        np.testing.assert_array_equal(obs.state, obs.joint_positions)
        assert obs.joint_positions[0] == pytest.approx(57.2958, abs=0.01)
        assert obs.joint_positions[5] == pytest.approx(17.1887, abs=0.01)
        assert obs.joint_positions[6] == pytest.approx(0.02, abs=1e-6)

    def test_get_observation_leader(self, mock_trossen_arm: MagicMock) -> None:
        """Leader observation has velocities but NOT efforts."""
        robot = _create_robot(mock_trossen_arm, role="leader")
        obs = robot.get_observation()  # type: ignore[union-attr]

        assert obs.sensor_data is not None
        assert "velocities" in obs.sensor_data
        assert "efforts" not in obs.sensor_data


# ---------------------------------------------------------------------------
# Tests: action
# ---------------------------------------------------------------------------


class TestWidowXAIAction:
    """Tests for send_action()."""

    def test_send_action_leader_raises(self, mock_trossen_arm: MagicMock) -> None:
        """Leader raises RuntimeError on send_action()."""
        robot = _create_robot(mock_trossen_arm, role="leader")

        with pytest.raises(RuntimeError, match="Cannot send actions to a leader arm"):
            robot.send_action(np.zeros(7, dtype=np.float32))  # type: ignore[union-attr]

    def test_send_action_wrong_shape_raises(self, mock_trossen_arm: MagicMock) -> None:
        """ValueError on wrong action shape."""
        robot = _create_robot(mock_trossen_arm, role="follower")

        with pytest.raises(ValueError, match="Expected at least"):
            robot.send_action(np.zeros(3, dtype=np.float32))  # type: ignore[union-attr]

    def test_send_action_accepts_degrees_and_sends_radians(self, mock_trossen_arm: MagicMock) -> None:
        """send_action expects degrees for non-gripper joints and converts to radians."""
        driver = mock_trossen_arm.TrossenArmDriver.return_value
        driver.get_all_positions.return_value = [0.0] * 7
        robot = _create_robot(mock_trossen_arm, role="follower")

        driver.set_all_positions.reset_mock()

        action_deg = np.array([14.32395, 14.32395, -14.32395, 14.32395, -14.32395, 14.32395, 0.05], dtype=np.float32)
        robot.send_action(action_deg, goal_time=0.1)  # type: ignore[union-attr]

        call_args = driver.set_all_positions.call_args
        assert call_args is not None
        sent = call_args.args[0]
        assert sent[0] == pytest.approx(0.25, abs=0.01)
        assert sent[1] == pytest.approx(0.25, abs=0.01)
        assert sent[6] == pytest.approx(0.05, abs=1e-6)
        assert call_args.args[1] == pytest.approx(0.1, abs=1e-6)


# ---------------------------------------------------------------------------
# Tests: set_external_efforts
# ---------------------------------------------------------------------------


class TestWidowXAIExternalEfforts:
    """Tests for set_external_efforts()."""

    def test_set_external_efforts_leader(self, mock_trossen_arm: MagicMock) -> None:
        """Leader set_external_efforts switches mode and sends negated efforts."""
        driver = mock_trossen_arm.TrossenArmDriver.return_value
        robot = _create_robot(mock_trossen_arm, role="leader")

        # Reset mocks after connect to isolate this call
        driver.set_all_modes.reset_mock()
        driver.set_all_external_efforts.reset_mock()

        efforts = np.array([1.0] * 7, dtype=np.float32)
        robot.set_external_efforts(efforts, gain=1.0)  # type: ignore[union-attr]

        driver.set_all_modes.assert_called_once_with(mock_trossen_arm.Mode.external_effort)

        effort_call = driver.set_all_external_efforts.call_args
        assert effort_call is not None
        sent_efforts = effort_call.args[0]
        # Input [1.0]*7 with gain=1.0 → SDK receives [-1.0]*7 (negated)
        assert sent_efforts == [-1.0] * 7

    def test_set_external_efforts_follower_raises(self, mock_trossen_arm: MagicMock) -> None:
        """Follower raises RuntimeError on set_external_efforts()."""
        robot = _create_robot(mock_trossen_arm, role="follower")

        with pytest.raises(RuntimeError, match="only available for leader"):
            robot.set_external_efforts(np.zeros(7, dtype=np.float32))  # type: ignore[union-attr]


@pytest.mark.parametrize("role", ["leader", "follower"])
def test_observation_scalar_parity_and_owned_buffers(mock_trossen_arm: MagicMock, role: str) -> None:
    """Vector conversion matches float32 scalar rounding and never aliases SDK state."""
    robot = _create_robot(mock_trossen_arm, role=role)
    driver = mock_trossen_arm.TrossenArmDriver.return_value
    rng = np.random.default_rng(42)
    vectors = list(rng.uniform(-np.pi, np.pi, (100, 7)))
    vectors.extend(
        [
            np.array([0.0, -0.0, np.inf, -np.inf, np.nan, np.finfo(np.float32).tiny, -0.0]),
            np.array([np.pi, -np.pi, 0.25, -0.25, 1e-20, -1e-20, 0.02]),
        ]
    )
    previous = None
    for values in vectors:
        driver.get_all_positions.return_value = values
        driver.get_all_velocities.return_value = values
        driver.get_all_external_efforts.return_value = values
        original = values.copy()
        expected = np.array(values, dtype=np.float32)
        for i in range(6):
            expected[i] = np.rad2deg(expected[i])
        obs = robot.get_observation()
        # Compare bits as well as dtype, including signed zero and NaN.
        np.testing.assert_array_equal(obs.joint_positions.view(np.uint32), expected.view(np.uint32))
        assert obs.joint_positions.dtype == np.float32
        np.testing.assert_array_equal(values, original)
        assert not np.shares_memory(obs.joint_positions, values)
        for sensor in obs.sensor_data.values():
            assert sensor.dtype == np.float32
            assert not np.shares_memory(sensor, values)
            assert not np.shares_memory(sensor, obs.joint_positions)
        if previous is not None:
            assert not np.shares_memory(previous.joint_positions, obs.joint_positions)
        previous = obs
    expected_calls = [call.get_all_positions(), call.get_all_velocities()]
    if role == "follower":
        expected_calls.append(call.get_all_external_efforts())
    # Ignore startup calls; every observation keeps the same getter sequence.
    assert driver.mock_calls[-len(expected_calls) :] == expected_calls


@pytest.mark.parametrize("dtype", [np.float32, np.float64, np.int32])
def test_action_scalar_parity_safety_and_input_ownership(mock_trossen_arm: MagicMock, dtype: type) -> None:
    """Preserve scalar rounding, clipping, extra dims, and read-only strided input."""
    robot = _create_robot(mock_trossen_arm)
    driver = mock_trossen_arm.TrossenArmDriver.return_value
    rng = np.random.default_rng(73)
    for _ in range(100):
        action = rng.uniform(-180, 180, 22).astype(dtype)[::2]
        action.flags.writeable = False
        original = action.copy()
        present = rng.uniform(-1, 1, 7).astype(np.float32)
        driver.get_all_positions.return_value = present
        expected = np.asarray(action[:7], dtype=np.float32).copy()
        for i in range(6):
            expected[i] = np.deg2rad(expected[i])
        expected = present + np.clip(expected - present, -0.25, 0.25)
        driver.reset_mock()
        with patch("physicalai_trossen_plugin.widowxai.logger.warning"):
            robot.send_action(action, goal_time=0.37)
        assert driver.mock_calls == [call.get_all_positions(), call.set_all_positions(expected.tolist(), 0.37, False)]
        sent = np.asarray(driver.set_all_positions.call_args.args[0], dtype=np.float32)
        np.testing.assert_array_equal(sent.view(np.uint32), expected.view(np.uint32))
        np.testing.assert_array_equal(action, original)


def test_action_uses_fresh_positions_and_propagates_read_errors(mock_trossen_arm: MagicMock) -> None:
    """Never reuse a previous observation for safety clipping or swallow SDK errors."""
    robot = _create_robot(mock_trossen_arm)
    driver = mock_trossen_arm.TrossenArmDriver.return_value
    driver.get_all_positions.side_effect = [[0.0] * 7, [1.0] * 7, RuntimeError("state unavailable")]
    driver.reset_mock()
    robot.send_action(np.zeros(7, dtype=np.float32))
    with patch("physicalai_trossen_plugin.widowxai.logger.warning"):
        robot.send_action(np.zeros(7, dtype=np.float32))
    assert driver.set_all_positions.call_args_list == [
        call([0.0] * 7, 0.1, False),
        call([0.75] * 7, 0.1, False),
    ]
    with pytest.raises(RuntimeError, match="state unavailable"):
        robot.send_action(np.zeros(7, dtype=np.float32))
    assert driver.set_all_positions.call_count == 2


def test_custom_joint_layout_retains_name_based_conversion(mock_trossen_arm: MagicMock) -> None:
    """Keep compatibility with subclasses placing the gripper at another index."""
    robot = _create_robot(mock_trossen_arm)
    robot.JOINT_ORDER = ["gripper", *robot.joint_names[:-1]]
    driver = mock_trossen_arm.TrossenArmDriver.return_value
    values = np.array([0.02, 0.1, -0.1, 0.2, -0.2, 0.05, -0.05], dtype=np.float32)
    driver.get_all_positions.return_value = values
    obs = robot.get_observation()
    expected = values.copy()
    for i in range(1, 7):
        expected[i] = np.rad2deg(expected[i])
    np.testing.assert_array_equal(obs.joint_positions, expected)
    robot.send_action(expected)
    np.testing.assert_allclose(driver.set_all_positions.call_args.args[0], values, atol=1e-8)


def test_short_sdk_response_keeps_index_error(mock_trossen_arm: MagicMock) -> None:
    """Malformed responses must not silently produce a shorter observation."""
    robot = _create_robot(mock_trossen_arm)
    driver = mock_trossen_arm.TrossenArmDriver.return_value
    driver.get_all_positions.return_value = [0.0] * 3
    with pytest.raises(IndexError):
        robot.get_observation()
