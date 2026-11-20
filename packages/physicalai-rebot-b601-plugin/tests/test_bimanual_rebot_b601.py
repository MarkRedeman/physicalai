# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

from unittest.mock import Mock

import numpy as np
import pytest
from physicalai.config import Config
from physicalai.robot import BimanualRobot, CompositeRobotObservation, Robot
from physicalai_rebot_b601_plugin import BimanualReBotB601, BimanualReBotB601Observation, ReBotB601DM, ReBotB601RS


@pytest.fixture(params=[ReBotB601DM, ReBotB601RS], ids=["dm", "rs"])
def robot(request: pytest.FixtureRequest) -> BimanualReBotB601:
    arm = request.param
    return BimanualReBotB601(left=arm(port="left"), right=arm(port="right"))


def test_protocol_order_and_config_roundtrip(robot: BimanualReBotB601) -> None:
    assert isinstance(robot, Robot)
    assert isinstance(robot, BimanualRobot)
    assert robot.role == "follower"
    assert robot.NUM_JOINTS == 14
    assert robot.joint_names == list(robot.JOINT_ORDER)
    assert robot.joint_names == [f"{side}_{joint}" for side in ("left", "right") for joint in robot.left.joint_names]
    assert robot.device_ids == tuple(sorted((*robot.left.device_ids, *robot.right.device_ids)))
    config = Config.from_instance(robot)
    assert config.class_path == "physicalai_rebot_b601_plugin.BimanualReBotB601"
    restored = config.instantiate()
    assert isinstance(restored, BimanualReBotB601)
    assert type(restored.left) is type(robot.left)
    assert type(restored.right) is type(robot.right)
    assert Config.from_instance(restored) == config
    assert not restored.is_connected()
    assert BimanualReBotB601Observation is CompositeRobotObservation


def test_rejects_mixed_variants_and_shared_connection() -> None:
    with pytest.raises(ValueError, match="same driver type"):
        BimanualReBotB601(ReBotB601DM(port="left"), ReBotB601RS(port="right"))
    with pytest.raises(ValueError, match="separate transport"):
        BimanualReBotB601(ReBotB601DM(), ReBotB601DM())


def test_observation_and_action_order(robot: BimanualReBotB601, monkeypatch: pytest.MonkeyPatch) -> None:
    for i, arm in enumerate((robot.left, robot.right)):
        positions = np.arange(7) + i * 7
        monkeypatch.setattr(
            arm,
            "get_observation",
            Mock(
                return_value=CompositeRobotObservation(
                    joint_positions=positions,
                    timestamp=123.0 + i,
                    sensor_data={"torques": positions * 2},
                )
            ),
        )
        monkeypatch.setattr(arm, "send_action", Mock())
    observation = robot.get_observation()
    np.testing.assert_array_equal(observation.joint_positions, np.arange(14))
    np.testing.assert_array_equal(observation.state, np.arange(14))
    np.testing.assert_array_equal(observation.sensor_data["torques"], np.arange(14) * 2)
    assert observation.timestamp == 123.0
    assert observation.images is None
    action = np.arange(14, dtype=np.float32)
    robot.send_action(action, goal_time=0.25)
    for i, arm in enumerate((robot.left, robot.right)):
        args, kwargs = arm.send_action.call_args
        np.testing.assert_array_equal(args[0], action[i * 7 : (i + 1) * 7])
        assert kwargs == {"goal_time": 0.25}


@pytest.mark.parametrize(
    "action",
    [np.zeros(13), np.zeros(15), np.zeros((2, 7)), np.full(14, np.nan), np.concatenate([np.zeros(13), [np.inf]])],
)
def test_invalid_action_never_commands_either_arm(
    robot: BimanualReBotB601,
    monkeypatch: pytest.MonkeyPatch,
    action: np.ndarray,
) -> None:
    monkeypatch.setattr(robot.left, "send_action", Mock())
    monkeypatch.setattr(robot.right, "send_action", Mock())
    with pytest.raises(ValueError):
        robot.send_action(action)
    robot.left.send_action.assert_not_called()
    robot.right.send_action.assert_not_called()


def test_disconnected_observation_and_action(robot: BimanualReBotB601) -> None:
    robot.disconnect()
    robot.disconnect()
    with pytest.raises(ConnectionError):
        robot.get_observation()
    with pytest.raises(ConnectionError):
        robot.send_action(np.zeros(14))


@pytest.mark.parametrize("left_already_connected", [False, True])
def test_connect_rolls_back_only_new_connections(
    robot: BimanualReBotB601,
    monkeypatch: pytest.MonkeyPatch,
    left_already_connected: bool,
) -> None:
    monkeypatch.setattr(robot.left, "is_connected", Mock(return_value=left_already_connected))
    monkeypatch.setattr(robot.right, "is_connected", Mock(return_value=False))
    monkeypatch.setattr(robot.left, "connect", Mock())
    monkeypatch.setattr(robot.right, "connect", Mock(side_effect=ConnectionError("right failed")))
    monkeypatch.setattr(robot.left, "disconnect", Mock())
    with pytest.raises(ConnectionError, match="right failed"):
        robot.connect()
    assert robot.left.disconnect.call_count == (0 if left_already_connected else 1)


def test_disconnect_attempts_both_arms(robot: BimanualReBotB601, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(robot.left, "disconnect", Mock(side_effect=ConnectionError("left failed")))
    monkeypatch.setattr(robot.right, "disconnect", Mock())
    with pytest.raises(ConnectionError, match="left failed"):
        robot.disconnect()
    robot.right.disconnect.assert_called_once()


def test_zero_releases_both_arms_before_storing_origins(
    robot: BimanualReBotB601, monkeypatch: pytest.MonkeyPatch
) -> None:
    events = []
    for side, arm in (("left", robot.left), ("right", robot.right)):
        monkeypatch.setattr(arm, "disable_torque", Mock(side_effect=lambda s=side: events.append(f"release {s}")))
        monkeypatch.setattr(arm, "set_zero_position", Mock(side_effect=lambda s=side: events.append(f"zero {s}")))
    robot.set_zero_position()
    assert events == ["release left", "release right", "zero left", "zero right"]
