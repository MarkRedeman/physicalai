# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

from unittest.mock import Mock

import numpy as np
import pytest
from physicalai.config import Config
from physicalai.robot import BimanualRobot, CompositeRobotObservation, Robot
from physicalai_stararm_plugin import (
    BimanualStarArm102,
    BimanualStarArm102Observation,
    StarArm102FLFollower,
    StarArm102HDLeader,
    StarArm102LDLeader,
)


@pytest.fixture(params=[StarArm102LDLeader, StarArm102HDLeader, StarArm102FLFollower], ids=["ld", "hd", "fl"])
def robot(request: pytest.FixtureRequest) -> BimanualStarArm102:
    arm = request.param
    return BimanualStarArm102(left=arm(port="left"), right=arm(port="right"))


def test_protocol_order_and_config_roundtrip(robot: BimanualStarArm102) -> None:
    assert isinstance(robot, Robot)
    assert isinstance(robot, BimanualRobot)
    assert robot.role == ("follower" if isinstance(robot.left, StarArm102FLFollower) else "leader")
    assert robot.NUM_JOINTS == 14
    assert robot.joint_names == list(robot.JOINT_ORDER)
    assert robot.device_ids == tuple(sorted((*robot.left.device_ids, *robot.right.device_ids)))
    config = Config.from_instance(robot)
    assert config.class_path == "physicalai_stararm_plugin.BimanualStarArm102"
    restored = config.instantiate()
    assert isinstance(restored, BimanualStarArm102)
    assert type(restored.left) is type(robot.left)
    assert type(restored.right) is type(robot.right)
    assert Config.from_instance(restored) == config
    assert not restored.is_connected()
    assert BimanualStarArm102Observation is CompositeRobotObservation


@pytest.mark.parametrize("right", [StarArm102HDLeader(port="right"), StarArm102FLFollower(port="right")])
def test_rejects_mixed_variants(right: StarArm102HDLeader | StarArm102FLFollower) -> None:
    with pytest.raises(ValueError, match="same driver type"):
        BimanualStarArm102(StarArm102LDLeader(port="left"), right)


def test_rejects_shared_port_and_incompatible_leader_modes() -> None:
    with pytest.raises(ValueError, match="separate transport"):
        BimanualStarArm102(StarArm102FLFollower(), StarArm102FLFollower())
    with pytest.raises(ValueError, match="same control mode"):
        BimanualStarArm102(StarArm102HDLeader(port="left"), StarArm102HDLeader(port="right", control_mode="assist"))
    with pytest.raises(ValueError, match="same follower profile"):
        BimanualStarArm102(StarArm102HDLeader(port="left"), StarArm102HDLeader(port="right", follower_profile="b601"))


def test_observation_and_action_order(robot: BimanualStarArm102, monkeypatch: pytest.MonkeyPatch) -> None:
    for i, arm in enumerate((robot.left, robot.right)):
        positions = np.arange(7) + i * 7
        monkeypatch.setattr(
            arm,
            "get_observation",
            Mock(
                return_value=CompositeRobotObservation(
                    joint_positions=positions,
                    timestamp=123.0 + i,
                    sensor_data={"reliable": positions},
                )
            ),
        )
        monkeypatch.setattr(arm, "send_action", Mock())
    observation = robot.get_observation()
    np.testing.assert_array_equal(observation.joint_positions, np.arange(14))
    np.testing.assert_array_equal(observation.sensor_data["reliable"], np.arange(14))
    assert observation.timestamp == 123.0
    assert observation.images is None
    robot.send_action(np.arange(14), goal_time=0.25)
    for i, arm in enumerate((robot.left, robot.right)):
        args, kwargs = arm.send_action.call_args
        np.testing.assert_array_equal(args[0], np.arange(7) + i * 7)
        assert kwargs == {"goal_time": 0.25}


@pytest.mark.parametrize("action", [np.zeros(13), np.zeros(15), np.zeros((2, 7))])
def test_invalid_shape_never_commands_either_arm(
    robot: BimanualStarArm102,
    monkeypatch: pytest.MonkeyPatch,
    action: np.ndarray,
) -> None:
    monkeypatch.setattr(robot.left, "send_action", Mock())
    monkeypatch.setattr(robot.right, "send_action", Mock())
    with pytest.raises(ValueError, match="Expected action shape"):
        robot.send_action(action)
    robot.left.send_action.assert_not_called()
    robot.right.send_action.assert_not_called()


@pytest.mark.parametrize("arm_type", [StarArm102LDLeader, StarArm102HDLeader])
def test_passive_leaders_keep_read_only_behavior(arm_type: type[StarArm102HDLeader]) -> None:
    robot = BimanualStarArm102(arm_type(port="left"), arm_type(port="right"))
    robot.send_action(np.zeros(14))
    assert not robot.is_connected()


def test_hd_assist_converts_both_b601_action_slices(monkeypatch: pytest.MonkeyPatch) -> None:
    robot = BimanualStarArm102(
        StarArm102HDLeader(port="left", control_mode="assist", follower_profile="b601"),
        StarArm102HDLeader(port="right", control_mode="assist", follower_profile="b601"),
    )
    for arm in (robot.left, robot.right):
        monkeypatch.setattr(arm, "_send_action_internal", Mock())
    action = np.tile([-10, -20, 30, 40, 50, -60, -270], 2)
    robot.send_action(action, goal_time=0.3)
    for arm in (robot.left, robot.right):
        args, kwargs = arm._send_action_internal.call_args
        np.testing.assert_array_equal(args[0], [10, 20, 30, 40, 50, 60, 45])
        assert kwargs == {"goal_time": 0.3}
    restored = Config.from_instance(robot).instantiate()
    assert restored.left.control_mode == restored.right.control_mode == "assist"
    assert restored.left.follower_profile == restored.right.follower_profile == "b601"


def test_connect_rolls_back_left(robot: BimanualStarArm102, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(robot.left, "connect", Mock())
    monkeypatch.setattr(robot.right, "connect", Mock(side_effect=ConnectionError("right failed")))
    monkeypatch.setattr(robot.left, "disconnect", Mock())
    with pytest.raises(ConnectionError, match="right failed"):
        robot.connect()
    robot.left.disconnect.assert_called_once()


def test_disconnected_reads_and_follower_actions(robot: BimanualStarArm102) -> None:
    robot.disconnect()
    robot.disconnect()
    with pytest.raises(ConnectionError):
        robot.get_observation()
    if robot.role == "follower":
        with pytest.raises(ConnectionError):
            robot.send_action(np.zeros(14))


def test_zero_releases_both_arms_before_storing_origins(
    robot: BimanualStarArm102, monkeypatch: pytest.MonkeyPatch
) -> None:
    events = []
    for side, arm in (("left", robot.left), ("right", robot.right)):
        monkeypatch.setattr(arm, "disable_torque", Mock(side_effect=lambda s=side: events.append(f"release {s}")))
        monkeypatch.setattr(arm, "set_zero_position", Mock(side_effect=lambda s=side: events.append(f"zero {s}")))
    robot.set_zero_position()
    assert events == ["release left", "release right", "zero left", "zero right"]
