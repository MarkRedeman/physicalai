# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
# ruff: noqa: D101, D102, D103, D107, S101, SLF001, PLR2004, ANN401, INP001
# Match the repository's test conventions when linting this excluded tree explicitly.

"""Hardware-free parity and ownership checks for both adapter hot paths."""

from __future__ import annotations

from typing import Any

import numpy as np
import pytest
from physicalai_lerobot_plugin.lerobot_adapter import LeRobotAdapter, LeRobotTeleoperatorAdapter


class FakeDevice:
    is_connected = True

    def __init__(self) -> None:
        self.observation_features = {"z.pos": float, "a.pos": float}
        self.action_features = self.observation_features
        self.values: dict[str, Any] = {"z.pos": 2.0, "a.pos": 1.0}
        self.reads = 0
        self.commands: list[dict[str, Any]] = []

    def get_observation(self) -> dict[str, Any]:
        self.reads += 1
        return self.values

    def get_action(self) -> dict[str, Any]:
        return self.get_observation()

    def send_action(self, action: dict[str, Any]) -> None:
        self.commands.append(action)

    def send_feedback(self, action: dict[str, Any]) -> None:
        self.send_action(action)


@pytest.fixture(params=[LeRobotAdapter, LeRobotTeleoperatorAdapter])
def adapter_and_device(request: pytest.FixtureRequest) -> tuple[Any, FakeDevice]:
    device = FakeDevice()
    keyword = "_robot" if request.param is LeRobotAdapter else "_teleoperator"
    return request.param("fake", {}, role="follower", **{keyword: device}), device


def test_observations_keep_fresh_positions_and_dynamic_auxiliary_data(
    adapter_and_device: tuple[Any, FakeDevice],
) -> None:
    adapter, device = adapter_and_device
    first = adapter.get_observation()
    assert first.sensor_data is None
    assert first.images is None
    assert adapter.joint_names == ["a", "z"]
    assert first.joint_positions.dtype == np.float32
    np.testing.assert_array_equal(first.joint_positions, [1.0, 2.0])

    vector = np.array([3.0, 4.0], dtype=np.float64)
    image = np.zeros((2, 2, 3), dtype=np.uint8)
    device.values.update({"a.pos": 9.0, "vector": vector, "camera": image, "scalar": 4, "ignored": "text"})
    second = adapter.get_observation()
    assert second.timestamp >= first.timestamp
    np.testing.assert_array_equal(second.joint_positions, [9.0, 2.0])
    np.testing.assert_array_equal(first.joint_positions, [1.0, 2.0])
    assert not np.shares_memory(first.joint_positions, second.joint_positions)
    assert second.sensor_data["vector"] is vector
    assert second.sensor_data["scalar"].dtype == np.float32
    np.testing.assert_array_equal(second.sensor_data["scalar"], [4.0])
    assert set(second.sensor_data) == {"vector", "scalar"}
    assert second.images["camera"].data is image
    assert second.images["camera"].sequence == 0

    # Auxiliary schemas and value types can change between calls.
    device.values["camera"] = vector
    third = adapter.get_observation()
    assert third.images is None
    assert third.sensor_data["camera"] is vector
    device.values["camera"] = image
    fourth = adapter.get_observation()
    assert fourth.images["camera"].sequence == 1
    assert second.images["camera"].sequence == 0
    assert device.reads == 4


def test_missing_position_still_raises(adapter_and_device: tuple[Any, FakeDevice]) -> None:
    adapter, device = adapter_and_device
    del device.values["a.pos"]
    with pytest.raises(KeyError, match=r"a\.pos"):
        adapter.get_observation()
    assert device.reads == 1


@pytest.mark.parametrize("dtype", [np.float32, np.float64, np.int32, object])
def test_actions_keep_order_python_floats_and_independent_dictionaries(
    adapter_and_device: tuple[Any, FakeDevice], dtype: Any
) -> None:
    adapter, device = adapter_and_device
    action = np.array([1, 2, 3, 4], dtype=dtype)[::2]
    adapter.send_action(action)
    adapter.send_action(-action)
    assert device.commands == [{"a.pos": 1.0, "z.pos": 3.0}, {"a.pos": -1.0, "z.pos": -3.0}]
    assert device.commands[0] is not device.commands[1]
    assert all(type(value) is float for command in device.commands for value in command.values())
    device.commands[0]["a.pos"] = 999
    assert device.commands[1]["a.pos"] == -1.0
    assert device.reads == 0


def test_invalid_or_disconnected_actions_never_write(adapter_and_device: tuple[Any, FakeDevice]) -> None:
    adapter, device = adapter_and_device
    with pytest.raises(ValueError, match="Expected action shape"):
        adapter.send_action(np.zeros((2, 1)))
    device.is_connected = False
    with pytest.raises(ConnectionError, match="not connected"):
        adapter.send_action(np.zeros(2))
    assert device.commands == []


def test_leader_never_writes(adapter_and_device: tuple[Any, FakeDevice]) -> None:
    adapter, device = adapter_and_device
    keyword = "_robot" if isinstance(adapter, LeRobotAdapter) else "_teleoperator"
    leader = type(adapter)("fake", {}, role="leader", **{keyword: device})
    with pytest.raises(RuntimeError, match="Cannot send actions to a leader"):
        leader.send_action(np.zeros(2))
    assert device.commands == []


def test_restored_adapter_rediscovers_keys(adapter_and_device: tuple[Any, FakeDevice]) -> None:
    adapter, device = adapter_and_device
    state = adapter.__getstate__()
    restored = type(adapter).__new__(type(adapter))
    restored.__setstate__(state)
    device.values = {"new.pos": 7.0}
    if isinstance(restored, LeRobotAdapter):
        restored._robot = device
    else:
        restored._teleoperator = device
    observation = restored.get_observation()
    assert restored.joint_names == ["new"]
    assert observation.sensor_data is None
    np.testing.assert_array_equal(observation.joint_positions, [7.0])


def test_joint_key_membership_is_reused(adapter_and_device: tuple[Any, FakeDevice]) -> None:
    adapter, _ = adapter_and_device
    keys = adapter._position_key_set
    assert keys == frozenset({"a.pos", "z.pos"})
    adapter.get_observation()
    adapter.get_observation()
    assert adapter._position_key_set is keys


@pytest.mark.parametrize("keys", [["elbow_position", "gripperpos"], []])
def test_fallback_and_empty_joint_keys(adapter_and_device: tuple[Any, FakeDevice], keys: list[str]) -> None:
    adapter, device = adapter_and_device
    device.values = dict.fromkeys(keys, 1.0)
    device.observation_features = dict(device.values)
    device.action_features = dict(device.values)
    keyword = "_robot" if isinstance(adapter, LeRobotAdapter) else "_teleoperator"
    rebuilt = type(adapter)("fake", {}, role="follower", **{keyword: device})
    observation = rebuilt.get_observation()
    assert observation.sensor_data is None
    assert observation.joint_positions.shape == (len(keys),)
    assert rebuilt._position_key_set == frozenset(keys)
    rebuilt.send_action(np.zeros(len(keys)))
    assert device.commands == [dict.fromkeys(keys, 0.0)]
