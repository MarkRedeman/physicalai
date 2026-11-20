# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import asyncio
from types import SimpleNamespace

import pytest
from physicalai.config import Config
from physicalai.robot import Robot
from physicalai_studio_plugin import RobotCatalogDefinition, SerialPortInfo, validate_robot_payload_ui
from pydantic import BaseModel, ValidationError

from physicalai_stararm_plugin import BimanualStarArm102, StarArm102FLFollower, StarArm102HDLeader, StarArm102LDLeader
from physicalai_stararm_plugin.bimanual_studio_catalog import get_bimanual_definitions


class _Factory:
    def __init__(self, ports: tuple[str | None, str | None] = ("left-port", "right-port")) -> None:
        self.ports = iter(ports)
        self.calls: list[SerialPortInfo] = []

    async def find_port(self, port: SerialPortInfo) -> str | None:
        self.calls.append(port)
        return next(self.ports)


@pytest.fixture(params=get_bimanual_definitions(), ids=lambda d: d.type)
def definition(request: pytest.FixtureRequest) -> RobotCatalogDefinition:
    return request.param


def _payload(definition: RobotCatalogDefinition) -> BaseModel:
    arms = {side: {"serial_number": side.upper(), "baudrate": 115200} for side in ("left", "right")}
    if definition.role == "leader":
        for arm in arms.values():
            arm["follower_profile"] = "b601"
    return definition.robot_payload.model_validate(arms)


def test_nested_schema_reuses_serial_connection_picker(definition: RobotCatalogDefinition) -> None:
    model = definition.robot_payload
    model.model_rebuild(raise_errors=True)
    validate_robot_payload_ui(model)
    schema = model.model_json_schema()
    assert schema["required"] == ["left", "right"]
    for side in ("left", "right"):
        arm_schema = schema["$defs"][schema["properties"][side]["$ref"].rsplit("/", 1)[1]]
        assert arm_schema["x-physicalai-ui"][0]["bind"] == {
            "connection": "connection_string",
            "serial_number": "serial_number",
        }
    with pytest.raises(ValidationError):
        model.model_validate({"left": {"serial_number": "LEFT"}})
    with pytest.raises(ValidationError, match="connection_string or serial_number"):
        model.model_validate({"left": {}, "right": {"serial_number": "RIGHT"}})


def test_builds_disconnected_serializable_pair(definition: RobotCatalogDefinition) -> None:
    factory = _Factory()
    driver = asyncio.run(definition.robot_builder(SimpleNamespace(payload=_payload(definition)), factory))
    assert isinstance(driver, Robot)
    assert isinstance(driver, BimanualStarArm102)
    expected = {"LD": StarArm102LDLeader, "HD": StarArm102HDLeader, "FL": StarArm102FLFollower}
    arm_type = next(v for k, v in expected.items() if f"_{k}_" in definition.type)
    assert type(driver.left) is type(driver.right) is arm_type
    assert not driver.is_connected()
    assert (driver.left.port, driver.right.port) == ("left-port", "right-port")
    assert driver.left.baudrate == driver.right.baudrate == 115200
    assert len(driver.joint_names) == 14
    assert [p.serial_number for p in factory.calls] == ["LEFT", "RIGHT"]
    assert Config.from_instance(driver).instantiate().device_ids == driver.device_ids
    assert definition.role == driver.role
    assert definition.asset is None
    assert (definition.zero_calibration is not None) == (driver.role == "leader")
    if driver.role == "leader":
        assert driver.left.follower_profile == driver.right.follower_profile == "b601"


@pytest.mark.parametrize("ports", [(None, "right"), ("left", None)])
def test_missing_connection(definition: RobotCatalogDefinition, ports: tuple[str | None, str | None]) -> None:
    with pytest.raises(RuntimeError, match="Robot not found"):
        asyncio.run(definition.robot_builder(SimpleNamespace(payload=_payload(definition)), _Factory(ports)))


def test_duplicate_resolved_port_is_rejected(definition: RobotCatalogDefinition) -> None:
    with pytest.raises(ValueError, match="separate transport"):
        asyncio.run(definition.robot_builder(SimpleNamespace(payload=_payload(definition)), _Factory(("same", "same"))))


def test_probe_requires_both_connections_and_discovers_devices(definition: RobotCatalogDefinition) -> None:
    payload = _payload(definition)

    class _Scanner:
        robots = [SerialPortInfo(connection_string="left-port", serial_number="LEFT")]

        async def find_robots(self) -> None:
            pass

    scanner = _Scanner()
    assert not asyncio.run(definition.probe.is_online(payload, scanner))
    scanner.robots.append(SerialPortInfo(connection_string="right-port", serial_number="RIGHT"))
    assert asyncio.run(definition.probe.is_online(payload, scanner))
    assert asyncio.run(definition.probe.discover(scanner)) == scanner.robots
    assert not asyncio.run(definition.probe.is_online(payload))
