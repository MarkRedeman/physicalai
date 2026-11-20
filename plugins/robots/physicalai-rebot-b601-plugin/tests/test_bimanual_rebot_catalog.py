# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import asyncio
from pathlib import Path
from types import SimpleNamespace

import pytest
from physicalai.config import Config
from physicalai.robot import Robot
from physicalai_studio_plugin import RobotCatalogDefinition, SerialPortInfo, validate_robot_payload_ui
from pydantic import BaseModel, ValidationError

from physicalai_rebot_b601_plugin import BimanualReBotB601, ReBotB601DM, ReBotB601RS
from physicalai_rebot_b601_plugin.bimanual_studio_catalog import get_bimanual_definitions


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
    return definition.robot_payload.model_validate({
        "left": {"connection_string": "can0"} if "RS" in definition.type else {"serial_number": "LEFT"},
        "right": {"connection_string": "can1"} if "RS" in definition.type else {"serial_number": "RIGHT"},
    })


def test_schema_requires_both_nested_arms_and_reuses_connection_metadata(definition: RobotCatalogDefinition) -> None:
    model = definition.robot_payload
    model.model_rebuild(raise_errors=True)
    validate_robot_payload_ui(model)
    schema = model.model_json_schema()
    assert schema["required"] == ["left", "right"]
    for side in ("left", "right"):
        arm_schema = schema["$defs"][schema["properties"][side]["$ref"].rsplit("/", 1)[1]]
        if "DM" in definition.type:
            assert arm_schema["x-physicalai-ui"][0]["bind"] == {
                "connection": "connection_string",
                "serial_number": "serial_number",
            }
        assert arm_schema["properties"]["approach_speed"]["default"] == 30.0
    with pytest.raises(ValidationError):
        model.model_validate({"left": _payload(definition).left.model_dump()})


def test_builds_disconnected_serializable_pair(definition: RobotCatalogDefinition) -> None:
    factory = _Factory()
    driver = asyncio.run(definition.robot_builder(SimpleNamespace(payload=_payload(definition)), factory))
    assert isinstance(driver, Robot)
    assert isinstance(driver, BimanualReBotB601)
    expected = ReBotB601RS if "RS" in definition.type else ReBotB601DM
    assert type(driver.left) is type(driver.right) is expected
    assert not driver.is_connected()
    assert len(driver.joint_names) == 14
    assert Config.from_instance(driver).instantiate().device_ids == driver.device_ids
    assert definition.role == driver.role == "follower"
    assert definition.zero_calibration is not None
    assert definition.asset is None  # Never present a single-arm URDF as a bimanual preview.
    if "DM" in definition.type:
        assert [p.serial_number for p in factory.calls] == ["LEFT", "RIGHT"]
        assert (driver.left.port, driver.right.port) == ("left-port", "right-port")
    else:
        assert factory.calls == []
        assert (driver.left.port, driver.right.port) == ("can0", "can1")


@pytest.mark.parametrize("ports", [(None, "right"), ("left", None)])
def test_missing_dm_connection(ports: tuple[str | None, str | None]) -> None:
    definition = next(d for d in get_bimanual_definitions() if "DM" in d.type)
    with pytest.raises(RuntimeError, match="Robot not found"):
        asyncio.run(definition.robot_builder(SimpleNamespace(payload=_payload(definition)), _Factory(ports)))


def test_duplicate_resolved_dm_port_is_rejected() -> None:
    definition = next(d for d in get_bimanual_definitions() if "DM" in d.type)
    with pytest.raises(ValueError, match="separate transport"):
        asyncio.run(definition.robot_builder(SimpleNamespace(payload=_payload(definition)), _Factory(("same", "same"))))


def test_duplicate_rs_interface_is_rejected() -> None:
    definition = next(d for d in get_bimanual_definitions() if "RS" in d.type)
    payload = definition.robot_payload.model_validate({"left": {}, "right": {}})
    with pytest.raises(ValueError, match="separate transport"):
        asyncio.run(definition.robot_builder(SimpleNamespace(payload=payload), _Factory()))


def test_dm_probe_requires_both_connections_and_discovers_adapters() -> None:
    definition = next(d for d in get_bimanual_definitions() if "DM" in d.type)
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


def test_rs_probe_checks_both_can_interfaces(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    from physicalai_rebot_b601_plugin import studio_catalog

    monkeypatch.setattr(studio_catalog, "_SYSFS_NET_ROOT", tmp_path)
    definition = next(d for d in get_bimanual_definitions() if "RS" in d.type)
    payload = _payload(definition)
    for interface, state in (("can0", "up"), ("can1", "down")):
        path = tmp_path / interface
        path.mkdir()
        (path / "operstate").write_text(state, encoding="utf-8")
    assert not asyncio.run(definition.probe.is_online(payload))
    (tmp_path / "can1/operstate").write_text("up", encoding="utf-8")
    assert asyncio.run(definition.probe.is_online(payload))
    assert asyncio.run(definition.probe.discover(SimpleNamespace())) == []
