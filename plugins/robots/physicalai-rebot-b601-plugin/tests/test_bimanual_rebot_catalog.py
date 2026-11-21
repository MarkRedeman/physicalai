# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import asyncio
from pathlib import Path
from types import SimpleNamespace
from xml.etree import ElementTree as ET

import pytest
from defusedxml.ElementTree import parse
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
    assert definition.asset is not None
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


def test_bimanual_asset_tree_and_joint_mapping(definition: RobotCatalogDefinition) -> None:
    from physicalai_rebot_b601_plugin.studio_catalog import _definitions

    asset = definition.asset
    single = next(d.asset for d in _definitions() if d.type == definition.type.replace("Bimanual_", ""))
    assert asset is not None and single is not None
    assert asset.root_resolver is not None and single.root_resolver is not None
    root = asset.root_resolver()
    variant = "rs" if "RS" in definition.type else "dm"
    assert asset.urdf_relative_path == Path(f"rebot-b601-{variant}/urdf/rebot_b601_bimanual_{variant}.urdf")
    assert asset.packages == single.packages
    robot = parse(root / asset.urdf_relative_path).getroot()
    arm = parse(single.root_resolver() / single.urdf_relative_path).getroot()
    links = [link.attrib["name"] for link in robot.findall("link")]
    joints = {joint.attrib["name"]: joint for joint in robot.findall("joint")}
    assert len(links) == len(set(links)) == 1 + 2 * len(arm.findall("link"))
    assert len(joints) == len(robot.findall("joint")) == 2 + 2 * len(arm.findall("joint"))
    parents = {joint.find("child").attrib["link"]: joint.find("parent").attrib["link"] for joint in joints.values()}
    assert len(parents) == len(joints)
    assert set(links) - parents.keys() == {"world"}
    for link in links:
        visited = set()
        while link != "world":
            assert link not in visited
            visited.add(link)
            link = parents[link]
    expected_map = {
        f"{side}_{key}": [f"{side}_{joint}" for joint in names]
        for side in ("left", "right")
        for key, names in single.joint_map.items()
    }
    assert asset.joint_map == expected_map
    assert len(asset.joint_map) == 14
    for names in asset.joint_map.values():
        assert all(name in joints and joints[name].attrib["type"] == "revolute" for name in names)
    for side, offset in (("left", "0 0.2 0"), ("right", "0 -0.2 0")):
        mount = joints[f"{side}_mount"]
        assert mount.attrib["type"] == "fixed"
        assert mount.find("origin").attrib == {"xyz": offset, "rpy": "0 0 0"}
        assert mount.find("parent").attrib["link"] == "world"
        assert mount.find("child").attrib["link"] == f"{side}_base_link"
        for original in arm.findall("joint"):
            copied = joints[f"{side}_{original.attrib['name']}"]
            assert copied.attrib["type"] == original.attrib["type"]
            for tag in ("origin", "axis", "limit"):
                before, after = original.find(tag), copied.find(tag)
                if before is None:
                    assert after is None
                else:
                    assert after is not None and after.attrib == before.attrib
            for tag in ("parent", "child"):
                assert copied.find(tag).attrib["link"] == f"{side}_{original.find(tag).attrib['link']}"
            mimic = original.find("mimic")
            if mimic is not None:
                assert copied.find("mimic").attrib == {**mimic.attrib, "joint": f"{side}_{mimic.attrib['joint']}"}
                assert copied.find("mimic").attrib["joint"] in joints


def test_bimanual_asset_preserves_link_geometry_and_shares_meshes(definition: RobotCatalogDefinition) -> None:
    from physicalai_rebot_b601_plugin.studio_catalog import _definitions

    asset = definition.asset
    single = next(d.asset for d in _definitions() if d.type == definition.type.replace("Bimanual_", ""))
    root = asset.root_resolver()
    robot = parse(root / asset.urdf_relative_path).getroot()
    arm = parse(root / single.urdf_relative_path).getroot()
    material_names = [material.attrib["name"] for material in robot.findall("material")]
    assert len(material_names) == len(set(material_names)) == len(arm.findall("material"))
    assert {m.attrib["name"] for m in robot.findall(".//visual/material")} <= set(material_names)
    package_name = next(iter(asset.packages))
    for side in ("left", "right"):
        for original in arm.findall("link"):
            copied = robot.find(f"link[@name='{side}_{original.attrib['name']}']")
            assert copied is not None
            inertial = original.find("inertial")
            if inertial is not None:
                assert ET.canonicalize(
                    ET.tostring(copied.find("inertial"), encoding="unicode"), strip_text=True
                ) == ET.canonicalize(ET.tostring(inertial, encoding="unicode"), strip_text=True)
            for tag in ("visual", "collision"):
                assert len(copied.findall(tag)) == len(original.findall(tag))
                for before, after in zip(original.findall(tag), copied.findall(tag), strict=True):
                    assert before.find("origin").attrib == after.find("origin").attrib
                    before_mesh = before.find("geometry/mesh").attrib
                    after_mesh = after.find("geometry/mesh").attrib
                    assert after_mesh == {
                        **before_mesh,
                        "filename": f"package://{package_name}/{before_mesh['filename']}",
                    }
    for mesh in robot.findall(".//mesh"):
        package, relative = mesh.attrib["filename"].removeprefix("package://").split("/", 1)
        assert (root / asset.packages[package] / relative).is_file()
