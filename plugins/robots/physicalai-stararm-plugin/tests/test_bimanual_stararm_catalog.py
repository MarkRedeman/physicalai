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
    assert definition.asset is not None
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


def test_bimanual_asset_tree_and_joint_mapping(definition: RobotCatalogDefinition) -> None:
    from physicalai_stararm_plugin.studio_catalog import _definitions

    asset = definition.asset
    single = next(d.asset for d in _definitions() if d.type == definition.type.replace("Bimanual_", ""))
    assert asset is not None and single is not None
    assert asset.root_resolver is not None and single.root_resolver is not None
    root = asset.root_resolver()
    variant = next(v for v in ("ld", "hd", "fl") if f"_{v.upper()}_" in definition.type)
    assert asset.urdf_relative_path == Path(f"stararm102/urdf/stararm102_bimanual_{variant}_description.urdf")
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
        assert asset.joint_map[f"{side}_gripper.pos"] == [f"{side}_joint7_left", f"{side}_joint7_right"]
        for original in arm.findall("joint"):
            copied = joints[f"{side}_{original.attrib['name']}"]
            assert copied.attrib["type"] == original.attrib["type"]
            for tag in ("origin", "axis", "limit"):
                assert copied.find(tag).attrib == original.find(tag).attrib
            for tag in ("parent", "child"):
                assert copied.find(tag).attrib["link"] == f"{side}_{original.find(tag).attrib['link']}"


def test_bimanual_asset_preserves_link_geometry_and_shares_meshes(definition: RobotCatalogDefinition) -> None:
    from physicalai_stararm_plugin.studio_catalog import _definitions

    asset = definition.asset
    single = next(d.asset for d in _definitions() if d.type == definition.type.replace("Bimanual_", ""))
    root = asset.root_resolver()
    robot = parse(root / asset.urdf_relative_path).getroot()
    arm = parse(root / single.urdf_relative_path).getroot()
    material_names = [material.attrib["name"] for material in robot.findall("material")]
    assert len(material_names) == len(set(material_names)) == len(arm.findall("material"))
    assert {m.attrib["name"] for m in robot.findall(".//visual/material")} <= set(material_names)
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
                    assert before.find("geometry/mesh").attrib == after.find("geometry/mesh").attrib
    for mesh in robot.findall(".//mesh"):
        package, relative = mesh.attrib["filename"].removeprefix("package://").split("/", 1)
        assert (root / asset.packages[package] / relative).is_file()
