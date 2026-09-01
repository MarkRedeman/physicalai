from __future__ import annotations

import asyncio
from pathlib import Path

import pytest
from physicalai_studio_plugin import (
    PortScanner,
    RobotAdapterOptions,
    RobotAsset,
    RobotCatalogDefinition,
    RobotProbe,
    SerialPortInfo,
    robot_field_ui,
    robot_payload_ui,
    validate_robot_payload_ui,
)
from pydantic import BaseModel, ConfigDict, Field, ValidationError


class PayloadForTest(BaseModel):
    serial_number: str = Field(...)
    connection_string: str = ""


class TestProbe:
    async def discover(self, manager: PortScanner) -> list[SerialPortInfo]:
        return []

    async def identify(self, payload: PayloadForTest, manager: PortScanner | None, joint: str | None = None) -> None:
        self.last_payload = payload

    async def is_online(self, payload: PayloadForTest, manager: PortScanner | None = None) -> bool:
        return payload.serial_number != ""


def test_probe_is_runtime_checkable() -> None:
    assert isinstance(TestProbe(), RobotProbe)


def test_definition_creation() -> None:
    definition = RobotCatalogDefinition[PayloadForTest](
        type="Test_Follower",
        display_name="Test Follower",
        role="follower",
        robot_payload=PayloadForTest,
        asset=RobotAsset(Path("test/model.urdf"), {"test": Path("test")}, {"gripper.pos": ["gripper"]}),
        adapter_options=RobotAdapterOptions(include_velocities=True),
        probe=TestProbe(),
    )
    assert definition.type == "Test_Follower"
    assert definition.robot_payload is PayloadForTest


def test_valid_payload_passes_validation() -> None:
    definition = RobotCatalogDefinition[PayloadForTest](
        type="Test_Follower", display_name="Test Follower", role="follower", robot_payload=PayloadForTest
    )
    payload_model = definition.robot_payload
    assert payload_model is not None
    assert payload_model.model_validate({"serial_number": "SN-003"}).serial_number == "SN-003"


def test_invalid_payload_raises() -> None:
    with pytest.raises(ValidationError):
        PayloadForTest.model_validate({"connection_string": "/dev/ttyUSB0"})


def test_field_ui() -> None:
    assert robot_field_ui({"required": True}) == {"x-physicalai-ui": {"required": True}}


def test_validate_robot_payload_ui() -> None:
    class ConnectionPayload(BaseModel):
        connection_string: str
        serial_number: str

        model_config = ConfigDict(
            json_schema_extra=robot_payload_ui(
                [{"kind": "connection", "bind": {"connection": "connection_string", "serial_number": "serial_number"}}]
            )
        )

    validate_robot_payload_ui(ConnectionPayload)


@pytest.mark.parametrize(
    ("items", "message"),
    [
        ({"groups": {}}, "must be a list of items"),
        ([{"kind": "field", "name": "missing"}], "must reference an existing payload field"),
    ],
)
def test_validate_robot_payload_ui_rejects_invalid_metadata(items: object, message: str) -> None:
    class InvalidPayload(BaseModel):
        connection_string: str
        model_config = ConfigDict(json_schema_extra={"x-physicalai-ui": items})

    with pytest.raises(ValueError, match=message):
        validate_robot_payload_ui(InvalidPayload)


def test_typed_payload_reaches_identify() -> None:
    probe = TestProbe()
    payload = PayloadForTest(serial_number="SN-001")
    asyncio.run(probe.identify(payload, None))
    assert probe.last_payload is payload
