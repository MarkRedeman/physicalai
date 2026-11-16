# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

"""Public API for the Physical AI Studio plugin.

Exports load on first access so metadata and CLI helpers do not load the robot
runtime or model-validation dependencies they do not use.
"""

from __future__ import annotations

from importlib import import_module
from typing import TYPE_CHECKING

from ._exports import EXPORT_MODULES as _EXPORT_MODULES

if TYPE_CHECKING:
    from .assets import RobotAsset
    from .catalog import (
        BuildRobotCallable,
        CatalogRobot,
        PayloadContainer,
        RobotAdapterOptions,
        RobotCatalogDefinition,
        RobotCatalogRegistry,
    )
    from .factory import CatalogRobotFactory
    from .probe import PortScanner, RobotProbe
    from .schemas import SerialPortInfo
    from .simulation import SimulationArgvBuilder, SimulationLaunch, SimulationPayloadArgvBuilder, SimulationScene
    from .transport import shared_robot_name
    from .ui_schema import (
        RobotFieldUiOptions,
        RobotPayloadUiOptions,
        RobotUiCalibrationItem,
        RobotUiConnectionBinding,
        RobotUiConnectionItem,
        RobotUiFieldItem,
        RobotUiInfoItem,
        RobotUiIpAddressItem,
        RobotUiItem,
        RobotUiSectionOptions,
        robot_field_ui,
        robot_payload_ui,
        validate_robot_payload_ui,
    )
    from .zero_calibration import RobotZeroCalibration

__all__ = [
    "BuildRobotCallable",
    "CatalogRobot",
    "CatalogRobotFactory",
    "PayloadContainer",
    "PortScanner",
    "RobotAdapterOptions",
    "RobotAsset",
    "RobotCatalogDefinition",
    "RobotCatalogRegistry",
    "RobotFieldUiOptions",
    "RobotPayloadUiOptions",
    "RobotProbe",
    "RobotUiCalibrationItem",
    "RobotUiConnectionBinding",
    "RobotUiConnectionItem",
    "RobotUiFieldItem",
    "RobotUiInfoItem",
    "RobotUiIpAddressItem",
    "RobotUiItem",
    "RobotUiSectionOptions",
    "RobotZeroCalibration",
    "SerialPortInfo",
    "SimulationArgvBuilder",
    "SimulationLaunch",
    "SimulationPayloadArgvBuilder",
    "SimulationScene",
    "robot_field_ui",
    "robot_payload_ui",
    "shared_robot_name",
    "validate_robot_payload_ui",
]


def __getattr__(name: str) -> object:
    """Load and cache a public export on first access.

    Returns:
        The original object from its defining module.

    Raises:
        AttributeError: If the name is not a public export.
    """
    module_name = _EXPORT_MODULES.get(name)
    if module_name is None:
        msg = f"module {__name__!r} has no attribute {name!r}"
        raise AttributeError(msg)
    value = getattr(import_module(f".{module_name}", __name__), name)
    globals()[name] = value
    return value


def __dir__() -> list[str]:
    """Include unloaded public exports in module introspection.

    Returns:
        Sorted module attributes and public export names.
    """
    return sorted(set(globals()) | set(__all__))
