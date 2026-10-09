# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

"""Tests for the Trossen Studio catalog registration and bundled URDF assets."""

from __future__ import annotations

from pathlib import Path

from physicalai.config import Config
from physicalai.robot import Robot


class _FakeRegistry:
    def __init__(self) -> None:
        self.definitions: list[object] = []

    def register_robot(self, definition: object) -> None:
        self.definitions.append(definition)


def test_registers_all_trossen_catalog_definitions() -> None:
    from physicalai_trossen_plugin.studio_catalog import register_physicalai_studio_plugin

    registry = _FakeRegistry()
    register_physicalai_studio_plugin(registry)

    assert {definition.type for definition in registry.definitions} == {
        "Trossen_WidowXAI_Follower",
        "Trossen_WidowXAI_Leader",
        "Trossen_Bimanual_WidowXAI_Follower",
        "Trossen_Bimanual_WidowXAI_Leader",
    }


def test_catalog_assets_resolve_to_bundled_urdfs() -> None:
    from physicalai_trossen_plugin.studio_catalog import _definitions

    expected_paths = {
        "Trossen_WidowXAI_Follower": Path("urdf/generated/wxai/wxai_follower.urdf"),
        "Trossen_Bimanual_WidowXAI_Follower": Path("urdf/generated/stationary_ai.urdf"),
    }
    definitions = {definition.type: definition for definition in _definitions()}

    for robot_type, expected_path in expected_paths.items():
        asset = definitions[robot_type].asset
        assert asset is not None
        assert asset.urdf_relative_path == expected_path
        assert asset.root_resolver is not None
        assert (asset.root_resolver() / asset.urdf_relative_path).is_file()


def test_drivers_export_plugin_construction_recipes() -> None:
    from physicalai_trossen_plugin import BimanualWidowXAI, WidowXAI

    left = WidowXAI(ip="192.168.1.10")
    right = WidowXAI(ip="192.168.1.11")
    bimanual = BimanualWidowXAI(left=left, right=right)

    assert isinstance(left, Robot)
    assert Config.from_instance(left).to_dict() == {
        "class_path": "physicalai_trossen_plugin.WidowXAI",
        "init_args": {"ip": "192.168.1.10"},
    }
    assert Config.from_instance(bimanual).to_dict()["class_path"] == "physicalai_trossen_plugin.BimanualWidowXAI"
