# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import importlib
import subprocess
import sys
import textwrap

import pytest


def _run(code: str) -> None:
    result = subprocess.run([sys.executable, "-c", textwrap.dedent(code)], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr


def test_lightweight_exports_do_not_import_validation_or_robot_dependencies() -> None:
    _run("""
        import importlib.abc
        import sys

        class BlockDependencies(importlib.abc.MetaPathFinder):
            def find_spec(self, fullname, path=None, target=None):
                if fullname.split('.')[0] in {'pydantic', 'numpy', 'torch', 'loguru', 'physicalai'}:
                    raise AssertionError(f'Unexpected import: {fullname}')

        sys.meta_path.insert(0, BlockDependencies())
        import physicalai_studio_plugin as sdk
        loaded = {name for name in sys.modules if name.startswith('physicalai_studio_plugin.')}
        assert loaded == {'physicalai_studio_plugin._exports'}
        assert set(sdk.__all__) <= set(dir(sdk))
        from physicalai_studio_plugin import (
            CatalogRobotFactory, RobotAsset, RobotZeroCalibration,
            robot_field_ui, robot_payload_ui, shared_robot_name,
        )
        from uuid import UUID
        assert shared_robot_name(UUID(int=0)) == '00000000-0000-0000-0000-000000000000'
        assert robot_field_ui({'required': True}) == {'x-physicalai-ui': {'required': True}}
        assert robot_payload_ui([]) == {'x-physicalai-ui': []}
    """)


def test_simulation_helpers_do_not_import_robot_runtime() -> None:
    _run("""
        import sys
        from physicalai_studio_plugin import SimulationLaunch, SimulationScene
        assert 'physicalai.robot' not in sys.modules
        assert 'numpy' not in sys.modules
        assert 'torch' not in sys.modules
    """)


def test_all_exports_keep_their_original_identity_and_cache() -> None:
    import physicalai_studio_plugin as sdk

    expected = """
        BuildRobotCallable CatalogRobot CatalogRobotFactory PayloadContainer PortScanner
        RobotAdapterOptions RobotAsset RobotCatalogDefinition RobotCatalogRegistry
        RobotFieldUiOptions RobotPayloadUiOptions RobotProbe RobotUiCalibrationItem
        RobotUiConnectionBinding RobotUiConnectionItem RobotUiFieldItem RobotUiInfoItem
        RobotUiIpAddressItem RobotUiItem RobotUiSectionOptions RobotZeroCalibration SerialPortInfo
        SimulationArgvBuilder SimulationLaunch SimulationPayloadArgvBuilder SimulationScene
        robot_field_ui robot_payload_ui shared_robot_name validate_robot_payload_ui
    """.split()
    assert sorted(sdk.__all__) == sorted(expected)
    assert set(sdk._EXPORT_MODULES) == set(expected)
    for name in expected:
        original = getattr(importlib.import_module(f"physicalai_studio_plugin.{sdk._EXPORT_MODULES[name]}"), name)
        assert getattr(sdk, name) is original
        assert vars(sdk)[name] is original
        assert getattr(sdk, name) is original


def test_star_import_remains_supported() -> None:
    _run("""
        import physicalai_studio_plugin as sdk
        from physicalai_studio_plugin import *
        for name in sdk.__all__:
            assert globals()[name] is getattr(sdk, name)
    """)


def test_unknown_names_do_not_trigger_imports(monkeypatch: pytest.MonkeyPatch) -> None:
    import physicalai_studio_plugin as sdk

    def unexpected_import(*args: object) -> None:
        pytest.fail(f"Unexpected import: {args}")

    monkeypatch.setattr(sdk, "import_module", unexpected_import)
    with pytest.raises(AttributeError, match="has no attribute 'not_an_export'"):
        sdk.__getattr__("not_an_export")


def test_failed_import_is_not_cached(monkeypatch: pytest.MonkeyPatch) -> None:
    import physicalai_studio_plugin as sdk

    monkeypatch.delitem(vars(sdk), "RobotAsset", raising=False)
    original_import = sdk.import_module

    def broken_import(*args: object) -> None:
        raise ImportError("dependency failed")

    monkeypatch.setattr(sdk, "import_module", broken_import)
    with pytest.raises(ImportError, match="dependency failed"):
        _ = sdk.RobotAsset
    assert "RobotAsset" not in vars(sdk)
    monkeypatch.setattr(sdk, "import_module", original_import)
    from physicalai_studio_plugin.assets import RobotAsset

    assert sdk.RobotAsset is RobotAsset
