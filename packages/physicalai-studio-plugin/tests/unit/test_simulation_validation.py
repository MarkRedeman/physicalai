# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

from dataclasses import replace
from unittest.mock import Mock

import pytest
from physicalai_studio_plugin import SimulationLaunch, SimulationScene


def _launch(ids: tuple[str, ...], default: str = "z") -> SimulationLaunch:
    return SimulationLaunch(
        scenes=tuple(SimulationScene(id=name, display_name=name) for name in ids),
        default_scene=default,
        payload_owner_name=lambda _: "test-owner",
        build_argv=lambda **_: ["simulation", "start"],
    )


def test_duplicates_are_sorted_and_reported_once() -> None:
    with pytest.raises(ValueError) as raised:
        _launch(("z", "a", "z", "a", "a"))
    assert str(raised.value) == "scene ids must be unique, got duplicates ['a', 'z']"


def test_default_scene_error_preserves_catalog_order() -> None:
    with pytest.raises(ValueError) as raised:
        _launch(("z", "a", "b"), default="missing")
    assert str(raised.value) == "default_scene 'missing' is not one of the scenes ['z', 'a', 'b']"


@pytest.mark.parametrize("method", ["argv", "standalone_argv"])
def test_unknown_scene_is_rejected_before_callbacks(method: str) -> None:
    callback = Mock(side_effect=AssertionError("must validate before calling plugin code"))
    launch = replace(
        _launch(("z", "a", "b")),
        payload_owner_name=callback,
        build_argv=callback,
        build_standalone_argv=callback,
    )
    with pytest.raises(ValueError) as raised:
        getattr(launch, method)("missing", None, seed=-1)
    assert str(raised.value) == "Unknown scene 'missing'; choose one of ['z', 'a', 'b']"
    callback.assert_not_called()


@pytest.mark.parametrize("index", [0, 128, 255])
def test_large_catalog_matches_first_middle_and_last_scene(index: int) -> None:
    launch = _launch(tuple(f"scene-{i}" for i in range(256)), default="scene-0")
    assert launch.argv(f"scene-{index}", None) == ["simulation", "start"]


def test_replaced_catalog_is_validated_without_stale_scene_ids() -> None:
    launch = _launch(("z", "a"))
    updated = replace(launch, scenes=(SimulationScene(id="new", display_name="New"),), default_scene="new")
    assert launch.argv("a", None) == ["simulation", "start"]
    assert updated.argv("new", None) == ["simulation", "start"]
    with pytest.raises(ValueError, match="Unknown scene 'a'"):
        updated.argv("a", None)


def test_each_request_returns_a_fresh_argv_list() -> None:
    shared = ["simulation", "start"]
    launch = replace(_launch(("z",)), build_argv=lambda **_: shared)
    first = launch.argv("z", None)
    first.append("--modified")
    assert launch.argv("z", None) == ["simulation", "start"]
    assert shared == ["simulation", "start"]
