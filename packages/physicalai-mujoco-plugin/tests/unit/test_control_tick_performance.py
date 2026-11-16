# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

"""Asset-free trajectory and dispatch regressions for native substep batching."""

from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import Mock

import mujoco
import numpy as np
import pytest

from physicalai_mujoco_plugin.channels import ArmChannels
from physicalai_mujoco_plugin.profiles import RobotProfile
from physicalai_mujoco_plugin.profiles.derive import derive_profile
from physicalai_mujoco_plugin.robot import MuJoCoRobot


def _robot(kind: str, mode: str = "pd", integrator: str = "Euler") -> MuJoCoRobot:
    model = mujoco.MjModel.from_xml_string(
        f'<mujoco><compiler angle="radian"/><option timestep="0.002" integrator="{integrator}"/>'
        '<worldbody><geom type="plane" size="1 1 0.1"/>'
        '<body pos="0 0 0.03"><joint name="j" axis="0 1 0" range="-1 1"/>'
        '<geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.03" mass="1"/></body>'
        f'</worldbody><actuator><{kind} joint="j" ctrlrange="-20 20"/></actuator>'
        '<sensor><jointpos name="position" joint="j"/><jointvel name="velocity" joint="j"/></sensor></mujoco>'
    )
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    channels = ArmChannels(model, data, derive_profile(model), RobotProfile("test", "Test", "unused"), torque_mode=mode)
    robot = MuJoCoRobot.__new__(MuJoCoRobot)
    robot._sim = SimpleNamespace(model=model, data=data, channels=[channels], bases=())
    robot._substeps = 10
    robot._ease = robot._watcher = robot._replay = robot._viewer = robot._episode_auto_reset = None
    robot._automation = SimpleNamespace(tick=Mock(), take_reset_request=Mock(return_value=False))
    robot._objects = SimpleNamespace(apply_held=Mock(), publish=Mock())
    robot._cameras = SimpleNamespace(publish=Mock())
    robot._drain_commands = robot._record_tick = Mock()
    return robot


@pytest.mark.parametrize("integrator", ["Euler", "RK4", "implicitfast"])
@pytest.mark.parametrize(("kind", "mode"), [("position", "pd"), ("motor", "raw"), ("motor", "pd"), ("velocity", "pd")])
def test_tick_matches_unbatched_trajectory(kind: str, mode: str, integrator: str) -> None:
    """Keep every bit of state, contacts, sensors and time over changing targets."""
    robot = _robot(kind, mode, integrator)
    reference = _robot(kind, mode, integrator)
    sim, expected = robot._sim, reference._sim
    saw_contact = False
    for target in np.sin(np.arange(50)) * 30:
        sim.channels[0].write(np.array([target]))
        expected.channels[0].write(np.array([target]))
        robot._step_and_sync()
        for _ in range(robot._substeps):
            for channels in expected.channels:
                channels.apply_pd()
            mujoco.mj_step(expected.model, expected.data)
        for name in ("qpos", "qvel", "qacc", "act", "ctrl", "sensordata", "qacc_warmstart"):
            np.testing.assert_array_equal(getattr(sim.data, name), getattr(expected.data, name))
        assert sim.data.time == expected.data.time
        assert sim.data.ncon == expected.data.ncon
        saw_contact |= sim.data.ncon > 0
    assert saw_contact


@pytest.mark.parametrize(
    ("kind", "mode", "required"),
    [("position", "pd", False), ("motor", "raw", False), ("motor", "pd", True), ("velocity", "pd", True)],
)
def test_only_software_controllers_require_python_substeps(kind: str, mode: str, required: bool) -> None:
    robot = _robot(kind, mode)
    assert robot._sim.channels[0].requires_step_control is required


def test_batch_dispatch_and_tick_service_frequency(monkeypatch: pytest.MonkeyPatch) -> None:
    robot = _robot("position")
    step = Mock(wraps=mujoco.mj_step)
    monkeypatch.setattr(mujoco, "mj_step", step)
    robot._step_and_sync()
    step.assert_called_once_with(robot._sim.model, robot._sim.data, nstep=10)
    robot._automation.tick.assert_called_once()
    robot._objects.apply_held.assert_called_once()
    robot._objects.publish.assert_called_once()
    robot._cameras.publish.assert_called_once()


@pytest.mark.parametrize("reason", ["easing", "control_callback", "pd", "velocity", "single_step"])
def test_per_step_work_is_never_batched(reason: str, monkeypatch: pytest.MonkeyPatch) -> None:
    robot = _robot({"pd": "motor", "velocity": "velocity"}.get(reason, "position"))
    if reason == "single_step":
        robot._substeps = 1
    if reason == "easing":
        robot._ease = object()
        # Ending the plan mid-tick must not skip the remaining integration steps.
        robot._advance_ease = Mock(side_effect=lambda: setattr(robot, "_ease", None))
    callback = Mock()
    if reason == "control_callback":
        mujoco.set_mjcb_control(callback)
    step = Mock(wraps=mujoco.mj_step)
    monkeypatch.setattr(mujoco, "mj_step", step)
    try:
        robot._step_and_sync()
        assert step.call_count == robot._substeps
        assert all(not call.kwargs for call in step.call_args_list)
        if reason == "easing":
            robot._advance_ease.assert_called_once()
        if reason == "control_callback":
            assert callback.call_count == robot._substeps
    finally:
        if reason == "control_callback":
            mujoco.set_mjcb_control(None)


def test_dispatch_follows_replacement_channels(monkeypatch: pytest.MonkeyPatch) -> None:
    """A scene replacement must not reuse a cached decision from the old actuator kinds."""
    robot = _robot("position")
    robot._step_and_sync()
    robot._sim = _robot("motor")._sim
    step = Mock(wraps=mujoco.mj_step)
    monkeypatch.setattr(mujoco, "mj_step", step)
    robot._step_and_sync()
    assert step.call_count == robot._substeps


def test_second_arm_controller_prevents_batching(monkeypatch: pytest.MonkeyPatch) -> None:
    robot = _robot("position")
    controller = SimpleNamespace(requires_step_control=True, apply_pd=Mock(), model_targets=lambda: np.zeros(1))
    robot._sim.channels.append(controller)
    step = Mock(wraps=mujoco.mj_step)
    monkeypatch.setattr(mujoco, "mj_step", step)
    robot._step_and_sync()
    assert step.call_count == robot._substeps
    assert controller.apply_pd.call_count == robot._substeps


def test_eased_trajectory_matches_per_step_reference() -> None:
    robot, reference = _robot("position"), _robot("position")
    for driver in (robot, reference):
        targets = driver._arm_targets()
        start = driver._targets_to_units(targets)
        driver._ease = (start, start + 15.0, 0.0, 0.03, driver._sim, targets)
    for _ in range(5):
        robot._step_and_sync()
        for _ in range(reference._substeps):
            if reference._ease is not None:
                reference._advance_ease()
            mujoco.mj_step(reference._sim.model, reference._sim.data)
        for name in ("qpos", "qvel", "ctrl"):
            np.testing.assert_array_equal(getattr(robot._sim.data, name), getattr(reference._sim.data, name))
