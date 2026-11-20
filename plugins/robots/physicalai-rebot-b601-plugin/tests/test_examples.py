from __future__ import annotations

from pathlib import Path

import pytest
import yaml

_EXAMPLES = Path(__file__).resolve().parents[1] / "examples" / "runtime"


@pytest.mark.parametrize("path", sorted(_EXAMPLES.glob("teleop-*.yaml")), ids=lambda p: p.name)
def test_star_arm_leaders_emit_the_b601_frame(path: Path) -> None:
    # The B601 followers take LeRobot-style joints; a Star Arm leader only emits them with the b601 profile.
    runtime = yaml.safe_load(path.read_text())["runtime"]
    leader = runtime["action_source"]["init_args"]["leader"]

    assert leader["class_path"].startswith("physicalai_stararm_plugin.")
    assert leader["init_args"]["follower_profile"] == "b601"


@pytest.mark.parametrize("path", sorted(_EXAMPLES.glob("bimanual-*.yaml")), ids=lambda p: p.name)
def test_bimanual_teleop_recipes_instantiate_without_hardware(path: Path) -> None:
    from physicalai.cli.run import build_parser
    from physicalai.config import Config
    from physicalai_rebot_b601_plugin import BimanualReBotB601
    from physicalai_stararm_plugin import BimanualStarArm102

    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    leader_args = document["runtime"]["action_source"]["init_args"]["leader"]["init_args"]
    assert all(leader_args[side]["init_args"]["follower_profile"] == "b601" for side in ("left", "right"))
    parser = build_parser()
    runtime = parser.instantiate(parser.parse_object(document)).runtime
    robot = runtime.robot
    assert isinstance(robot, BimanualReBotB601)
    assert len(robot.joint_names) == 14
    assert not robot.is_connected()
    leader_recipe = Config.from_dict(document["runtime"]["action_source"]["init_args"]["leader"])
    leader = leader_recipe.instantiate()
    assert isinstance(leader, BimanualStarArm102)
    assert robot.joint_names == leader.joint_names
    assert not leader.is_connected()


@pytest.mark.parametrize(("value", "expected"), [(False, 0.0), (True, 1.0), (0.3, 0.3)])
def test_feedforward_settings_accept_booleans_and_shares(value: bool | float, expected: float) -> None:
    from jsonargparse import ArgumentParser

    from physicalai_rebot_b601_plugin.rs import ReBotB601RS

    parser = ArgumentParser()
    parser.add_class_arguments(ReBotB601RS, "robot")
    cfg = parser.parse_object({"robot": {"gravity_compensation": value, "velocity_feedforward": value}})

    assert cfg.robot.gravity_compensation == expected
    assert cfg.robot.velocity_feedforward == expected
