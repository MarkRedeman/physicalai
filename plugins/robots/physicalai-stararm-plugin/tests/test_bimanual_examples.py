# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

from pathlib import Path

import yaml
from physicalai.cli.run import build_parser
from physicalai.config import Config
from physicalai_stararm_plugin import BimanualStarArm102, StarArm102FLFollower, StarArm102HDLeader


def test_bimanual_teleop_recipe_instantiates_without_hardware() -> None:
    path = Path(__file__).resolve().parents[1] / "examples/runtime/bimanual-teleop-hd-to-fl.yaml"
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    parser = build_parser()
    runtime = parser.instantiate(parser.parse_object(document)).runtime
    assert isinstance(runtime.robot, BimanualStarArm102)
    assert isinstance(runtime.robot.left, StarArm102FLFollower)
    assert not runtime.robot.is_connected()
    leader = Config.from_dict(document["runtime"]["action_source"]["init_args"]["leader"]).instantiate()
    assert isinstance(leader, BimanualStarArm102)
    assert isinstance(leader.left, StarArm102HDLeader)
    assert leader.left.follower_profile is leader.right.follower_profile is None
    assert runtime.robot.joint_names == leader.joint_names
    assert not leader.is_connected()
