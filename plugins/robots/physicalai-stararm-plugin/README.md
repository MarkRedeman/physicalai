# PhysicalAI Star Arm Plugin

Fashion Star Arm 102 plugin for [PhysicalAI](https://github.com/openvinotoolkit/physicalai). This package provides Star Arm 102-LD/102-HD leaders and a Star Arm 102-FL follower, and registers all variants with [Physical AI Studio](https://github.com/open-edge-platform/physical-ai-studio).

[![PyPI version](https://img.shields.io/pypi/v/physicalai-stararm-plugin.svg)](https://pypi.org/project/physicalai-stararm-plugin/)
[![Python versions](https://img.shields.io/pypi/pyversions/physicalai-stararm-plugin.svg)](https://pypi.org/project/physicalai-stararm-plugin/)

## Features

- Star Arm 102-LD leader driver (`StarArm102LDLeader`) for read-only teleoperation input
- Star Arm 102-HD leader driver (`StarArm102HDLeader`) with passive mode (default) and optional assist mode for hold/action commands
- Star Arm 102-FL follower driver (`StarArm102FLFollower`) for position control
- Bimanual LD/HD leader and FL follower pairs (`BimanualStarArm102`)
- Studio catalog plugin entries for LD leader, HD leader, and FL follower
- Bundled `stararm102` URDF package

## Follower profiles

Star Arm leaders default to `follower_profile=None`, which preserves their native
joint frame. Set `follower_profile="b601"` when controlling a reBot B601 to emit
LeRobot-compatible actions: the profile applies directions
`[-1, -1, 1, 1, 1, -1, -6]` and clips to the B601 public limits, including a
`-270..0` degree gripper range.

The native default is suitable when a Star Arm leader controls a Star Arm 102-FL
follower.

## Installation

```bash
uv add physicalai-stararm-plugin
```

## Quick start

```python
from physicalai.robot import connect
from physicalai_stararm_plugin import StarArm102FLFollower

robot = StarArm102FLFollower(port="/dev/ttyUSB1", baudrate=1_000_000)

with connect(robot) as arm:
    obs = arm.get_observation()
    arm.send_action(obs.joint_positions)
```

## Bimanual setup

`BimanualStarArm102` composes two matching LD leaders, HD leaders, or FL
followers using Runtime's shared `BimanualRobot` implementation:

```python
from physicalai_stararm_plugin import BimanualStarArm102, StarArm102HDLeader

leader = BimanualStarArm102(
    left=StarArm102HDLeader(port="/dev/ttyUSB0", follower_profile="b601"),
    right=StarArm102HDLeader(port="/dev/ttyUSB1", follower_profile="b601"),
)
```

Observations and actions contain exactly **14 joint positions in degrees**, left
arm first, then right arm. Joint names use `left_` and `right_` prefixes. Each arm
needs its own UART port. Mixed variants, duplicate ports, mismatched leader
control modes, and mismatched follower profiles are rejected.

Use `follower_profile="b601"` on both leaders for a bimanual ReBot follower;
leave both profiles at `None` for Star Arm FL followers. Passive leaders stay
read-only, and HD leaders configured with `control_mode="assist"` on both arms
accept 14-dimensional actions. Single-arm methods, including HD hold controls,
remain available through `leader.left` and `leader.right`.

Studio registers **Star Arm Bimanual 102-LD Leader**, **Star Arm Bimanual 102-HD
Leader**, and **Star Arm Bimanual 102-FL Follower** with separate nested left/right
settings and bimanual 3D previews. Both leader types offer guided zero calibration
for both arms. Restart Studio after installing the updated plugin.

The bundled bimanual descriptions reuse each variant's existing meshes:

- `stararm102/urdf/stararm102_bimanual_ld_description.urdf`
- `stararm102/urdf/stararm102_bimanual_hd_description.urdf`
- `stararm102/urdf/stararm102_bimanual_fl_description.urdf`

Paths are relative to `get_urdf_path()`. The shared `world` root and fixed
`left_mount` / `right_mount` joints place the bases at y=+0.2 / -0.2 m, matching
the SO-101 bimanual layout. Each arm retains its original orientation, geometry,
and limits, including both gripper jaws. This is a preview layout, not a measured
robot-cell calibration; adjust the mounts to match your installation before
using the model for spatial planning.

Review the four ports, calibrate both pairs, and clear the work area before running:

```bash
uv run --extra plugin-stararm physicalai run --config plugins/robots/physicalai-stararm-plugin/examples/runtime/bimanual-teleop-hd-to-fl.yaml
```

For LD leaders, change both leader `class_path` values to
`physicalai_stararm_plugin.StarArm102LDLeader`. `disable_torque()` unlocks both
arms, and `set_zero_position()` stores both current poses as servo origins.

## Run with the PhysicalAI CLI

```bash
uv run physicalai run --config plugins/robots/physicalai-stararm-plugin/examples/runtime/teleop-hd-to-fl.yaml
uv run physicalai run --config plugins/robots/physicalai-stararm-plugin/examples/runtime/teleop-ld-to-fl.yaml
uv run physicalai run --config plugins/robots/physicalai-stararm-plugin/examples/runtime/read-joints-hd.yaml
uv run physicalai run --config plugins/robots/physicalai-stararm-plugin/examples/runtime/read-joints-ld.yaml
uv run physicalai run --config plugins/robots/physicalai-stararm-plugin/examples/runtime/read-joints-fl.yaml
uv run physicalai run --config plugins/robots/physicalai-stararm-plugin/examples/runtime/move-joints-fl.yaml
uv run physicalai run --config plugins/robots/physicalai-stararm-plugin/examples/runtime/hold-hd.yaml
uv run physicalai run --config plugins/robots/physicalai-stararm-plugin/examples/runtime/policy-follow-hd.yaml
```

## Zero-pose calibration

`StarArm102HDLeader.set_zero_position()` (also on the LD leader) stores the arm's current pose as
every servo's origin. Physical AI Studio runs it from its calibration step when you add a Star Arm
leader: move the leader into the same rest pose as the follower with the gripper closed, then set
zero.

## URDF

```python
from physicalai_stararm_plugin import get_urdf_path

urdf_dir = get_urdf_path()
hd_urdf = urdf_dir / "stararm102" / "urdf" / "stararm102_hd_description.urdf"
```

## Development

```bash
uv sync
uv run pytest plugins/robots/physicalai-stararm-plugin/tests/
```

After changing a single-arm URDF, regenerate the bimanual descriptions from the
repository root. The shared generator uses `defusedxml` from the ReBot extra:

```bash
uv run --extra plugin-rebot-b601 python .github/scripts/generate_bimanual_urdfs.py
uv run --extra plugin-rebot-b601 python .github/scripts/generate_bimanual_urdfs.py --check
```
