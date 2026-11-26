# PhysicalAI OpenArm Plugin

Direct control of [OpenArm](https://github.com/enactic/openarm) arms in
[PhysicalAI](https://github.com/openvinotoolkit/physicalai) over Damiao motors on
CAN. It provides:

| Class                     | Role                                                        |
| ------------------------- | ----------------------------------------------------------- |
| `OpenArmFollower`         | One follower arm, position-controlled with safety limits    |
| `OpenArmLeader`           | One hand-guided leader arm, read-only                       |
| `BimanualOpenArmFollower` | Left and right `OpenArmFollower` as one 16-joint robot      |
| `BimanualOpenArmLeader`   | KER exoskeleton leader, retargeted to the bimanual follower |

It also registers single-arm and bimanual OpenArm entries in PhysicalAI Studio.

## Safety

OpenArm is powerful hardware. Secure each arm, keep clear of its reach, keep an
emergency stop within reach, and follow OpenArm's official safety guide.

- Only one process may use a CAN interface at a time. Do not run this plugin
  alongside LeRobot, ROS 2 or another OpenArm controller on the same bus.
- The plugin does not set motor IDs, write motor parameters, calibrate or zero
  encoders. Do that with OpenArm's own tools before the first run.

## Installation

```bash
pip install "physicalai[plugin-openarm]"
```

Linux only. The plugin defaults to OpenArm's CAN-FD configuration: 1 Mbps
arbitration and 5 Mbps data rate.

## CAN setup

Bring up the interfaces (`can0` to `can3`) in CAN-FD mode with OpenArm's script:

```bash
openarm-can-configure-socketcan-4-arms -fd
```

Without `-fd` the interfaces come up in classic CAN 2.0 and the motors appear
dead. This setting is not persistent: run the script again after every reboot
and every time the arms or USB-CAN adapters are reconnected to the computer.

Also enable automatic bus-off recovery with a 100 ms restart. Without it, one
bus error leaves the interface bus-off until you reconfigure it:

```bash
for can in can0 can1 can2 can3; do
  sudo ip link set "$can" down
  sudo ip link set "$can" type can restart-ms 100
  sudo ip link set "$can" up
done
```

The plugin allows 0.5 s of silence for this recovery before it reports an arm as
disconnected. If disconnects keep triggering, the bus keeps going bus-off
(check `ip -details -statistics link show can0`); look for a loose cable or
missing termination.

### CAN adapters

Set `port` to a SocketCAN interface such as `can0`. USB-CAN adapters work when
the host exposes them as SocketCAN, including `slcand` adapters (which appear as
`slcan0`).

## Joints

Positions are in **degrees**, in this order:

```text
shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_yaw, wrist_pitch, wrist_roll, gripper
```

The bimanual classes use the left arm's eight joints followed by the right arm's,
prefixed `left_` and `right_`.

Followers need `side: left` or `side: right` and clip targets to the joint
limits in `constants.py`; the two sides differ only in gripper direction. Set
`max_relative_target` (degrees) to cap how far a single step may move. Leaders
are read-only; force feedback is not implemented.

## Run

The example configs in `examples/runtime/` use `CHANGE_ME` placeholders for
ports. Edit them, then run from the repo root:

```bash
# One leader arm driving one follower arm
uv run physicalai run --config plugins/robots/physicalai-openarm-plugin/examples/runtime/teleop.yaml

# Two follower arms driven by a KER exoskeleton
uv run physicalai run --config plugins/robots/physicalai-openarm-plugin/examples/runtime/bimanual-teleop.yaml
```

Add `--print_config` to check a config without hardware.

## Studio assets

The bundled parallel-gripper URDFs are plugin-authored kinematic models for
Studio visualization, not OpenArm's CAD or mesh assets. OpenArm software is
Apache-2.0; its hardware design data is licensed separately under
CERN-OHL-S-2.0.

## Development

```bash
uv sync
uv run pytest plugins/robots/physicalai-openarm-plugin/tests/
```
