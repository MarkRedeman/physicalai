# Kinematics Examples

Examples for wrapping an existing joint-space robot as an end-effector pose robot.

Install the kinematics extra plus the hardware driver extra you need:

```bash
pip install physicalai[kinematics,so101]
pip install physicalai[kinematics,trossen]
```

## end_effector_pose

Read the current end-effector pose as `x, y, z, qx, qy, qz, qw` using a URDF.

SO-101 example:

```bash
python examples/kinematics/end_effector_pose.py \
  --robot so101 \
  --port /dev/ttyUSB0 \
  --calibration ./calibration.json \
  --urdf ./so101.urdf \
  --end-effector-frame gripper_link \
  --controlled-joint shoulder_pan \
  --controlled-joint shoulder_lift \
  --controlled-joint elbow_flex \
  --controlled-joint wrist_flex \
  --controlled-joint wrist_roll
```

WidowXAI example:

```bash
python examples/kinematics/end_effector_pose.py \
  --robot widowxai \
  --ip 192.168.1.2 \
  --urdf ./widowxai.urdf \
  --end-effector-frame ee_link \
  --controlled-joint shoulder_pan \
  --controlled-joint shoulder_lift \
  --controlled-joint elbow_flex \
  --controlled-joint wrist_flex \
  --controlled-joint wrist_yaw \
  --controlled-joint wrist_roll
```

By default the script is read-only and only prints poses. To test Cartesian control, pass a small jog plus `--execute`:

```bash
python examples/kinematics/end_effector_pose.py ... --jog-x 0.01 --execute
```

`--jog-x 0.01` means: read the current pose, add 1 cm to `x`, keep the orientation unchanged, solve IK, and send the resulting joint command to the wrapped robot.

The example rejects jogs larger than `--max-jog` meters, defaulting to 0.05 m. The adapter also applies safety guardrails before sending IK results:

- maximum Cartesian target delta: 0.05 m by default
- maximum orientation target delta: 0.25 rad by default
- maximum solved controlled-joint delta: 10 degrees by default

Notes:

- The URDF joint names must match the wrapped robot's `joint_names` for all controlled joints.
- `SO101(unit="normalized")` is treated as degrees by this adapter for now.
- Non-controlled joints, such as gripper, are preserved from the current internal observation when sending a Cartesian action.
