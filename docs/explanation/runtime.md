# Runtime

`RobotRuntime` runs the control loop on robot hardware. It owns hardware I/O, the callback lifecycle, and timing — while a required, pluggable `action_source` owns the actual decision of what action to send each tick. `PolicySource` wraps a trained policy (model + execution strategy + action queue); `TeleopSource` reads a leader arm; `PolicyTeleopSource` provides a guarded keyboard handoff between both; custom logic can implement the `ActionSource` protocol directly.

```python
runtime = RobotRuntime(
    fps=30,
    robot=robot,
    action_source=PolicySource(model=model, execution=SyncExecution()),
)

with runtime:
    runtime.run(duration_s=60)
```

## Responsibilities

| Component        | Owns                                                             | Does not own                |
| ---------------- | ---------------------------------------------------------------- | --------------------------- |
| `InferenceModel` | model load, preprocess, inference, postprocess                   | robot loop timing           |
| `Execution`      | where and when inference runs                                    | robot IO                    |
| `ActionQueue`    | action chunks and buffering                                      | model inference             |
| `PolicySource`   | wiring model + execution + action queue into one action per tick | robot IO, loop timing       |
| `RobotRuntime`   | observe, call the action source, send action, callbacks, timing  | policy math, decision logic |
| `Robot`          | hardware connection, observations, actions                       | policy inference            |

## Loop

The runtime loop follows this general pattern:

```text
while running:
    robot_state, camera_frames = read_observation()
    action = action_source.update(robot_state, camera_frames, step)
    action = on_action_ready(action)  # callback hook, may transform
    send_action_to_robot(action)
    on_action_sent(action)            # callback hook, notification only
    sleep_until_next_tick()
```

The exact observation structure and merging strategy may change as the API stabilizes. Everything left of `action_source.update()` — deciding whether to run inference, pulling from the queue, holding the last action — is internal to the action source; `RobotRuntime` itself only ever sees one action per tick.

## Execution Modes

> **Preview:** `RemoteExecution` is a planned API.

| Mode               | Where inference runs | Use                              |
| ------------------ | -------------------- | -------------------------------- |
| `SyncExecution()`  | runtime thread       | simple deployments and debugging |
| `AsyncExecution()` | worker thread        | avoid blocking the control loop  |
| `RemoteExecution`  | remote server        | planned API                      |

## Product Workflows

`PolicyTeleopSource` supports DAgger-style interventions without stopping model
execution. Press `t` to arm teleop; the follower immediately holds position
while the leader is aligned and remains held through the configured stable
period. A second `t` engages teleop; press `t` again to return to policy. The
source exposes its `mode` and emits it as the `action_mode` metric
(`policy=0`, `arming=1`, `hold=2`, `teleop=3`) so a future recorder can label
intervention segments.
Audio cues are enabled by default and announce when teleop is armed, ready,
active, and when policy control resumes. Set `audio_cues=False` for systems
without `espeak` and `aplay`.

For an actuated, same-morphology leader, set `leader_follows_follower=True` to
keep it aligned with the follower while the policy runs. Tracking stops as soon
as teleop is armed and leader torque is disabled so the operator can take the
leader. The source restores torque when policy resumes. For SO-101 tracking,
configure the leader as `role: follower`, not the passive torque-disabled
`role: leader`.

```python
class HILCallback:
    def on_action_ready(self, *, action, step):
        if teleop.enabled:
            return teleop.read_action()
        return action
```
