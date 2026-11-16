# MuJoCo control-tick performance

## Result

Batch physics substeps in MuJoCo when the tick has no Python work between them.
On six asset-free position-actuated pendulums, the alternating comparison reduced
CPU time by **14% at 10 substeps** and **17% at 40 substeps**. This is a CPU
microbenchmark, not a claim about rendered scene throughput or hardware latency.
At a paced 50 Hz, the absolute savings are only about 3/13 µs per tick.

The original loop called `ArmChannels.apply_pd()` and crossed Python → MuJoCo on
every substep, including position/raw actuators whose software controller does
nothing. The new path uses `mj_step(..., nstep=substeps)`, which repeats the same
integration steps in native code. It does **not** increase the timestep or skip
integration, contacts, sensors or native actuator dynamics.

The Python loop remains active for software torque PD, velocity position tracking,
target easing, registered MuJoCo control callbacks, and single-substep ticks.
Every arm participates in the eligibility check. The decision is recomputed per
tick, so scene replacement cannot leave stale actuator metadata. Automation,
episode checks, object updates, viewer sync and camera publication retain their
once-per-tick order. No locks, snapshots, renderer ownership or observation buffer
ownership changed. A native batch returns to Python after the control tick rather
than between substeps; Python signal handling can therefore wait for that batch.

## Environment and method

- Linux x86-64, kernel `6.8.0-147-generic`; Intel Core i9-12900K, 24 logical CPUs.
- CPython 3.13.1, MuJoCo 3.16.0 release wheel, NumPy 2.5.4.
- Original source: `b71bc333bab5f57fc55e375a6f414fa9a7cfa96d`.
  Baseline harness commit: `35f98e6`; optimized implementation: `727cdf5`.
- Six independent capsule pendulums, 2 ms Euler integration, fixed 10-degree
  targets, position / motor (software PD) / velocity actuators. No physical robot,
  network, display, GL context, model download, LFS asset or background render thread.
- The harness uses the real `_step_and_sync`, `ArmChannels`, `MjModel` and `MjData`.
  It replaces unrelated scene services with no-ops; it excludes construction,
  configuration, observation conversion and rendering. It still includes target
  readback/allocation, native physics and Python dispatch.
- 1,000 warmup ticks per variant; five repeats of 5,000 measured ticks each.
  Per-call wall samples plus per-repeat `process_time_ns` CPU and wall totals.
  Means include sample-collection overhead; percentiles measure just the tick.
- First measured the unchanged implementation before editing production code.
  Then ran paired variants sequentially, reversing order on each repeat.
  The paired reference forces the retained Python loop with a channel facade;
  it also pays the new dispatch check and is **not** an old binary. Its per-step
  controller is the original bound method, not a mocked physics implementation.
- Other package agents ran benchmarks concurrently. Frequency scaling,
  contention and scheduling were not controlled. In particular, motor results
  show strong run-to-run noise; do not infer a PD speedup or slowdown from them.
  The paired position results agree with the separately measured old baseline.
- Final paired execution completed 450,000 measured ticks without exceptions;
  final positions were finite and bit-identical between each pair. The harness
  checks final positions, not every intermediate native warning or state field.

## Measurements

All times are **µs/tick**. CPU columns show mean ± sample standard deviation over
five repeat means (not a confidence interval). Percentiles pool 25,000 calls per
variant/scenario. “Original” was measured before the production change; “loop” is
the contemporaneous forced-loop reference. No performance threshold is asserted
in CI.

| Actuator | Steps | Original CPU mean | After CPU mean ± SD | Paired loop CPU mean ± SD | After wall mean | After wall p50 / p95 / p99 / max |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| position | 1 | 5.324 | 5.293 ± 0.045 | 5.453 ± 0.016 | 5.293 | 5.165 / 5.400 / 6.289 / 27.689 |
| position | 10 | 21.784 | 19.228 ± 0.243 | 22.344 ± 0.334 | 19.228 | 19.063 / 19.652 / 20.761 / 42.219 |
| position | 40 | 75.155 | 63.081 ± 0.372 | 76.112 ± 0.518 | 63.090 | 63.132 / 65.080 / 66.321 / 168.383 |
| motor PD | 1 | 8.640 | 8.566 ± 0.037 | 8.733 ± 0.076 | 8.567 | 8.417 / 8.819 / 9.779 / 26.001 |
| motor PD | 10 | 58.828 | 61.069 ± 4.989 | 75.837 ± 16.287 | 61.253 | 58.572 / 79.432 / 113.864 / 2943.859 |
| motor PD | 40 | 223.858 | 242.371 ± 13.845 | 224.748 ± 2.185 | 242.387 | 224.360 / 394.256 / 398.649 / 489.913 |
| velocity | 1 | 8.355 | 8.498 ± 0.083 | 8.582 ± 0.099 | 8.498 | 8.357 / 8.723 / 9.740 / 31.517 |
| velocity | 10 | 58.642 | 57.051 ± 0.560 | 57.481 ± 0.509 | 57.053 | 56.900 / 59.109 / 60.643 / 103.522 |
| velocity | 40 | 235.938 | 218.325 ± 5.374 | 220.338 ± 4.477 | 218.340 | 217.442 / 225.582 / 235.922 / 1410.015 |

For position actuators the paired-loop wall p50/p95/p99/max values were:

| Steps | Loop wall p50 / p95 / p99 / max |
| ---: | --- |
| 1 | 5.321 / 5.528 / 6.461 / 28.791 |
| 10 | 22.174 / 22.954 / 24.440 / 44.759 |
| 40 | 75.969 / 78.434 / 80.107 / 212.600 |

## Reproduce without changing the shared environment

From the worktree root, reuse the existing Runtime environment read-only and add
MuJoCo plus small test dependencies in a separate environment. Adjust `SHARED`
when using another checkout. `PYTHONPATH` prioritizes this worktree's sources and
then the shared dependency directory; this run therefore used shared NumPy 2.5.4
rather than the isolated environment's automatically resolved NumPy 2.5.5.

```bash
SHARED=/home/mark/projects/geti/physicalai/.venv
BENCH_ENV=/tmp/opencode/package-performance/mujoco-env
PKG=packages/physicalai-mujoco-plugin
uv venv --python "$SHARED/bin/python" --system-site-packages "$BENCH_ENV"
uv pip install --python "$BENCH_ENV/bin/python" 'mujoco==3.16.0' \
  'mujoco-menagerie==2026.9.4' defusedxml uvicorn fastapi httpx
export PYTHONPATH="$PWD/$PKG/src:$PWD/src:$SHARED/lib/python3.13/site-packages"

uv run --no-project "$BENCH_ENV/bin/python" \
  "$PKG/scripts/benchmark_control_tick.py" --calls 5000 --repeats 5 --paired
```

For the original baseline, run the non-paired command at `35f98e6` (before the
production optimization); its harness has no `--paired` option. On a normally
provisioned checkout, `uv run --no-project .venv/bin/python` can replace the
isolated interpreter, with the worktree package and Runtime `src` on `PYTHONPATH`.
Do not sync or install into an environment shared with other worktrees.

## Validation

```bash
uv run --no-project "$BENCH_ENV/bin/python" -m pytest \
  "$PKG/tests/unit/test_control_tick_performance.py" \
  "$PKG/tests/test_profiles.py" "$PKG/tests/test_robot.py" \
  "$PKG/tests/test_services.py" \
  -m 'not requires_download' -k 'not TestViewer' -q

uv tool run --from ruff==0.15.12 ruff check "$PKG/src" \
  "$PKG/scripts/benchmark_control_tick.py"
uv tool run --from ruff==0.15.12 ruff format --check \
  "$PKG/src/physicalai_mujoco_plugin/robot.py" \
  "$PKG/src/physicalai_mujoco_plugin/channels.py" \
  "$PKG/scripts/benchmark_control_tick.py" \
  "$PKG/tests/unit/test_control_tick_performance.py"
uv run --no-project "$SHARED/bin/pyrefly" check \
  "$PKG/src/physicalai_mujoco_plugin/robot.py" \
  "$PKG/src/physicalai_mujoco_plugin/channels.py"
```

- **238 passed, 22 deselected, 2 warnings in 3.54 s.** Warnings: FastAPI/Starlette
  test-client deprecation and an existing intentional overflow-orientation case.
  The new asset-free regression file contributes **25 passing tests**.
- Regression coverage includes bit-exact trajectories across Euler/RK4/implicitfast,
  position/raw torque/software PD/velocity control, changing targets, contacts,
  sensors, warm-start state, easing, callbacks, a second arm's controller, scene
  replacement, native-call counts and once-per-tick service calls.
- Pinned Ruff: package source and benchmark lint passed; four modified Python
  files passed formatting. Shared Ruff 0.17.5 reports preexisting style findings;
  use the repository-pinned 0.15.12 instead of migrating unrelated suppressions.
- Focused Pyrefly: **0 errors (68 suppressed, 1 warning not shown)**. Existing
  MuJoCo C-extension attribute suppressions remain in effect.
- A broader robot/services attempt produced **145 passed, 16 failed**: all 16
  failures required absent `viser`/`mjviser` dependencies. The final focused run
  excludes viewer classes. Earlier missing lightweight dependencies were installed
  only in the isolated environment. The full package suite, rendered scenes,
  download-marked tests, profile smoke matrix and browser viewer were not validated.
  Real scene tests need `git lfs pull` plus Menagerie assets (downloaded on first
  use); a headless display-free physics test still needs those model/mesh files.
  No new test or benchmark requires them.

## Assessment and limits

- **Physics:** removing no-op controller calls and native crossings gives bounded
  CPU savings. Complex contacts, solver cost and many-body scenes can dominate
  the remaining time. PD/velocity cases intentionally retain their old algorithm.
- **Lookups/conversions:** channel bindings already resolve actuator/joint IDs at
  load. Scalar unit conversions, clipping and target readback still allocate/read
  arrays per tick. Blindly caching model limit arrays would hide live model edits;
  vectorization needs rounding, signed-zero, grouped-member and transmission parity
  tests before adoption. Observation arrays remain caller-owned; no shared scratch
  buffers were introduced.
- **Rendering:** the service already owns persistent renderers on the camera
  thread and gates frames by FPS. Do not cache the public one-shot `render_camera`
  renderer across arbitrary callers/threads: OpenGL context ownership and changing
  sizes/scenes make that unsafe without lifecycle work. The camera thread currently
  recomputes positional kinematics for newly published poses even when no camera
  is due. Deferring this until a due frame is promising future work, but needs
  scheduling, failed-camera and scene-switch tests plus render-inclusive profiling.
- **Concurrency:** the simulation thread still exclusively writes physics state;
  viewer/HTTP controls still queue commands, and cameras still consume snapshots.
  This optimization adds no sleeps, threads, caches or synchronization changes.
- **Next measurements:** repeat on a quiet pinned CPU, alternate actual old/new
  builds, use longer fixed windows and confidence intervals, then profile real
  single/bimanual scenes with cameras and the viewer separately. Measure RSS and
  allocation counts before attempting buffer reuse. Do not extrapolate these
  synthetic CPU results into rendered FPS or end-to-end policy throughput.
