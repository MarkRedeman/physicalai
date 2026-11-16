# B601 performance investigation

## Scope and bus budget

No hardware was connected. This investigation follows the SO-101 handoff's
bus-first approach, but does **not** transplant its serial broadcast writes.
B601 uses seven individually addressed CAN motors, not an STS sync-write bus.

Inspection of `dm.py`, `rs.py`, and MotorBridge 0.6.0's Python `core.py`
(the version in the root lockfile) establishes these SDK call budgets:

| Operation | Calls per operation, before optimization |
| --- | --- |
| Observation, both variants | 7 `request_feedback`, 1 `poll_feedback_once`, 7 `get_state` |
| DM POS_VEL action, steady state | 6 `send_pos_vel`, 1 `send_force_pos`, 7 cached `get_state` |
| DM MIT action, FORCE_POS gripper | 6 `send_mit`, 1 `send_force_pos`, 1 cached `get_state` |
| RS MIT action, steady-state holding gripper | 7 `send_mit`, 1 poll, 1 cached `get_state` |
| RS torque-limit transition/refresh | 1 additional `robstride_write_param_f32` |

MotorBridge documents Damiao `request_feedback` as nonblocking and RobStride's
as a no-op (no individual query command). `get_state` reads an SDK cache, not a
fresh bus round trip. A poll is not seven acknowledged transactions. Commands
can generate feedback, so read/write traffic cannot simply be summed without
capturing the actual bus. The native SDK implementation and adapter buffering
are outside this package; fake call counts are **not** wire packet captures.

For scale, a classical CAN 8-byte standard-ID frame costs about 111 bits
including intermission before stuffing; an extended-ID frame about 131 bits.
At an **assumed** 1 Mbit/s, seven commands cost at least 0.777 ms (standard,
DM) or 0.917 ms (extended, RS); seven equally sized feedback frames double
those to 1.554/1.834 ms, before stuffing, arbitration and adapter overhead.
Confirm payload lengths, actual CAN bitrate and firmware response policy on
hardware. The DM host serial baud defaults to 921600: 8N1 costs 10.85 us per
byte, or at least 0.608 ms for seven raw 8-byte payloads, excluding adapter
framing. This is a separate pipelined transport, not a serial-servo budget.

Startup retains `disable_all`, seven `ensure_mode` calls, `enable_all`, a
measured-pose read and a hold command. The hold can retry feedback three times
with 10 ms retry sleeps. SDK `ensure_mode` and enable/disable transaction counts
depend on current modes/native SDK implementation; no fixed startup wire count
is claimed. No discovery pings or per-action enable writes exist to eliminate.
The RS gripper already caches its torque limit, refreshes it every 0.5 s, and
polls feedback for stall protection. None of these safety operations should be
removed to improve a fake benchmark. No acknowledgement policy, EEPROM setting,
torque lifecycle, sleep, feedback freshness, gains, or motion limit is changed.

## Hardware-free baseline

Baseline source: `b71bc333bab5f57fc55e375a6f414fa9a7cfa96d`.
Linux x86_64, Intel Core i9-12900K, CPython 3.13.1, NumPy 2.5.4,
defusedxml 0.7.1. MotorBridge is **not installed or loaded** by the harness.
It replaces the SDK before importing drivers, attaches inert fakes without
calling `connect()`, and never enables, parks, disconnects, or opens hardware.
Models parse outside timing. Defaults retain motion limiting and MIT gravity;
approach is disabled and the repeated action is the zero vector. Gravity uses
the fixed public-radian reach pose `[0.3, -1.5, -0.5, 0.4, 0.2, 0.1]`.

Each scenario warms up for 100 calls, then measures seven repeats of 1000
calls using `perf_counter_ns`. Reported means include timing/list overhead;
percentiles describe individual CPU-only calls. Runs share a host and are not
CPU-pinned. They do not measure throughput on hardware or real SDK/FFI cost.

| Scenario | Baseline mean ± repeat SD (us) | p50 / p95 / p99 / max (us) |
| --- | ---: | --- |
| DM gravity | 331.07 ± 5.74 | 330.97 / 343.32 / 360.73 / 604.95 |
| RS gravity | 461.63 ± 14.11 | 456.87 / 474.96 / 755.65 / 802.82 |
| DM POS_VEL read | 6.52 ± 0.03 | 6.35 / 6.76 / 8.51 / 12.36 |
| DM POS_VEL write | 53.59 ± 0.32 | 53.24 / 55.70 / 57.90 / 167.91 |
| DM MIT read | 6.46 ± 0.04 | 6.31 / 6.64 / 7.58 / 12.87 |
| DM MIT write | 405.02 ± 13.80 | 400.40 / 416.19 / 653.86 / 671.70 |
| RS read | 6.27 ± 0.07 | 6.11 / 6.49 / 7.37 / 72.43 |
| RS write | 529.01 ± 3.61 | 527.86 / 543.78 / 555.35 / 1425.49 |

The gravity calculation is material relative to the estimated wire budget. It
calls generic NumPy `cross` once per loaded-link/ancestor pair on tiny 3-vectors.
This is the optimization target, after finding no verified removable bus work.

## Reproduce

Run from the worktree root. The shared environment is only read, never synced
or installed into. It lacks defusedxml, so a package-local ignored environment
supplies that one small dependency while reusing existing tools read-only:

```bash
PKG=packages/physicalai-rebot-b601-plugin
SHARED=/home/mark/projects/geti/physicalai/.venv
uv venv --python "$SHARED/bin/python" "$PKG/.venv"
uv pip install --python "$PKG/.venv/bin/python" defusedxml==0.7.1
export PYTHONPATH="$PWD/$PKG/src:$PWD/src:$PWD/packages/physicalai-studio-plugin/src:$SHARED/lib/python3.13/site-packages"
uv run --no-project "$PKG/.venv/bin/python" "$PKG/benchmarks/benchmark_cpu.py" --iterations 1000 --repeats 7
uv run --no-project "$PKG/.venv/bin/python" -m pytest "$PKG/tests" -q
```

Baseline package suite: **243 passed in 1.70 s**.

## Change and paired results

Optimized source and paired harness: `ded0d9fed1960d1ab618982526ce359553f48ca4`.
Baseline harness: `e451d77147582ebafc443d39159b5538f2b864e2`.

Expand `dot(cross(axis, lever), weight)` into its three scalar terms. Convert
the small vectors to Python floats with `tolist()` instead of dispatching
generic NumPy `cross` and `dot` for each loaded-link/ancestor pair. Keep the
same kinematic traversal, link masses, joint frames, accumulation order and
float64 output. Do not cache torques or reuse caller-visible buffers. The
calculation still handles arbitrary gravity directions and arbitrary joint axes.

The final harness compares the original kernel (a reference subclass) and the
optimized model in the **same process**, alternating their order each repeat.
Only the gravity kernel differs; the reference subclass reproduces the old
implementation exactly. Fake drivers use separate state and model instances.
The SDK budget snapshot must match for each public scenario or the harness
raises. This pairing controls for order better than the initial sequential
baseline, but does not remove contention from other agents/processes on the
host. Both kernels run seven repeats of 1000 calls after 100 warmup calls.

| Scenario | Original mean ± SD (us) | Optimized mean ± SD (us) | Host speedup |
| --- | ---: | ---: | ---: |
| DM gravity | 336.90 ± 7.36 | 70.57 ± 1.15 | 4.77x |
| RS gravity | 451.64 ± 3.76 | 76.88 ± 0.81 | 5.87x |
| DM POS_VEL read | 6.30 ± 0.04 | 6.40 ± 0.04 | unchanged path |
| DM POS_VEL write | 52.80 ± 0.58 | 52.25 ± 0.68 | unchanged path |
| DM MIT read | 6.35 ± 0.04 | 6.37 ± 0.04 | unchanged path |
| DM MIT write | 400.87 ± 4.86 | 137.52 ± 10.12 | 2.91x |
| RS read | 6.33 ± 0.10 | 6.26 ± 0.04 | unchanged path |
| RS write | 532.48 ± 22.99 | 148.09 ± 13.99 | 3.60x |

| Affected scenario | Original p50 / p95 / p99 / max (us) | Optimized p50 / p95 / p99 / max (us) |
| --- | --- | --- |
| DM gravity | 335.74 / 346.51 / 363.64 / 1243.84 | 70.19 / 73.12 / 77.39 / 292.50 |
| RS gravity | 448.44 / 468.38 / 486.01 / 825.80 | 76.53 / 79.72 / 82.16 / 112.99 |
| DM MIT write | 400.63 / 415.70 / 431.71 / 559.43 | 132.35 / 214.65 / 218.54 / 258.44 |
| RS write | 522.29 / 547.76 / 870.46 / 890.59 | 142.88 / 231.33 / 235.23 / 322.51 |

All six read/write fake call snapshots match the budgets above, before and
after. No errors occurred in the timed calls. No motion accuracy, reply age,
dropped frames, hardware CPU use, real throughput or control jitter was measured.
The gain is CPU headroom in gravity-enabled MIT commands, **not** a claim of
4.77–5.87x faster physical motion or bus throughput. POS_VEL and observations do
not use gravity and show only small timing noise. In particular, do not infer a
read regression from the 0.10 us difference in the first read row.

## Regression checks

Final package suite: **253 passed in 1.94 s** (243 existing + 10 new cases).
The new tests compare the old kernel at zero, pi and 100 seeded random poses
for each bundled URDF, with ordinary, tilted and zero gravity. A synthetic
branched tree covers non-axis-aligned joints, rotated origins, fixed payloads,
an unactuated revolute joint and a reordered active-joint subset. Comparisons
use `rtol=atol=1e-13`, not bitwise parity: scalar summation and NumPy dot can
differ in the last floating-point bits. Existing analytic and MuJoCo-reference
tests remain independent checks of sign, magnitude and public/motor frames.
Additional tests preserve rejection of invalid gravity lengths, input/output
ownership, and reentrant calls on the same model. Existing driver tests still
cover motion shaping, dropout detection, gripper limits/stalls, park/release,
connection cleanup and error handling.

Checks from the root (repository-pinned Ruff 0.15.12, Pyrefly 1.4.1):

```bash
GRAVITY="$PKG/src/physicalai_rebot_b601_plugin/gravity.py"
BENCH="$PKG/benchmarks/benchmark_cpu.py"
TEST="$PKG/tests/unit/test_gravity_performance.py"
uv tool run --from ruff==0.15.12 ruff check "$PKG/src"
uv tool run --from ruff==0.15.12 ruff check --select E,F,I,UP \
  --config 'include=["*.py"]' --config 'extend-exclude=[]' "$BENCH" "$TEST"
uv tool run --from ruff==0.15.12 ruff format --check \
  --config 'include=["*.py"]' --config 'extend-exclude=[]' "$GRAVITY" "$BENCH" "$TEST"
uv run --no-project "$SHARED/bin/pyrefly" check \
  --python-interpreter-path "$SHARED/bin/python" \
  --search-path "$PKG/.venv/lib/python3.13/site-packages" "$GRAVITY"
git diff --check
```

The entire package source passes all configured pinned Ruff checks; benchmark/tests
pass the explicit selection (repository defaults exclude tests/benchmarks).
All three files pass formatting. Focused Pyrefly reports **0 errors**, with one
missing-defusedxml-stubs warning. The initial check with shared Ruff 0.17.5
reported 15 diagnostics in untouched files; these are not failures under the
repository-pinned Ruff 0.15.12. Existing numeric suppression IDs remain unchanged.
No shared lockfile/environment was modified and no heavy optional SDK was installed.

## Limits and future hardware validation

No protocol batching was attempted because the wrapper exposes no verified
multi-motor CAN command primitive. No feedback requests, stall-protection polls,
torque-limit refreshes or startup checks were removed. The Python SDK source
inspection does not verify native timing, firmware response semantics or all
supported MotorBridge versions (the package permits versions from 0.4.4).

Before claiming hardware gains, a supervised operator should:

1. Record DM/RS model, firmware, SDK version, bus bitrate, adapter, power supply,
   control settings, URDF, starting pose and commit SHAs. Support the arm and
   verify emergency stop/torque release with the existing safety procedure.
2. Capture actual CAN/serial traffic to confirm the estimated frame budget,
   command responses, feedback age and adapter queue behavior. Preserve all
   startup acknowledgements and torque-limit refreshes.
3. Alternate old/new runs sequentially on the same bus for 5–10 repeats, with
   2 s warmup and 10 s windows. Report latency p50/p95/p99/max, errors, dropped
   frames, CPU use and RSS, not only mean rate. Start with read-only captures;
   never run an unattended tight action loop on the robot.
4. For motion validation, require explicit operator confirmation and use small,
   slowly interpolated targets near a supported neutral pose. Check tracking,
   gravity signs, gripper force/stall protection, dropout handling and supervised
   park/release. Ramp back to the starting pose. A faster host loop does not
   justify increasing command rates or weakening limits without this validation.
