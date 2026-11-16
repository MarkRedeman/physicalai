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

Baseline package suite: **243 passed in 1.70 s**. Measurements of optimized
code, parity coverage and future hardware validation follow with the change.
