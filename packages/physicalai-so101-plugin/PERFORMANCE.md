# SO-101 CPU hot-path performance

## Scope and bus ceiling

This work starts at `b71bc33`, after the existing torque cache, grouped torque
writes, conversion tables, batched configuration, and sync-read discovery.
It does not repeat those optimizations. No hardware was connected or benchmarked.

At 1 Mbps with 8N1, each byte takes 10 microseconds. The SDK's protocol-1
packet layout gives these **ideal wire-only estimates** for six servos:

| Operation | Bytes | Wire time |
| --- | ---: | ---: |
| Position sync-read request | 14 | 0.14 ms |
| Six two-byte position status replies | 48 | 0.48 ms |
| Two-byte goal-position sync write | 26 | 0.26 ms |
| One-byte torque sync write, only on state changes | 20 | 0.20 ms |

The ideal steady-state read/write floor is therefore 0.88 ms, before servo
turnaround, USB/serial adapter buffering, SDK parsing, or host scheduling.
The handoff's hardware observations (~1 ms reads and ~782 Hz control) include
those costs; they are historical measurements, not results from this work.
There is no safe, justified extra bus reduction here without changing behavior.
In particular, this change does not add a parser, change status/EEPROM settings,
skip reads, suppress goal writes, or alter settle/drain/retry behavior.

## Change

Normalized observation conversion previously computed the same clipped
tick fraction twice, then clipped both derived body and gripper values again.
Compute the fraction once and remove the two redundant output clips: valid
calibration ranges already bound the fraction to `[0, 1]`. Retain the original
multiply/subtract order, float64 intermediates, final float32 cast, invalid-range
zero fallback, and NaN propagation. Do not replace division with reciprocals or
reassociate arithmetic.

Normalized actions also no longer construct an upper-bound array containing
100 in every element. NumPy broadcasts the identical scalar upper bound.
Finite-input checks, round-to-even behavior, int32 output, and final hardware
encoder safety clamping remain unchanged. No buffers are cached or reused.
Degree conversion and all SDK communication code remain unchanged.

## Hardware-free measurements

Environment: Linux 6.8.0-147-generic x86_64, Intel Core i9-12900K, Python 3.13.1,
NumPy 2.5.4, feetech-servo-sdk 1.0.0. All runs used the existing root virtual
environment read-only. CPU affinity was not pinned (24 logical CPUs available).
Other package benchmark agents ran concurrently; contention and frequency
changes affect the absolute times and individual batches.

Before changing source, a seven-repeat, 10,000-call run measured normalized
conversion at 7.634 microseconds/read and 9.725 microseconds/action, with fake
observation/action/control at 9.899/13.623/23.873 microseconds. The original
package suite passed all 225 tests. The fake control cycle already used one
read and one write; torque writes were absent.

The primary comparison below uses the final benchmark harness with the reviewed
baseline conversion module exported from `b71bc33`. Both variants use the same
current driver and separate identical fake SDK groups. Only their conversion
tables differ. Each scenario warms both variants with 1,000 calls, then runs
11 pairs of 20,000-call batches, alternating current/baseline order each pair.
Results are **host-only batch-average latency**, not serial throughput, per-call
tail latency, or hardware speedups. The fake groups count SDK entry points and
never open a port. They omit the real SDK's packet construction, serial I/O,
and input-buffer drain costs.

| Unit | Scenario | Baseline median, us | Current median, us | Median paired latency reduction |
| --- | --- | ---: | ---: | ---: |
| normalized | ticks to configured unit | 7.372 | 4.546 | 38.12% |
| normalized | configured unit to ticks | 9.519 | 9.092 | 4.74% |
| normalized | observation | 10.008 | 7.123 | 29.18% |
| normalized | action | 14.040 | 13.481 | 4.68% |
| normalized | observation + action | 23.834 | 20.412 | 13.95% |
| degrees | ticks to configured unit | 4.217 | 4.237 | -0.11% |
| degrees | configured unit to ticks | 8.047 | 8.026 | 0.60% |
| degrees | observation | 6.883 | 6.926 | -0.62% |
| degrees | action | 12.366 | 12.337 | -0.23% |
| degrees | observation + action | 19.626 | 19.488 | 0.74% |

Each paired percentage is the median of `100 * (1 - current / baseline)` for
the corresponding paired batches, not a ratio of the two medians. Negative
values mean slower. Degree code is unchanged; its sub-percent paired variation
is a useful noise control, not an improvement claim. Normalized read conversion
batch averages ranged from 7.230–9.162 us baseline and 4.497–4.727 us current;
normalized control ranged from 23.638–24.402 us baseline and 20.122–22.640 us
current. The full harness emits every batch in JSON for independent inspection.

The roughly 3.4 us normalized fake-control savings are under 0.4% of the ideal
wire-only read/write budget. The main benefit is less host work/allocation;
do not interpret the 13.95% fake-control reduction as real robot throughput.
No CPU-utilization, GIL-concurrency, memory-peak, or hardware latency claim is made.

Both before and after, one steady-state control cycle makes exactly:

| Fake SDK entry point | Calls |
| --- | ---: |
| position read `txRxPacket` | 1 |
| position read `isAvailable` / `getData` | 6 each |
| goal write `clearParam` / `addParam` / `txPacket` | 1 / 6 / 1 |
| torque group methods | 0 |

## Reproduce

Run from the assigned worktree root. These commands do not install dependencies
or synchronize the shared environment. The optional baseline argument imports
Python code: pass only the trusted, reviewed local source exported below.

```bash
PY=/home/mark/projects/geti/physicalai/.venv/bin/python
export PYTHONPATH="$PWD/packages/physicalai-so101-plugin/src:$PWD/src"
PKG=packages/physicalai-so101-plugin

# Initial baseline command (before modifying conversion source):
uv run --no-project "$PY" "$PKG/benchmarks/benchmark_hot_path.py"

git show b71bc33:"$PKG/src/physicalai_so101_plugin/_conversion.py" \
  > /tmp/opencode/so101-baseline-conversion.py
uv run --no-project "$PY" "$PKG/benchmarks/benchmark_hot_path.py" \
  --baseline-conversion /tmp/opencode/so101-baseline-conversion.py \
  --number 20000 --repeat 11

uv run --no-project "$PY" -m pytest "$PKG/tests" -q
uv run --no-project "$PY" -m ruff check "$PKG/src/physicalai_so101_plugin/_conversion.py"
uv run --no-project "$PY" -m ruff format --check "$PKG/src/physicalai_so101_plugin/_conversion.py"
uv run --no-project "$PY" -m ruff check --isolated --select E,F,I --line-length 120 \
  "$PKG/benchmarks/benchmark_hot_path.py" "$PKG/tests/unit/test_hot_path_contract.py"
uv run --no-project "$PY" -m ruff format --isolated --line-length 120 --check \
  "$PKG/benchmarks/benchmark_hot_path.py" "$PKG/tests/unit/test_hot_path_contract.py"
uv run --no-project "$PY" -m pyrefly check "$PKG/src/physicalai_so101_plugin/_conversion.py"
```

Checks: **234 package tests passed**, including nine new regression cases;
changed-source Ruff checks and formatting passed; explicit E/F/I checks and
formatting passed for the new benchmark/tests (these paths are outside the
repository's default source-only Ruff selection); changed-source Pyrefly
reported zero errors. Full-repository hooks and type checks were not run.

The independent numerical reference freezes the pre-change expressions rather
than calling the convenience wrapper that delegates to the same implementation.
Coverage includes every encoder tick plus out-of-range neighbors in three input
dtypes, randomized fractional/noncontiguous arrays, NaN/infinities, degenerate
legacy ranges, command endpoints and half-tick rounding, output dtypes,
observation/output ownership, and fake SDK call counts in both calibrated units.
The existing suite covers errors, retries, sign decoding, and safety clamps.

## Future hardware validation

No hardware validation is required to exercise the exact numerical parity tests,
but real performance remains unmeasured. Use the same reviewed calibration,
baudrate, adapter, unit, starting pose, and software versions for alternating
baseline/current runs. Record valid samples, communication errors/retries, and
p50/p95/p99/max latency, with repeated warmup and fixed measurement windows.
Start read-only. For any motion validation, support the arm, use small bounded
interpolated moves, require operator confirmation, retain an emergency stop,
and never benchmark unattended. Do not change EEPROM/status settings or parser
behavior as part of this validation. Expect small wall-time differences on a
bus-bound arm; prioritize correctness and CPU headroom over throughput claims.
