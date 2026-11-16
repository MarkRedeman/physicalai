# Trossen host-path performance

## Scope and protocol budget

This change reduces Python/NumPy work, not network transactions. No hardware
connections, motion, SDK construction, or hardware benchmarks were performed.

The SO-101 serial-bus budgets do **not** apply to this Ethernet-controlled arm.
The plugin already uses all-joint operations. Before optimizing, we inspected
the plugin and the vendor's public C++ header and programming guide at
[`TrossenRobotics/trossen_arm@65c1532`](https://github.com/TrossenRobotics/trossen_arm/tree/65c1532e1be3102a8d1c0862a96c7df2b8b1ac49):

- `docs/programming_guide/concepts.rst` describes UDP real-time commands/state
  and TCP handshake/configuration/logging, with Python binding the C++ driver.
- `include/libtrossen_arm/trossen_arm.hpp` exposes a daemon thread, shared state,
  mutexes, and a communication cycle from `set_joint_inputs` through
  `receive_robot_output`. **An SDK getter count is not a network round-trip
  count.** The binary library implementation is not present as source in that
  checkout, and the native SDK is not installed in the benchmark environment.
- As a lower-bound field budget only, its raw input has one mode byte plus three
  position/feedforward floats per joint: `7 * (1 + 3 * 4) = 91` bytes. Its raw
  output has an ID, timestamp, and six floats per joint:
  `4 + 8 + 7 * 6 * 4 = 180` bytes. This is **not** an asserted wire packet layout:
  padding, framing, command fields, network overhead, and extra packets are not
  verified. Even 271 payload bytes would take only 21.68 microseconds to serialize
  at a hypothetical 100 Mbit/s; this is not a control-cycle or round-trip bound.
  Link speed, scheduling, firmware processing, SDK synchronization, retransmits,
  and compatibility across supported SDK versions need hardware validation.

The wrapper-level budget remains the same before and after:

| Operation, one arm | SDK operations | Wire round trips |
| --- | --- | --- |
| Follower observation | positions + velocities + external efforts getters | Not established; SDK owns transport |
| Leader observation | positions + velocities getters | Not established |
| Follower action | fresh positions getter + nonblocking all-positions setter | Not established |
| Leader feedback | all-modes setter + all-external-efforts setter | Not established; preserve configuration semantics |
| Follower connect | configure + all-modes + blocking all-positions | Configure encapsulates multiple operations; includes 2 s home goal |
| Leader connect | follower sequence + all-modes + all-external-efforts | Same caveat |
| Disconnect | all-modes + blocking all-positions + cleanup | Includes 2 s home goal |

These are API calls, not per-joint operations or measured packets. The benchmark
counts steady-state getters/setters with a fake driver. Source inspection covers
startup/feedback calls; no startup timing is claimed. Bimanual operation delegates
to each single-arm driver and benefits from the same conversions.

We deliberately did not cache the action safety read, skip repeated leader mode
writes, consolidate getters through a different snapshot API, or change any
acknowledgement, broadcast, EEPROM, retry, or homing behavior. The available
protocol evidence does not justify those changes.

## Implementation and regression coverage

Six per-joint scalar ufunc calls become one in-place ufunc on the six angular
joints of an already-owned float32 array. The gripper remains an untouched native
scalar. The name-based fallback preserves custom joint layouts and errors from
short SDK state vectors. The action input is still copied, and each observation
still owns independent position and sensor arrays. There is no shared scratch
buffer or state cache.

The original clipping calculation, warning, fresh-position read, exception
propagation, goal time, and nonblocking command remain unchanged. Tests compare
float32 output bits against the original scalar algorithm over deterministic
random values and boundaries (including signed zero, NaN and infinity for
observations). Action tests cover float32/float64/integer input, strided read-only
arrays, ignored extra dimensions, clipping against changing state, and failed
reads that must prevent writes. Existing lifecycle and bimanual tests still pass.

An import-only SDK placeholder in `tests/conftest.py` allows existing fully mocked
driver tests to execute instead of skipping when the native SDK is absent. Their
fixtures replace the placeholder before using any driver methods.

## Hardware-free measurements

Environment: Linux 6.8.0-147-generic x86_64, Intel Core i9-12900K, CPython 3.13.1,
NumPy 2.5.4. The fake SDK uses `list` for `VectorDouble`; native binding, locking,
transport, firmware, and interpolation costs are excluded.

Baseline code: `b71bc33` (unchanged driver in baseline harness commit
`dd3ecd8`). Optimized code: `b5861be`. Baseline source was checked against the
original checkout by SHA-256:
`60406a83713b8c6dfe7fba09bbc5242332e075a2017e3b090ad927d337edc30f`.

Each process warms up each scenario for 1,000 calls, then runs seven repeats of
20,000 calls. Timings are repeat averages, not individual-call latency
percentiles. Three process pairs ran in alternating order: baseline/optimized,
optimized/baseline, baseline/optimized. Other package workers were active on this
host; no CPU affinity, governor control, or contention isolation was used.

| Scenario | Baseline process medians, microseconds | Optimized process medians, microseconds | Median of medians reduction |
| --- | --- | --- | --- |
| Follower observation | 2.455, 2.447, 2.460 | 1.799, 1.818, 1.760 | 26.7% |
| Follower action | 4.774, 4.778, 4.740 | 3.942, 3.967, 4.000 | 16.9% |
| Leader observation | 2.141, 2.080, 2.079 | 1.416, 1.375, 1.441 | 31.9% |

Across all 21 repeats per variant, the minimum/maximum repeat averages were:

| Scenario | Baseline microseconds | Optimized microseconds |
| --- | --- | --- |
| Follower observation | 2.424 / 2.997 | 1.721 / 2.457 |
| Follower action | 4.703 / 5.736 | 3.904 / 4.051 |
| Leader observation | 2.068 / 2.242 | 1.360 / 1.453 |

Every process reported the same fake calls per operation: follower observation
`positions=1, velocities=1, efforts=1`; follower action
`positions=1, set_positions=1`; leader observation `positions=1, velocities=1`.
No benchmark errors occurred. The timed action stays inside the clipping limit
so warning logging does not distort the CPU conversion measurement. This is
roughly 0.7–0.8 microseconds saved per API call, **not** a robot throughput claim.

### Reproduce

From the repository root, use an existing environment without syncing or
installing into it. In this environment:

```bash
PY=/home/mark/projects/geti/physicalai/.venv/bin/python
PKG=packages/physicalai-trossen-plugin
export PYTHONPATH="$PWD/$PKG/src:$PWD/src"
uv run --no-project "$PY" "$PKG/benchmarks/benchmark_hot_path.py"
uv run --no-project "$PY" -m pytest "$PKG/tests" -q
```

For A/B reproduction, keep the same benchmark script and runtime source, point
the first `PYTHONPATH` entry at the package `src` directory of a separate checkout
of `dd3ecd8`, and alternate that entry with the optimized checkout as described
above. The script replaces even an installed SDK before importing the driver; it
cannot instantiate a real vendor driver. Do not call `connect()` in this harness.

### Validation results

- Full package tests: **46 passed in 0.41 s**. An earlier unoptimized run with the
  SDK placeholder executed the 31 existing single/bimanual tests successfully.
- Focused Pyrefly check of `src/physicalai_trossen_plugin/widowxai.py`: **0 errors**
  (root configuration ignores missing third-party imports; no native SDK types
  were validated).
- Ruff 0.17.5 source check: four pre-existing findings, three `RUF105` file-level
  suppression-style warnings and one `PLW0717` startup try-block warning.
  `ruff check packages/physicalai-trossen-plugin/src --ignore RUF105,PLW0717`
  passes. Unrelated existing code was not changed to silence them.
- Ruff source formatting: six files already formatted. Isolated formatting with
  line length 120 for the changed test, conftest, and benchmark: three files
  already formatted. Root Ruff inclusion/exclusion patterns omit tests and the
  benchmark, so these were checked explicitly with `--isolated`.
- `git diff --check`: passed. Repository-wide hooks/tests were not run; this work
  is scoped to the Trossen package. No environment sync/install was performed.

## Future hardware validation

Before claiming control-loop improvements, record exact SDK/firmware, link speed,
topology, robot role, goal time, joint units, load, and starting pose. With an
operator and safety stop, support the arm, use small interpolated moves around a
known safe pose, and retain homing/cleanup. Do not run motion tests unattended.

Alternate baseline/optimized runs on the same arm after warmup. Record per-call
p50/p95/p99/max latency, error counts, clipping events, state freshness, packet
captures and CPU utilization over repeated fixed-duration windows. Verify native
`VectorDouble` conversion and float32 parity on supported NumPy versions. Inspect
the installed SDK's actual getter synchronization and UDP/TCP packet counts
before considering a combined state snapshot or cached mode optimization. Keep
the fresh safety read and acknowledged configuration behavior unless independent
protocol evidence and hardware tests justify a change.
