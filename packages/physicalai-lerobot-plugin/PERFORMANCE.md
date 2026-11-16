# LeRobot adapter performance

## Scope and safety

The adapters delegate device I/O, calibration, torque, and motion limits to
LeRobot. This change only reduces Python observation-shaping overhead. No device
was connected, no SDK was installed, and no calibration or hardware configuration
was changed. Both adapters still perform one vendor read per observation and one
vendor write per valid action. Startup still performs its initial read.

The handoff's first priority is redundant I/O. There is no redundant bus operation
at this adapter layer to remove safely: LeRobot supports many unrelated devices,
and its implementations own register batching and retries. In particular, caching
observations or skipping repeated actions would change freshness or device-side
behavior. A meaningful wire-byte/round-trip budget needs a particular LeRobot
device, SDK, protocol, and baud rate; none was selected or measured here. Adapter
wire packets added: zero before and after.

## Change

Cache an immutable set of discovered joint keys instead of constructing two sets
on every robot observation and one on every teleoperator observation. Remove one
redundant joint-discovery guard call per observation; the existing key-requirement
helper still checks discovery. Reset the cached membership on state restoration
and populate it during discovery, including empty and fallback key schemas.

Only joint membership is cached. Each call still scans the actual incoming
dictionary, so auxiliary keys and value types can change at runtime. Joint
positions remain fresh, independent float32 arrays; images and array-valued
sensors retain their original zero-copy behavior. Camera sequence and timestamp
behavior, key ordering, missing-key exceptions, leader-role protection, action
validation, disconnect behavior, and live-device-free exported state stay intact.

Action conversion was measured but left unchanged. `map(float, action)` plus
`dict(zip(...))` was slower in isolated experiments; `action.tolist()` would need
dtype-specific handling to preserve Python-float conversion for integers, object
arrays, and extended floats. No action dictionary or observation array is reused.

## Hardware-free measurements

Environment: Intel Core i9-12900K, Linux 6.8.0-147-generic x86_64/glibc 2.39,
CPython 3.13.1, NumPy 2.5.4, Loguru 0.8.0, Pydantic 2.14.3. Runtime imports came
from this worktree's `src`, not the installed runtime distribution. LeRobot and
draccus were unavailable; neither is needed by these prebuilt fake devices.

- Baseline source: `b71bc333bab5f57fc55e375a6f414fa9a7cfa96d` (also unchanged
  in benchmark/test commit `02cdade7cd6eaf23cf50d5f1c7a27c481426b140`).
- Optimized source: `c0d539b9167afdbba970297c5fa41ab84ea4eacc`.
- Initial pre-change measurement: seven repeats of 20,000 calls after 2,000-call
  warmup. Six-joint robot read averaged 1.246 µs, teleoperator read 1.100 µs;
  fourteen-joint robot read 1.655 µs, teleoperator read 1.463 µs.
- Final comparison: nine repeats of 50,000 calls per version per scenario, after
  2,000 warmup calls per version. A/B order alternates each repeat, in one process.
  Cyclic GC is disabled only while timing. The timer wraps a batch, not each call.
- Fakes return a prebuilt dictionary and discard writes. `aux` adds one scalar,
  one vector, and one 8×8 RGB image (no image copy). Writes use float32 arrays.
- **Contention-sensitive:** other package agents ran on the same host. There was
  no CPU affinity, frequency lock, or system isolation. Treat small write-control
  differences as noise, not a change to the untouched write path.

Mean ± sample standard deviation across repeats, in **µs per call**:

| Scenario | Joints | Baseline | After | Mean reduction |
| --- | ---: | ---: | ---: | ---: |
| Robot read | 6 | 1.308 ± 0.076 | 0.994 ± 0.020 | 24.0% |
| Teleoperator read | 6 | 1.134 ± 0.026 | 0.967 ± 0.020 | 14.7% |
| Robot read, aux | 6 | 2.339 ± 0.030 | 2.025 ± 0.019 | 13.4% |
| Teleoperator read, aux | 6 | 2.244 ± 0.040 | 2.049 ± 0.026 | 8.7% |
| Robot write (control) | 6 | 0.648 ± 0.013 | 0.644 ± 0.009 | 0.6% |
| Teleoperator write (control) | 6 | 0.625 ± 0.004 | 0.621 ± 0.005 | 0.6% |
| Robot read | 14 | 1.685 ± 0.017 | 1.318 ± 0.021 | 21.8% |
| Teleoperator read | 14 | 1.509 ± 0.089 | 1.301 ± 0.026 | 13.8% |
| Robot read, aux | 14 | 2.745 ± 0.023 | 2.362 ± 0.016 | 14.0% |
| Teleoperator read, aux | 14 | 2.747 ± 0.149 | 2.450 ± 0.065 | 10.8% |
| Robot write (control) | 14 | 1.213 ± 0.029 | 1.245 ± 0.094 | -2.6% |
| Teleoperator write (control) | 14 | 1.193 ± 0.013 | 1.212 ± 0.014 | -1.6% |

No benchmark calls raised errors. The benchmark reports mean, standard deviation,
minimum, and maximum of batch-average call times. These are **not per-call tail
latencies**, robot throughput, bus timings, CPU utilization, or motion accuracy.
No RSS or allocation-byte claim is made; eliminated temporary set counts follow
directly from the implementation.

### Reproduce from the repository root

Use an existing environment containing the runtime's lightweight dependencies,
NumPy, and pytest. The commands below use the shared environment without syncing
or installing into it. Update `PYTHON` for another checkout. `--baseline` executes
trusted local Python source; never point it at unreviewed downloaded code.

```bash
export PYTHONPATH="$PWD/packages/physicalai-lerobot-plugin/src:$PWD/src"
PYTHON=/home/mark/projects/geti/physicalai/.venv/bin/python
git show b71bc33:packages/physicalai-lerobot-plugin/src/physicalai_lerobot_plugin/lerobot_adapter.py \
  > /tmp/opencode/lerobot-adapter-baseline.py
uv run --no-project "$PYTHON" \
  packages/physicalai-lerobot-plugin/benchmarks/adapter_overhead.py \
  --baseline /tmp/opencode/lerobot-adapter-baseline.py --iterations 50000 --repeats 9
```

Omit `--baseline` to measure the current version alone.

## Regression checks

```bash
export PYTHONPATH="$PWD/packages/physicalai-lerobot-plugin/src:$PWD/src"
PYTHON=/home/mark/projects/geti/physicalai/.venv/bin/python
uv run --no-project "$PYTHON" -m pytest \
  packages/physicalai-lerobot-plugin/tests/unit \
  packages/physicalai-lerobot-plugin/tests/test_lerobot_adapter.py \
  -q -k 'not config_lookup'
uv run --no-project "$PYTHON" -m pytest packages/physicalai-lerobot-plugin/tests -q
```

- Targeted hardware-free adapter checks: **48 passed, 1 deselected**, 0.17 s.
  The deselected existing config-lookup test requires LeRobot.
- Full package before changes: **58 passed, 31 failed**, 0.36 s.
- Full package after changes: **82 passed, 31 failed**, 0.22 s. The same 31 tests
  fail because `lerobot` or `draccus` is missing, not because of this change.
  Those dependency-backed calibration/config/catalog checks remain unvalidated.
- The 24 new regression cases cover both adapters, fresh observation ownership,
  dynamic auxiliary data and types, image sequence continuity, one read per call,
  sorted joint order, Python-float actions and independent dictionaries, float32/
  float64/integer/object and strided actions, invalid/disconnected/leader writes,
  missing joints, restored discovery, cached membership reuse, empty schemas, and
  fallback position keys.
- Repository-pinned Ruff 0.15.12 (`uv tool run --from ruff==0.15.12 ruff check`)
  passed on package source, benchmark, and the new tests.
  `ruff format --check` passed on all three changed Python files.
  Shared-environment Ruff 0.17.5 additionally flags existing `noqa` syntax; this
  task does not migrate unrelated existing suppression comments to its new style.
- `uv run --no-project /home/mark/projects/geti/physicalai/.venv/bin/pyrefly check
  packages/physicalai-lerobot-plugin/src/physicalai_lerobot_plugin/lerobot_adapter.py`
  reported **0 errors (4 suppressed)** using the repository configuration.
- `git diff --check` passed. No repository-wide hooks were run against other
  packages, and no shared files or lockfiles were edited.

## Remaining hardware work

1. Install the supported LeRobot dependencies in an isolated environment and run
   the complete package tests, including calibration and catalog discovery.
2. Select a device and record LeRobot/SDK revisions, protocol, packet lengths,
   baud rate, units, calibration, and starting pose. Count actual transactions
   inside LeRobot before considering any lower-level batching or torque caching.
3. Measure observation-only wall-clock latency first. Run sequential alternating
   A/B windows with warmup and at least five repeats; report per-call p50/p95/p99/
   maximum, errors, dropped operations, and CPU utilization with cameras active.
4. If motion testing is needed, require an explicit operator-confirmed hardware
   mode, support the arm, use small interpolated moves and existing limits, and
   return slowly to the starting pose. Never run unattended.

The expected real-device benefit is small host-CPU headroom: approximately
0.17–0.38 µs saved per observation in these scenarios. No end-to-end hardware
speedup is claimed. Do not remove calibration validation, retries, or freshness
checks to chase this microbenchmark.
