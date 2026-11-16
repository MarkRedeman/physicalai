# Package performance investigation

This investigation applies the SO-101 performance handoff across the seven
first-party packages. Each package was assessed independently against baseline
`b71bc333bab5f57fc55e375a6f414fa9a7cfa96d`, using its own worktree and incremental
signed Conventional Commits. The reviewed changes are integrated on
`mark/perf-additional-performance-improvements`.

## Scope and safety

No physical hardware was connected or moved. Measurements use CPU-only helpers,
inert vendor SDK replacements, fresh Python interpreters, or headless simulation.
They do not establish physical robot throughput, motion accuracy, or bus latency.
Other workers shared the host, so the reports describe scheduling noise and
whether before/after runs alternated.

The drivers retain their transport operations, feedback freshness checks, safety
reads, motion limits, torque lifecycle, acknowledgements, and configuration writes.
No unverified protocol batching, EEPROM changes, or hot-loop sleeps were added.
Vendor SDK calls are not assumed to be wire round trips. Each package report
documents the available protocol evidence and remaining hardware validation.

## Measured host-side improvements

These are different workloads, not a ranking of packages or languages. Consult
the linked reports for inputs, statistics, dependency versions, commands, and
limitations. Some results are batch averages rather than per-call percentiles.

| Package | Change | Representative before → after |
| --- | --- | --- |
| [LeRobot](../../packages/physicalai-lerobot-plugin/PERFORMANCE.md) | Reuse discovered joint-key membership | Six-joint observation shaping: 1.308 → 0.994 µs; 24% less host time |
| [SO-101](../../packages/physicalai-so101-plugin/PERFORMANCE.md) | Reuse normalized tick fractions and remove redundant conversion work | Normalized read conversion: 7.372 → 4.546 µs; fake control cycle: 23.834 → 20.412 µs |
| [Stararm](../../packages/physicalai-stararm-plugin/PERFORMANCE.md) | Replace repeated scalar NumPy clipping dispatch | Follower observation: 21.281 → 7.404 µs; action: 16.627 → 2.475 µs |
| [Trossen](../../packages/physicalai-trossen-plugin/PERFORMANCE.md) | Vectorize angular-joint conversion while retaining gripper passthrough | Follower observation: 2.455 → 1.799 µs; action: 4.774 → 3.967 µs |
| [Rebot B601](../../packages/physicalai-rebot-b601-plugin/PERFORMANCE.md) | Expand tiny cross/dot products in gravity feedforward | DM gravity: 336.90 → 70.57 µs; RS gravity: 451.64 → 76.88 µs |
| [Studio](../../packages/physicalai-studio-plugin/PERFORMANCE.md) | Lazy public exports and linear duplicate-scene validation | Plain SDK import: 153.471 → 0.168 ms; validation of 256 scenes: 349.658 → 7.171 µs |
| [MuJoCo](../../packages/physicalai-mujoco-plugin/PERFORMANCE.md) | Batch native physics substeps when no Python controller needs per-step execution | Ten-substep ticks: 22.344 → 19.228 µs; forty-substep ticks: 76.112 → 63.081 µs against the paired Python-loop reference |

SO-101 is already close to its wire floor: the measured fake control-cycle
saving is less than 0.4% of the estimated ideal wire budget. Rebot's gravity
calculation provides a larger absolute CPU saving, but CAN/adapter timing still
needs measurement. Studio's plain-import improvement does not represent full
application startup; accessing all exports still loads their dependencies.

### Stararm's remaining latency suspects

There is no SO-101-style redundant torque write on every action. Default startup
performs seven pings, seven unlocks, and seven multi-turn resets; normal actions
do not repeat that configuration. However, the follower/assist action still sends
seven individual `set_angle` commands. The inspected SDK has no public batch-angle
write API, and native write acknowledgement/timeout behavior remains unverified.
The cached `motorbridge-smart-servo` 0.0.4 metadata explicitly labels write control
temporarily unsupported. The CPU optimization does not establish write reliability
or resolve a native serial bottleneck.

The plugin's direct-call default `goal_time=0.1` becomes a 100 ms servo movement
duration, with `command_interval_ms=10` as a lower bound. This is not a host sleep
or a 10 Hz call limit. Runtime callers can pass a different duration; `RobotRuntime`
uses three control ticks. Repeated interpolated targets can therefore feel laggy
without the Python call itself being slow. Do not reduce durations blindly.

Passive HD leaders send no action commands. Their read path uses one
`sync_monitor` request, but the SDK can return cached values with `reliable=False`.
The plugin also holds jumps larger than 90 degrees for up to three samples and
returns cached positions after read exceptions, marked unreliable. Measure raw
positions, reliability, missed replies and native read latency before attributing
leader lag to CPU overhead. Existing filtering and fallback behavior were retained.

## Validation and environment limits

The existing environment uses CPython 3.13.1 on an Intel Core i9-12900K running
Linux. Shared dependencies and lockfiles were not changed. Package source paths
were selected explicitly, and packages were also tested in separate processes to
avoid same-named test modules and optional-SDK mock interference.

After integrating all seven packages:

- SO-101: **234 passed**; Stararm: **79 passed**; Studio: **91 passed**;
  Trossen: **46 passed** using an import-only SDK placeholder and mocked drivers.
- Rebot B601: **253 passed** with `defusedxml` supplied by an isolated environment.
- MuJoCo: **238 passed, 22 deselected** in the focused asset-free/profile/robot/
  services suite, using isolated optional dependencies. This includes **25 new
  regression cases**. Viewer-dependent tests, full downloaded scenes, and the
  smoke matrix remain unvalidated; missing assets and viewer SDKs are not masked.
- LeRobot: **82 passed, 31 failed**, versus **58 passed, 31 failed** before the
  changes. The same failures require unavailable `lerobot` or `draccus`.
  Its focused hardware-free adapter checks pass: **48 passed, 1 deselected**.
- Core unit tests: **1,729 passed, 24 failed, 67 skipped**, versus baseline
  **1,728 passed, 25 failed, 67 skipped**. There are no new failing test IDs.
  The baseline capture freshness failure passed on rerun; this investigation
  does not claim to fix its timing variability. Remaining failures are outside
  the package changes and are recorded in the baseline comparison.
- Pyrefly: the same **133 errors**, with matching diagnostic headings, paths,
  count, and order. The existing errors were not suppressed or broadened into
  this performance work.
- Repository-pinned Ruff lint and formatting both pass across the configured
  source tree. Source whitespace checks also pass. Full mutation-capable hooks
  were not run because they include lockfile synchronization.

Repository-pinned Ruff is **0.15.12**, not the newer version in the shared
environment. Use the pinned tool when reproducing lint and format checks:

```bash
uv tool run --from ruff==0.15.12 ruff check
uv tool run --from ruff==0.15.12 ruff format --check
uv run --no-sync pytest tests/unit -q
uv run --no-sync pyrefly check
```

Package reports include source-path setup, focused checks, and benchmark
commands. Full hardware-dependent validation and the pre-existing missing-SDK
checks remain outstanding. Before any motion benchmark, require an attending
operator, explicit confirmation, a supported safe pose, small interpolated
targets, unchanged units/calibration, and a slow return. Start with read-only
measurements, alternate revision order, and report errors and latency tails
alongside mean throughput.
