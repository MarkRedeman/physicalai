# Star Arm host performance

The original CPU-only investigation below retains the seven-write transport path.
The subsequent [native sync-angle extension](#native-sync-angle-extension) reduces
actual command packets with a reviewed, locally built SDK wheel.

## Bus assessment before optimization

The seven-joint driver uses FashionStar UART at 1,000,000 baud by default.
Assuming 8N1, each wire byte takes 10 microseconds (multiply by
1,000,000 / configured baudrate at other rates). The inspected
`motorbridge-smart-servo==0.0.4` wheel's `fashionstar.py` documents
`sync_monitor` as one sync command (code 25). It delegates framing, status
handling, timeouts and retries to an opaque PyO3 implementation. Its Python
API exposes no multi-servo angle write. Do not equate SDK calls with measured
wire round trips:

| Operation | SDK calls before and after |
| --- | --- |
| Default connect | 7 ping + 7 unlock + 7 reset_multi_turn |
| Connect with zero_on_connect | above + 7 set_origin_point |
| Observation | 1 sync_monitor for IDs 0–6 |
| Follower/assist action | 7 addressed set_angle calls |
| Passive leader action | 0 |
| disable_torque | 7 unlock |

For request length Q, per-servo reply length R and seven servos, the read
serialization budget is `(Q + 7R) * 10 us` if all respond. For an addressed
write with request W and reply A, seven writes cost `7(W + A) * 10 us`.
These are formulas, **not measured byte counts**: packet sizes and whether
writes wait for replies are not established by the available wrapper. Native
source/protocol documentation or a hardware trace is required to resolve them.
Keep discovery, configuration, origin writes and acknowledgement behavior
unchanged. No EEPROM changes, torque cache, broadcast conversion, new sleeps,
or retries are introduced.

The cached SDK metadata also warns that angle-write behavior is not guaranteed
in its 0.0.4 release line. This work does not validate or expand write support.

## Hardware-free benchmark

Run from the worktree root (no installs or device access):

```bash
export PYTHONPATH="$PWD/packages/physicalai-stararm-plugin/src:$PWD/src:$PWD/packages/physicalai-studio-plugin/src"
uv run --no-project /home/mark/projects/geti/physicalai/.venv/bin/python \
  packages/physicalai-stararm-plugin/benchmarks/host_overhead.py
uv run --no-project /home/mark/projects/geti/physicalai/.venv/bin/python -m pytest \
  packages/physicalai-stararm-plugin/tests -q
```

The script replaces the SDK module **before importing the driver**, so even
`connect()` only constructs an in-memory bus. It counts calls separately and
times public APIs with a monotonic nanosecond timer, 1,000 warmups per scenario,
5 repeats of 5,000 calls, fixed float32 targets, and zero-degree reliable
monitor samples. Samples include timer, fake bus and counter overhead. Results
are microseconds per call, not real robot throughput. All calls succeeded.

Baseline source: `b71bc33` (repository starting revision; see git history for
the benchmark-only commit). Environment: Linux 6.8.0-147-generic x86_64,
CPython 3.13.1, NumPy 2.5.4, Intel Core i9-12900K. No hardware connected or motion performed.

| Scenario | Baseline mean | p50 | p95 | p99 | max |
| --- | ---: | ---: | ---: | ---: | ---: |
| Follower observation | 21.281 | 17.833 | 29.807 | 31.187 | 163.944 |
| Follower action | 16.627 | 13.887 | 23.897 | 24.204 | 46.115 |
| B601-profile observation | 37.217 | 31.213 | 52.360 | 53.819 | 102.580 |
| B601-profile assist action | 18.622 | 15.578 | 26.345 | 26.739 | 3063.607 |

Baseline tests: 70 passed. Fake counts matched the table above.

## Optimization and results

Replace per-joint scalar `np.clip` calls with a shared finite-bound scalar
comparison helper in native observation shaping, action clipping and the B601
profile transform. This avoids repeated NumPy dispatch without changing
float32 conversion order, wrap rounding, glitch counters or bus operations.
Keep the original NumPy operation for exact zero at a zero bound: NumPy
versions differ in the sign of that result. NaNs still propagate through
action clipping; the existing read-error/fallback behavior stays unchanged.
No returned arrays or caller-owned action buffers are reused or mutated.

After results, same command and environment (5 × 5,000 measured calls):

| Scenario | After mean | p50 | p95 | p99 | max | Mean host speedup |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Follower observation | 7.404 | 7.333 | 7.824 | 9.037 | 18.448 | 2.87× |
| Follower action | 2.475 | 2.448 | 2.565 | 3.240 | 7.479 | 6.72× |
| B601-profile observation | 12.713 | 11.360 | 19.427 | 19.566 | 23.643 | 2.93× |
| B601-profile assist action | 3.693 | 3.672 | 3.814 | 4.068 | 15.214 | 5.04× |

Per-repeat mean ranges (before → after, microseconds): observation
20.815–22.127 → 7.304–7.648; action 13.745–20.834 → 2.457–2.502;
B601 observation 34.591–38.157 → 11.282–17.672; B601 assist action
15.461–22.386 → 3.657–3.735. The repeat spread shows host scheduling noise;
ratios are descriptive rather than end-to-end predictions. All 25,000 calls
per scenario completed; fake SDK counts remained identical.

Validation:

- Package tests: **79 passed** (70 original plus 9 new parameterized cases).
- New parity tests compare against the original scalar NumPy expressions at
  limits, adjacent floats, random values, multiple dtypes, signed zero, NaN and
  infinities. Tests also cover output independence, native fallback ownership,
  wrap/clipping parity, B601 transforms, and partial write-error propagation.
- Changed source files pass repository Ruff rules, and all 9 package source
  files pass formatting. The two new test/benchmark files pass isolated Ruff
  `E,F,I,UP` checks and formatting (the repository excludes those directories).
- Pyrefly on both changed source files: **0 errors**.
- Full package source Ruff remains blocked by **11 pre-existing RUF105
  suppression-style findings** in untouched files. No whole-repository hooks
  or hardware checks were run.

Focused source check commands (from the root):

```bash
uv run --no-project /home/mark/projects/geti/physicalai/.venv/bin/ruff check \
  packages/physicalai-stararm-plugin/src/physicalai_stararm_plugin/{_stararm102,stararm102hd}.py
uv run --no-project /home/mark/projects/geti/physicalai/.venv/bin/ruff format --check \
  packages/physicalai-stararm-plugin/src
uv run --no-project /home/mark/projects/geti/physicalai/.venv/bin/pyrefly check \
  packages/physicalai-stararm-plugin/src/physicalai_stararm_plugin/{_stararm102,stararm102hd}.py
uv run --no-project /home/mark/projects/geti/physicalai/.venv/bin/ruff check --isolated \
  --select E,F,I,UP --line-length 120 \
  packages/physicalai-stararm-plugin/tests/unit/test_host_performance.py \
  packages/physicalai-stararm-plugin/benchmarks/host_overhead.py
uv run --no-project /home/mark/projects/geti/physicalai/.venv/bin/ruff format --isolated --check \
  packages/physicalai-stararm-plugin/tests/unit/test_host_performance.py \
  packages/physicalai-stararm-plugin/benchmarks/host_overhead.py
```

The scalar parity test also passes on CPython 3.12.12 / NumPy 1.26.0, using
read-only cached dependencies, without installation:

```bash
PYTHONPATH="$PWD/packages/physicalai-stararm-plugin/src:$PWD/src:/home/mark/.cache/uv/archive-v0/hBX5fJB2yme15j5B0htP1:/home/mark/projects/geti/physicalai/.venv/lib/python3.13/site-packages" \
  uv run --no-project /home/mark/.local/share/uv/python/cpython-3.12-linux-x86_64-gnu/bin/python3.12 \
  -m pytest packages/physicalai-stararm-plugin/tests/unit/test_host_performance.py::test_scalar_clip_matches_numpy -q
```

## Limitations and future hardware validation

This is CPU-only, sequential before/after on a shared host, not an alternating
hardware A/B test. Scheduler noise affects means and tails. No bus, adapter,
servo firmware, UART retries, real reliability misses, CPU utilization or
motion accuracy was measured. Do not claim end-to-end speedups from these
numbers. The existing command duration floor remains unchanged.

Before deploying, inspect the native SDK's protocol implementation and capture
packet lengths and acknowledgements. With a supported arm, an attending
operator and an emergency stop, validate read-only behavior first. Any later
motion experiment needs explicit approval, small interpolated moves from a
known safe supported pose, unchanged units/limits, and a slow return. Alternate
A/B order across repeated fixed-duration windows; report p50/p95/p99/max,
errors, loss flags, retries and accuracy alongside throughput. Verify reconnect,
partial servo loss, persistent jumps, and origin-reset behavior. Batching or
torque caching needs a separate verified protocol/safety design.

## Native sync-angle extension

The vendor protocol supports batching; the published MotorBridge 0.0.4 Python API
does not. The plugin now uses `set_angles` when the installed SDK exposes it and
retains the old addressed path otherwise. The checked-in
[SDK patch/build guide](sdk-patches/README.md) pins the native source revision,
documents firmware requirements, and produces `0.0.4+physicalai.sync1` in an
isolated environment. No shared environment, dependency source, or lockfile was
changed to an unpublished build.

The extension reuses the original native target encoder for every servo record,
then adds the official sync envelope: command 25, subcommand 14, record length 15,
count 7. It preserves float32 scaling/rounding, ID order, 20 ms acceleration and
deceleration fields, zero power field, movement duration, clipping, and gripper
handling. Zero-duration commands retain the corresponding subcommand 13 encoding.
Read behavior, torque, calibration, origin writes and connection setup stay intact.

All records are validated before sending anything. A failed batch propagates its
error without retrying seven addressed writes, since the batch could already be
partly transmitted. Neither the old nor the new angle-write path waits for servo
acknowledgements; no previous write acknowledgement is sacrificed. Broadcast
delivery/rejection remains unverified, so supervised hardware testing is required.

### Native SDK / OS capture

Measurements used the release-built patched wheel on Linux x86_64, CPython 3.13.1,
Rust 1.94.1, and maturin 1.16.0. The inert harness opens only an OS-created PTY;
there is no physical serial port, USB adapter or servo. `strace -yy` captures
`write` and `ioctl` calls on that PTY's slave descriptor. Encoder and system-call
counts are measured; wire serialization times are calculated, not measured.
The seven zero-target addressed packets also match a capture from the public
0.0.4 wheel byte-for-byte, not only the patched wheel's retained addressed path.

| Seven-joint timed action | Addressed | Sync |
| --- | ---: | ---: |
| Captured command packets | 7 | 1 |
| Captured command bytes | 140 | 113 |
| PTY-slave `write` system calls | 7 × 20 bytes | 1 × 113 bytes |
| PTY-slave drain calls (`TCSBRK`) | 7 | 1 |
| Ideal wire serialization, 1 Mbps 8N1 | 1.400 ms | 1.130 ms |

The byte reduction is **19.3%**, saving **270 µs** of ideal command serialization
and six host writes/drains per action. The 7→1 packet count is not a 7× wire-time
gain because all seven target payloads still have to be sent. USB scheduling,
firmware execution, buffer behavior and reply timing were not measured. The
shared-host CPU results from the earlier investigation are separate measurements.

Reproduce the capture with `benchmarks/trace_sync_angles.py --mode addressed` and
`--mode sync` in the patched SDK environment. The build guide includes full setup
and `strace` instructions. The script accepts no physical device path and does
not perform motion. No hardware throughput or accuracy claim follows from it.

### Validation and remaining work

- Native workspace: **26 Rust tests passed**, including three new encoder tests
  covering all command variants, vendor framing, record parity, lengths, IDs,
  nonfinite targets, and duration bounds. Existing reliability tests still pass.
- Release wheel: **18 pseudo-terminal integration cases passed**, including an
  actual plugin → Python wrapper → PyO3 → native controller → OS write. They
  compare addressed and sync target records, exercise malformed batches with no
  bytes emitted, and verify closed-bus errors. Tests are explicitly opt-in via
  `STARARM_TEST_NATIVE_SDK=1` and run separately from SDK-mocked tests.
- Plugin tests add 16 cases for clipping/order/duration, caller ownership, legacy
  SDK compatibility, repeated actions, error propagation without fallback,
  passive leaders, B601 assist conversion, and native-frame hold behavior.
  The full mocked package suite reports **95 passed, 18 skipped**; those 18 native
  cases run and pass separately against the patched release wheel. Pinned Ruff
  lint/format and focused source Pyrefly pass (**0 type errors**).
- The generated patch applies cleanly to the pinned base. The plugin source
  distribution includes the patch and build guide. No wheel is committed.
- Firmware support and real motion remain unvalidated. The vendor limits sync
  commands to brushless magnetic-encoder servos at V316 or newer. The upstream
  SDK's experimental-write warning remains applicable.
- Missing monitor replies can still consume the native SDK's **100 ms overall
  read timeout**. Servo movement durations can still introduce tracking lag.
  Batching writes does not fix either condition, and passive leaders gain no
  write-path benefit. Profile reads/reliability separately before further tuning.
