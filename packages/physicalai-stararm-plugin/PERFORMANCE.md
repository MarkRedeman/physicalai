# Star Arm host performance

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
