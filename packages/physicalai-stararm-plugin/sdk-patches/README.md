# Native FashionStar sync-angle support

This reviewed patch extends MotorBridge rather than replacing its serial reader,
filters, timeouts, locking, or reliability behavior. It adds `FashionStarServo.set_angles`
through the Python wrapper, PyO3 binding, native controller, and packet encoder.
It is a local SDK build, not a published upstream release.

## Protocol and source provenance

- SDK repository: <https://github.com/motorbridge/motorbridge-smart-servo>.
- Required base commit: `4f8895a00d9f786ada258ca6b12c46f65206e0ed`.
- Local SDK commit: `5f35a27` (`perf(fashionstar): batch angle targets in a native sync command`).
- Wheel version: `0.0.4+physicalai.sync1`, distinct from the unpatched `0.0.4`.
- Vendor reference: [`servodevelop/servo-uart-rs485-sdk` at `f728776`](https://github.com/servodevelop/servo-uart-rs485-sdk/tree/f72877669af0a4dc3501bb98df291cacc0a8592e).
  See `python/example/sync_mode/sync_mode.py`,
  `micro-python/src/uservo.py::send_sync_multiturnanglebyinterval`, and
  `python/README.md`'s supported-command table.

The vendor documents sync commands for brushless magnetic-encoder servos with
**V316 or newer firmware**. Verify the actual servo model/firmware before deployment.
Existing sync-monitor support is not a hardware test of sync motion support.
The upstream SDK still labels write control experimental; this patch does not
remove that qualification or establish physical motion reliability.

For timed multi-turn targets, the native encoder wraps the exact existing
per-servo records with command **25**, subcommand **14**, record length **15**,
and the servo count. All target, duration, acceleration/deceleration, and power
fields retain the existing addressed encoder's values and rounding. Duration
zero retains subcommand 13, matching the original API. Single-turn variants
are supported as well. A timed multi-turn batch supports up to 16 targets within
the protocol's one-byte parameter-length field.

All records are validated before any write. Empty/mismatched lists, duplicate or
invalid IDs, nonfinite targets, oversized packets, and invalid durations fail
without sending any command. The controller performs one `write_all`/`flush`,
with no input-buffer clearing, read, acknowledgement wait, or retry. Addressed
angle writes were already fire-and-forget; batching does not remove a previous
per-servo acknowledgement. A failed or partial batch must never trigger an
automatic fallback to individual motion commands.

## Build a reviewed release wheel

Run from the Physical AI Runtime repository root. These commands fetch the pinned
source and apply the checked-in patch, not an unreviewed branch. They build code;
they do not open a serial device. Use an unused `SDK_SRC` directory.

```bash
RUNTIME_ROOT="$PWD"
PLUGIN_ROOT="$RUNTIME_ROOT/packages/physicalai-stararm-plugin"
SDK_SRC=/tmp/opencode/stararm-motorbridge-build
SHARED="$RUNTIME_ROOT/.venv"

git clone https://github.com/motorbridge/motorbridge-smart-servo.git "$SDK_SRC"
git -C "$SDK_SRC" checkout --detach 4f8895a00d9f786ada258ca6b12c46f65206e0ed
git -C "$SDK_SRC" apply --check "$PLUGIN_ROOT/sdk-patches/motorbridge-smart-servo-sync-angles.patch"
git -C "$SDK_SRC" apply "$PLUGIN_ROOT/sdk-patches/motorbridge-smart-servo-sync-angles.patch"
cargo test --locked --workspace --manifest-path "$SDK_SRC/Cargo.toml"

(
  cd "$SDK_SRC/bindings/python"
  uv tool run --from maturin==1.16.0 maturin build --release --locked \
    --interpreter "$SHARED/bin/python" --out "$SDK_SRC/dist"
)
```

Rust/Cargo and a supported native compiler are required. The tested Linux wheel
targets x86_64, CPython's stable ABI for Python 3.9+, and glibc 2.34+. Build wheels
on the deployment platform when its architecture or libc differs. Do not copy
this Linux wheel to macOS, Windows, or older glibc deployments.

## Validate without hardware

The opt-in integration suite opens only pseudo-terminals created by `os.openpty()`;
it accepts no physical device path. Keep it separate from the SDK-mocked package
suite. The existing environment remains read-only:

```bash
CHECK_ENV=/tmp/opencode/stararm-sdk-validation
uv venv --python "$SHARED/bin/python" "$CHECK_ENV"
uv pip install --python "$CHECK_ENV/bin/python" "$SDK_SRC"/dist/*.whl
SHARED_SITE=$(uv run --no-project "$SHARED/bin/python" -c 'import sysconfig; print(sysconfig.get_path("purelib"))')
export PYTHONPATH="$PLUGIN_ROOT/src:$RUNTIME_ROOT/packages/physicalai-studio-plugin/src:$RUNTIME_ROOT/src:$SHARED_SITE"

STARARM_TEST_NATIVE_SDK=1 uv run --no-project "$CHECK_ENV/bin/python" -m pytest \
  "$PLUGIN_ROOT/tests/integration/test_native_sync_angles.py" -q

uv run --no-project "$CHECK_ENV/bin/python" \
  "$PLUGIN_ROOT/benchmarks/trace_sync_angles.py" --mode addressed
uv run --no-project "$CHECK_ENV/bin/python" \
  "$PLUGIN_ROOT/benchmarks/trace_sync_angles.py" --mode sync
```

These commands capture **140 bytes / 7 packets** versus **113 bytes / 1 packet**.
On Linux, prefix the interpreter with `strace -yy -e trace=write,ioctl -o trace.log`
to inspect the actual PTY-slave writes and drain calls. These are native SDK/OS
checks, not a measurement of USB adapter or servo latency.

## Deploy deliberately

After review, install the built wheel into the environment that owns the robot,
not the shared development environment. The plugin automatically selects
`set_angles` when the installed SDK exposes it; its connection log reports
`sync angle writes: True`. The public 0.0.4 wheel retains the original seven-write
path. No dependency or lockfile is silently redirected to a local build.

An ordinary `uv sync` can restore the lockfile's public SDK wheel. Persist the
reviewed wheel as a source override in the deployment project's configuration
if the optimization must survive synchronization. This repository does not
change its dependency source to an unpublished local build.

Before enabling motion, confirm firmware support with an attending operator and
the robot's existing safety procedure. Use supported poses and small interpolated
targets, preserve durations and limits, and check tracking, serial errors, reply
freshness, gripper behavior, and torque release. Never run unattended. The patch
does not address the SDK's 100 ms overall read timeout when monitor responses
are missing, nor the servo's commanded movement duration.
