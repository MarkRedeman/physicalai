# Studio SDK boundary performance

## Scope and changes

This package provides the Studio robot catalog SDK, not training, model export,
or the host `physicalai` CLI. Its measurable local costs are imports, catalog
metadata validation, and simulation command preparation. No robot, camera,
Studio server, training job, model download, or simulation process was started.

Following the SO-101 performance handoff, measure the boundary before tuning
Python: the relevant equivalent of a redundant bus transaction is loading the
robot runtime for a helper that does not use it.

- **Load public exports on first access.** A plain SDK import no longer eagerly
  imports catalog, Pydantic schemas, or the robot runtime. A static export table
  preserves every public name and its original object identity. Resolved exports
  are cached; `dir()`, static type checking, and star imports remain supported.
- **Validate duplicate scene IDs in one pass.** Definition construction changes
  from quadratic counting to linear set membership. Duplicate diagnostics stay
  sorted and deduplicated; invalid defaults retain the original scene ordering.
- **Stop scene lookup at the first match.** Each command request no longer builds
  a set of every scene ID. The lookup remains linear in the worst case, with no
  persistent index or invalidation state. Command lists remain fresh copies.

## Environment and revisions

Measured on 2026-11-16:

- Intel Core i9-12900K, Linux 6.8.0-147-generic x86_64, glibc 2.39.
- CPython 3.13.1 (Clang 18.1.8); normal Python source, no native build changes.
- Pydantic 2.14.3, NumPy 2.5.4, pytest 9.2.0.
- Installed Runtime distribution metadata: `physicalai 0.4.1.dev15+g0eb419e`.
  `PYTHONPATH` instead selects this worktree's Runtime source, unchanged at
  `b71bc333bab5f57fc55e375a6f414fa9a7cfa96d`.
- Baseline: that source revision, with the benchmark added in `1f47df0`.
- After: runtime changes through `ff4d239` (`3260b4d` adds lazy exports).
  Later benchmark provenance/formatting and documentation changes do not alter
  the measured runtime code.
- The pre-existing root environment was used read-only. No dependencies or
  lockfiles were changed. Ruff 0.15.12 ran in an isolated `uvx` tool environment.

## Method

Run from the worktree root, with the assigned package ahead of any installed
SDK and worktree Runtime ahead of installed Runtime:

```bash
export PYTHONPATH="$PWD/packages/physicalai-studio-plugin/src:$PWD/src"
PY=/home/mark/projects/geti/physicalai/.venv/bin/python
uv run --no-project "$PY" \
  packages/physicalai-studio-plugin/benchmarks/benchmark_boundary.py \
  --repeats 15 --calls 1000
```

Use the same interpreter and command on the baseline and after revisions. On
other machines, set `PY` to an existing environment containing the dependencies.
The benchmark now emits revision and installed dependency versions as provenance;
the initial baseline harness lacked those fields, recorded above separately.

Cold scenarios run in fresh child interpreters, reversing scenario order on
alternate repeats. The timer covers SDK import plus the selected public symbol
lookup, not interpreter startup or harness stdlib imports. Filesystem caches are
not flushed: these are cold Python-module imports, not cold-disk measurements.

Warm scenarios construct actual `SimulationLaunch` definitions and invoke its
public `argv()` API with a fixed, in-memory builder. Each operation gets 100
warmup calls and 1,000 individually timed measured calls. Scene objects are
created outside the timer. `argv` selects the **first** scene, representing the
benchmark's default selection; later or missing IDs still require a scan.
Two scenes represent a small catalog; 32 and 256 are scaling probes, not a claim
about typical deployed catalogs. The builder returns a two-element command;
the benchmark never executes it.

The baseline and after suites ran sequentially, not alternating revisions, on
an unpinned shared host. There is no across-repeat confidence interval for the
warm samples. Cold p95 and p99 both equal the maximum with only 15 samples;
neither is a reliable production tail estimate. Sub-microsecond differences and
the full-catalog change are particularly sensitive to scheduling/frequency drift.

## Results

All values are measured. Each latency cell lists
**mean / p50 / p95 / p99 / max**. Both suites completed without errors: 60 fresh
import workers and 6,000 measured warm calls per suite. An import or operation
failure aborts the benchmark rather than silently dropping a sample.

### Cold imports (milliseconds)

| Access after importing SDK | Before | After | Modules before → after | NumPy before → after |
| --- | --- | --- | --- | --- |
| SDK only | 153.471 / 162.444 / 170.114 / 170.114 / 170.114 | 0.168 / 0.168 / 0.173 / 0.173 / 0.173 | 485 → 108 | yes → no |
| `shared_robot_name` | 155.031 / 160.195 / 174.985 / 174.985 / 174.985 | 0.218 / 0.214 / 0.269 / 0.269 / 0.269 | 485 → 109 | yes → no |
| `SimulationLaunch` | 156.882 / 163.290 / 171.748 / 171.748 / 171.748 | 31.756 / 30.666 / 40.178 / 40.178 / 40.178 | 485 → 185 | yes → no |
| `RobotCatalogDefinition` | 160.510 / 164.262 / 190.107 / 190.107 / 190.107 | 124.925 / 117.304 / 162.376 / 162.376 / 162.376 | 485 → 469 | yes → yes |

Module counts include the harness's preloaded stdlib. Full catalog access still
imports the robot runtime for the public `BuildRobotCallable` alias, preserving
its concrete runtime typing object. The smaller import paths show the principal
benefit; do not interpret the full-catalog timing difference as a controlled
whole-Studio startup improvement.

### Warm simulation CLI preparation (microseconds)

| Operation | Scenes | Before | After |
| --- | --- | --- | --- |
| Construct/validate definition | 2 | 1.229 / 1.210 / 1.322 / 1.540 / 5.167 | 1.120 / 1.109 / 1.181 / 1.329 / 5.091 |
| Construct/validate definition | 32 | 7.748 / 7.710 / 7.919 / 8.851 / 11.078 | 1.947 / 1.929 / 2.078 / 2.215 / 3.832 |
| Construct/validate definition | 256 | 349.658 / 349.341 / 358.621 / 363.181 / 383.869 | 7.171 / 7.057 / 7.839 / 8.328 / 12.822 |
| Build first scene's argv | 2 | 0.390 / 0.385 / 0.427 / 0.481 / 1.348 | 0.327 / 0.324 / 0.353 / 0.391 / 0.434 |
| Build first scene's argv | 32 | 0.821 / 0.815 / 0.884 / 0.956 / 1.134 | 0.336 / 0.331 / 0.365 / 0.422 / 2.306 |
| Build first scene's argv | 256 | 3.133 / 3.113 / 3.304 / 3.794 / 5.825 | 0.322 / 0.318 / 0.351 / 0.390 / 1.472 |

The construction improvement matters for large/generated catalogs (about 49×
at 256 scenes); the absolute benefit for two scenes is negligible. Successful
first-scene command preparation no longer scales with the catalog size.

## Regression checks

```bash
export PYTHONPATH="$PWD/packages/physicalai-studio-plugin/src:$PWD/src"
PY=/home/mark/projects/geti/physicalai/.venv/bin/python
uv run --no-project "$PY" -m pytest packages/physicalai-studio-plugin/tests -q
uvx --from ruff==0.15.12 ruff check packages/physicalai-studio-plugin/src
uvx --from ruff==0.15.12 ruff format --check packages/physicalai-studio-plugin/src
uvx --from ruff==0.15.12 ruff check \
  --config 'include=["*.py"]' --config 'extend-exclude=[]' --select E,F,I,UP \
  packages/physicalai-studio-plugin/tests/unit \
  packages/physicalai-studio-plugin/benchmarks
uvx --from ruff==0.15.12 ruff format --check \
  --config 'include=["*.py"]' --config 'extend-exclude=[]' \
  packages/physicalai-studio-plugin/tests/unit \
  packages/physicalai-studio-plugin/benchmarks
uv run --no-project /home/mark/projects/geti/physicalai/.venv/bin/pyrefly check \
  -c pyproject.toml --python-interpreter-path "$PY" \
  packages/physicalai-studio-plugin/src \
  packages/physicalai-studio-plugin/tests/unit \
  packages/physicalai-studio-plugin/benchmarks
git diff --check
```

- Package tests: **91 passed**, one existing `PytestCollectionWarning` for
  `TestPayload` in `tests/test_contracts.py`.
- Source Ruff lint/format: passed. New tests and benchmark basic lint/format:
  passed. The root Ruff configuration otherwise excludes test/benchmark files.
- Pyrefly 1.4.1: **0 errors** for source, new tests, and benchmark with the explicit
  interpreter. Checking *all* package tests additionally reports **7 errors** in
  unchanged `tests/test_contracts.py`: one `BaseModel.serial_number` narrowing
  error and six Pydantic JSON-schema metadata typing errors. The contract test
  file is byte-for-byte unchanged from the baseline. These were not suppressed
  or broadened into this performance change.
- Repo-wide hooks/tests and live Studio integration were not run. Hooks include
  lockfile synchronization, and this work is confined to this package.

New regression tests cover export identity/caching, static public names,
`dir()` and star imports, lightweight access with dependency imports blocked,
simulation access without the robot runtime, unknown attribute rejection,
propagated import failures/retry, exact ordered validation diagnostics,
first/middle/last scene lookup, callback validation order, independent replaced
definitions, and fresh command-list ownership. Existing seed, owner-name,
builder, payload, and UI-schema validation tests still pass.

## Safety, limitations, and future work

- The lazy import allowlist is package-owned Python source, never a peer-provided
  module or class path. Unknown names raise `AttributeError`; dependency failures
  propagate without fallback or success caching. Import errors now occur when
  an export is first used instead of on a plain package import. `vars(module)`
  no longer contains unloaded exports; use the supported attributes or `dir()`.
- No changes to trusted configuration construction, artifact paths, payload/UI
  validation, process execution, transport defaults, calibration, torque, or
  hardware ownership. No new dependency or network access path.
- No schema-validation cache: Pydantic metadata and schema hooks can change.
  Caching by class alone risks stale validation at the catalog trust boundary.
- Accessing every export (including star import) still pays for every dependency.
  Direct catalog imports still load NumPy. Redesigning the runtime callable type
  alias to remove that dependency needs separate compatibility review.
- These are import and pure-CPU measurements, not training/export throughput,
  subprocess readiness, inference latency, RSS, or hardware measurements. An
  already-running Studio process with all dependencies loaded saves no cold
  import time from lazy exports.
- Next, measure real catalog registration/schema generation in a representative
  Studio startup trace, then isolate remaining repeated work. If scene lookup
  becomes material, benchmark last/missing IDs before adding an immutable index.
  Use alternating revision runs, CPU affinity, and independent warm repeats for
  stronger confidence intervals. Keep heavyweight training/download benchmarks
  separate from this SDK's hardware-free boundary tests.
