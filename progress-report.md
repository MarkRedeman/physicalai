# Plugin Package Migration Progress Report

## Overview

Four independently versioned Python distributions have been migrated into the
Physical AI Runtime repository as uv workspace packages:

| Package | Previous source | Release baseline | Migration commit |
| --- | --- | --- | --- |
| `physicalai-studio-plugin` | Physical AI Studio `application/plugin` | `0.1.0` | `b6a8182` |
| `physicalai-bimanual-so101-plugin` | PhysicalAI plugins repository | `0.2.3` | `e7b4ba5` |
| `physicalai-rebot-b601-plugin` | PhysicalAI plugins repository | `0.4.3` | `9bd4ea9` |
| `physicalai-lerobot-plugin` | PhysicalAI plugins repository | `0.2.3` | `00bdddd` |

The packages remain separate distributions rather than becoming modules of the
`physicalai` wheel. This preserves their existing installation names, Python
imports, optional hardware dependencies, Studio catalog entry points, and
independent release versions.

## Repository Layout

The root `pyproject.toml` is now a uv workspace with these members:

```text
packages/
  physicalai-studio-plugin/
  physicalai-bimanual-so101-plugin/
  physicalai-rebot-b601-plugin/
  physicalai-lerobot-plugin/
src/
  physicalai/
```

The workspace source mappings ensure local development resolves `physicalai`
and each migrated plugin from this checkout. The lockfile now includes the
dependencies needed by all workspace members, including the substantial
LeRobot/Torch dependency graph.

## Why Packages Rather Than `src`

The root `src/physicalai` directory remains the source tree for the single
`physicalai` runtime distribution. The migrated projects are installed and
released as four separate distributions, so each needs its own package root,
build configuration, version, README, changelog, and optional dependencies.

Placing their import modules directly under the root `src/` would cause the
root build configuration to ship them in the `physicalai` wheel. That would
remove independent installation and release boundaries: installing
`physicalai-bimanual-so101-plugin`, for example, would no longer be distinct
from installing `physicalai`.

The `packages/` layout keeps those boundaries explicit while still enabling
single-checkout development through uv workspaces. It also lets each plugin:

- Depend on hardware-specific libraries without making them runtime
  dependencies for every `physicalai` user.
- Retain its existing PyPI distribution name and Studio catalog entry point.
- Carry URDFs and meshes using package-specific build inclusion rules.
- Receive an independent Release Please version, tag, changelog, build, smoke
  test, and PyPI publication.

This is the same multi-distribution layout used by the original plugins
repository, now managed from the runtime repository rather than a separate
checkout.

## Release Configuration

Release Please now has one component for the runtime and one for each migrated
package. Component tags are enabled, preserving plugin tag names such as:

```text
physicalai-bimanual-so101-plugin-v0.2.4
physicalai-rebot-b601-plugin-v0.4.4
physicalai-lerobot-plugin-v0.2.4
physicalai-studio-plugin-v0.1.1
```

The manifest is seeded with the versions released from the original plugin
repository. This prevents Release Please from resetting the plugin versions
after the repository transfer.

`publish.yml` was converted from a single-package workflow into a component
matrix. It reads every package from the Release Please configuration, then only
builds, smoke-tests, and publishes components for which Release Please created
a release. Each release is checked out at its component tag before building,
which is required for the packages using `hatch-vcs`.

The root runtime package was updated to resolve versions from
`physicalai-v*` tags. This is necessary after enabling component-tagged
releases: root releases now have a component name in their tag too.

## Changes From Original Repositories

### Studio Plugin SDK

The Studio SDK was copied from `physical-ai-studio/application/plugin` into
`packages/physicalai-studio-plugin`.

Changes made during migration:

- Replaced its fixed `0.1.0` version with `hatch-vcs` component versioning.
- Added a fallback version of `0.1.0` for local builds before the first
  component tag exists in this repository.
- Replaced the Studio checkout's git-pinned `physicalai` source with a normal
  `physicalai>=0.1.1` package dependency. The workspace resolves it locally
  during development.
- Added package metadata, changelog, and this repository's GitHub project URLs.
- Shortened the package README to an installation and purpose overview. The
  longer Studio-source README was not copied because portions of its examples
  describe fields no longer present in the SDK API.
- Broadened the return annotations for `robot_field_ui` and
  `robot_payload_ui` to `dict[str, Any]`. This is behavior-preserving and makes
  the helpers compatible with Pydantic's declared `json_schema_extra` type.

### Bimanual SO-101 Plugin

The complete package was copied, including its drivers, tests, example runtime
configuration, calibration examples, URDF, and STL mesh assets.

Changes made during migration:

- Updated project URLs from the former `MarkRedeman/physicalai-plugins`
  repository to `openvinotoolkit/physicalai`.
- Changed the SDK dependency to `physicalai-studio-plugin>=0.1.0`, which is
  resolved as a workspace package locally.
- Registered the package in the root uv workspace and Release Please manifest.
- Applied mechanical Ruff import/noqa fixes.
- Removed two trailing spaces from the migrated URDF.

### reBot B601 Plugin

The complete package was copied, including B601 DM/RS drivers, the Star Arm
102 leader, motion callbacks, catalog definitions, examples, tests, URDFs, and
mesh assets.

Changes made during migration:

- Updated the project URLs and local SDK dependency as above.
- Registered the package with its `0.4.3` Release Please baseline.
- Applied mechanical Ruff import/noqa fixes.
- Replaced one Python 3.12 `type` statement with an equivalent `TypeAlias`
  declaration. The package still requires Python 3.12; the change lets the
  runtime repository's Python 3.11 Pyrefly configuration parse the source.
- Added a narrow `.gitattributes` rule for the CAD-exported reBot files. Those
  assets use CRLF and contain trailing spaces by design, which otherwise causes
  `git diff --check` to report thousands of violations. URDFs are normalized to
  LF; the remaining generated asset whitespace is explicitly exempted.

### LeRobot Plugin

The complete package was copied, including the robot/teleoperator adapters,
dynamic catalog generator, tests, runtime example, and URDF.

Changes made during migration:

- Updated the project URLs and local SDK dependency as above.
- Registered the package with its `0.2.3` Release Please baseline.
- Applied mechanical Ruff import/noqa fixes.
- Regenerated the shared lockfile with LeRobot 0.6.1 and its Torch/CUDA
  transitive dependencies.

## Validation Completed

Each migrated package was built as both an sdist and wheel, and each output was
validated with `twine check`.

| Package | Test result |
| --- | --- |
| Studio plugin SDK | 10 passed |
| Bimanual SO-101 | 36 passed |
| reBot B601 | 86 passed |
| LeRobot | 40 passed |

Ruff checks passed for every migrated source and test tree. Pyrefly checks pass
for the Studio SDK, Bimanual SO-101, and reBot B601 sources. The release
workflow also passes a `zizmor` security audit.

The reBot and LeRobot tests emit only existing deprecation warnings for
`physicalai.config.to_config`; migration did not introduce those warnings.

## Foreseeable Problems And Follow-Up Work

### TestPyPI Does Not Yet Support Components

`.github/workflows/publish-testpypi.yml` still builds, smoke-tests, and
publishes only the root `physicalai` distribution. It must be converted to a
component-aware matrix, matching `publish.yml`, before TestPyPI can validate or
publish the four migrated package distributions.

### Historical Tags Are Not Present Here

The destination repository does not contain the old plugin repository's Git
tags. Release Please has the correct release baselines, so future official
releases will begin at the next appropriate version and create correctly named
component tags. Local `hatch-vcs` builds before that initial tag produce a
development version based on this repository's history, not an exact historical
plugin version.

If preserving complete tag history in this repository is required, import the
four corresponding component tags before the first Release Please release.

### LeRobot Increases Development Footprint

LeRobot depends on Torch and currently resolves CUDA-related packages on this
Linux environment. This significantly increases `uv sync` download time and
disk usage for the shared workspace. The package remains separately installable,
but workspace synchronization now includes its dependency graph.

If this becomes problematic for runtime-only contributors or CI, consider
isolating plugin dependency groups or using a workspace strategy that does not
install every package by default.

### LeRobot Is Not Pyrefly-Clean

The source plugin repository intentionally excluded LeRobot from its Pyrefly
scope. Its dynamic Pydantic model generation and untyped portions of the
upstream LeRobot API currently produce static errors when checked under the
runtime repository's Python 3.11 Pyrefly baseline. Ruff, tests, and packaging
validation pass.

Improving this requires a dedicated typing effort: narrow runtime values from
`object`/`Any`, model the dynamic catalog builder's Pydantic calls, and possibly
add local protocols for upstream LeRobot objects.

### Public Documentation Links

Migrated package READMEs and changelogs retain some links to the former plugin
repository, particularly historical changelog comparison URLs and screenshot
assets. The package project URLs were updated, but documentation and image links
should be reviewed once the source repository transfer is finalized.
