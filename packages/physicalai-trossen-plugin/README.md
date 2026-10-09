# PhysicalAI Trossen Plugin

Trossen WidowX AI robot drivers and Physical AI Studio catalog definitions for
[PhysicalAI](https://github.com/openvinotoolkit/physicalai).

## Installation

```bash
uv add physicalai-trossen-plugin
```

The plugin depends on `trossen-arm`. That SDK does not currently publish
wheels for Python 3.14+, so use Python 3.12 or 3.13 when controlling a robot.

## Usage

```python
from physicalai_trossen_plugin import WidowXAI

robot = WidowXAI(ip="192.168.1.100", role="follower")
```

For two arms, construct `BimanualWidowXAI` from left and right `WidowXAI`
instances. Installing this distribution also registers the Trossen entries in
the Physical AI Studio robot catalog.

## URDF assets

This package vendors the Trossen `trossen_arm_description` repository at
revision `21d8b360c211c2ad8a065d8f462cbec0207626e7`. Its source license is
preserved at [`urdf/LICENSE`](urdf/LICENSE).

```python
from physicalai_trossen_plugin import get_urdf_path

urdf_root = get_urdf_path()
single_arm = urdf_root / "urdf/generated/wxai/wxai_follower.urdf"
bimanual = urdf_root / "urdf/generated/stationary_ai.urdf"
```

## Development

```bash
uv sync
uv run pytest packages/physicalai-trossen-plugin/tests
```
