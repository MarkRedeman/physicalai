# Physical AI Studio Plugin

Types, protocols, and utilities for building robot catalog plugins for Physical AI Studio.

Studio discovers robot catalog plugins through the
`physicalai.studio.catalog_plugins` entry-point group. Plugins use this package
to define their payloads, robot builders, visualization assets, and hardware
probes.

## Installation

```bash
uv add physicalai-studio-plugin
```

Requires Python 3.12 or newer. The package depends on `physicalai` and
`pydantic`.
