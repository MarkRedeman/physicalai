# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

"""Allow mocked driver tests to run without the optional native SDK."""

import importlib.util
import sys
from types import ModuleType

if importlib.util.find_spec("trossen_arm") is None:
    # Driver tests replace this import-only placeholder with their own mocks.
    sys.modules["trossen_arm"] = ModuleType("trossen_arm")
