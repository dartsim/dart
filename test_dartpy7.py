#!/usr/bin/env python3
"""Test dartpy7 import and basic functionality."""

import sys

sys.path.insert(0, "build/default/cpp/Release/Release/python")

print("=== Test 1: Import dartpy7 (INFO level by default) ===")
import dartpy7 as d

print("\n=== Test 2: Version API ===")
print(f"✅ version(): {d.version()}")
print(f"✅ __version__: {d.__version__}")
print(f"✅ version_major(): {d.version_major()}")
print(f"✅ version_minor(): {d.version_minor()}")
print(f"✅ version_patch(): {d.version_patch()}")

print("\n=== Test 3: World class ===")
world = d.World()
print(f"✅ World created: {world}")

print("\n=== Test 4: Logging API ===")
print(f"✅ Current log level: {d.get_log_level()}")
print(
    f"✅ Available log levels: {[level.name for level in d.LogLevel.__members__.values()]}"
)

print("\n✅ All tests passed!")
