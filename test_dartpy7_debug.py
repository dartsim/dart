#!/usr/bin/env python3
"""Test dartpy7 with DEBUG logging enabled."""

import sys

sys.path.insert(0, "build/default/cpp/Release/Release/python")

# First import to set up logging, then enable debug
import dartpy7 as d

print("=== Setting log level to DEBUG ===")
d.set_log_level(d.LogLevel.Debug)
print(
    "\nNow you would see detailed diagnostics if you re-import (not possible in Python)"
)
print("In practice, users can set DART7_LOG_LEVEL environment variable before import\n")

print("Current log level:", d.get_log_level())
print("\n✅ Test complete!")
