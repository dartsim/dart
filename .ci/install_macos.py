#!/usr/bin/env python3

# Copyright (c) 2011-2023, The DART development contributors
# All rights reserved.

import os
import subprocess
import sys

# Run brew update
subprocess.run(["brew", "update"], stdout=subprocess.DEVNULL, check=True)

# Install dependencies with brew bundle
subprocess.run(["brew", "bundle"], check=True)

# Install OpenSceneGraph
if os.environ.get("INSTALL_OSG_HEAD") == "OFF":
    subprocess.run(["brew", "install", "open-scene-graph"], check=True)
else:
    # Install master branch until 3.7.0 is released (see: https://github.com/dartsim/dart/issues/1439)
    subprocess.run(["brew", "install", "open-scene-graph", "--HEAD"], check=True)

# Install numpy, pytest, and requests with pip for the default Python3 version
py_version = f"{sys.version_info.major}.{sys.version_info.minor}"
subprocess.run(
    [f"pip{py_version}", "install", "-U", "numpy", "pytest", "requests"], check=True
)
