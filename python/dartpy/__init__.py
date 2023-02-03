# Copyright (c) 2011-2023, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License


import os
import sys

__doc__ = "Dynamic Animation and Robotics Toolkit (DART) is a powerful physics engine that simulates the behavior of rigid bodies and articulated skeletons in real-time."
__copyright__ = "Copyright 2011-2023, The DART development contributors"
__license__ = "BSD 2 Clause License"


def import_submodule(module):
    submodule = __import__("dartpy_" + module, fromlist=[""])
    sys.modules["dartpy." + module] = submodule
    setattr(sys.modules["dartpy"], module, submodule)


# Import all the submodules in the current directory
modules_to_import = [
    "common",
    "math",
    "optimization",
    "collision",
    "dynamics",
    "simulation",
    "io",
    "gui",
]
for module in modules_to_import:
    try:
        import_submodule(module)
    except ImportError as e:
        print(f"[WARN] Failed to import {module}: {e}")

from dartpy import *

del import_submodule, modules_to_import, os, sys
