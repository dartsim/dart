# Copyright (c) 2011, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License.

# Unit tests for the headless dartsim editor engine (dartsim/engine).
# The library target lives outside the tests tree; it is added before tests/
# in the top-level CMakeLists.txt.
dart_build_tests(
  TYPE unit
  TARGET_PREFIX UNIT
  LINK_LIBRARIES dartsim_engine
  SOURCES test_dartsim_engine.cpp
)
