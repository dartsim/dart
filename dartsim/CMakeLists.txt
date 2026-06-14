# Copyright (c) 2011, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License.

# dartsim: a standalone robotics multi-physics GUI simulator on the experimental
# World API. It is distributed as a runtime executable (via package managers),
# NOT as a library — so nothing here is installed or exported as a public
# dart:: component, and runtime dependencies are kept minimal. The library
# distribution lives under dart/. See docs/design/dartsim_gui_simulator.md.
#
# Layout:
#   engine/  headless editor engine (no GUI dependency)
#   ui/      panel layer bridging the engine to the dart-gui visualization lib
#   app/     thin executable entry point

# The headless engine has no GUI dependency, so it always builds (and is
# unit-tested by tests/unit/dartsim_engine) whenever the DART 7 World is
# available.
add_subdirectory(engine)

if(DART_BUILD_GUI AND TARGET dart-gui)
  add_subdirectory(ui)
  add_subdirectory(app)
else()
  message(
    STATUS
    "Skipping dartsim GUI (DART_BUILD_GUI == OFF); engine library still built"
  )
endif()
