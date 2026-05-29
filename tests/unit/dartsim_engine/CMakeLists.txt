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

# dartsim_engine is static but depends on the shared dart-simulation-experimental
# library, which the experimental subtree builds into its own output directory.
# dart_build_tests only adds the core `dart` directory to the test's runtime
# search path, and Windows has no RPATH, so without this the test aborts at
# launch with STATUS_DLL_NOT_FOUND (0xc0000135). Mirror dart_add_simulation_test
# by prepending the experimental library directory. Linux/macOS already resolve
# it via the build-tree RPATH; adding it there is harmless.
if(
  TEST UNIT_dartsim_engine
  AND NOT APPLE
  AND TARGET dart-simulation-experimental
)
  if(WIN32)
    set(_dartsim_engine_path_var "PATH")
  else()
    set(_dartsim_engine_path_var "LD_LIBRARY_PATH")
  endif()
  set_property(
    TEST UNIT_dartsim_engine
    APPEND
    PROPERTY
      ENVIRONMENT_MODIFICATION
        "${_dartsim_engine_path_var}=path_list_prepend:$<TARGET_FILE_DIR:dart-simulation-experimental>"
  )
  unset(_dartsim_engine_path_var)
endif()
