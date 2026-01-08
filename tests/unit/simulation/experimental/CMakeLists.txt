# Copyright (c) 2011-2025, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

# Simulation-experimental Tests (dart::simulation::experimental)
#
# This file only includes test subdirectories. Each subdirectory defines its own tests.
# Uses the dart_experimental_* CMake helpers.

cmake_minimum_required(VERSION 3.22.1)

# Include DART utility functions
include(dart_defs)

# Include dependency finding (makes GTest available)
include(dart_experimental_dependencies)

# Enable CTest for test registration
enable_testing()

# Note: Tests will be added to DART_ALL_TESTS via dart_add_simulation_test().
# We collect them after registration to create the meta target.

#==============================================================================
# Unit Tests
#==============================================================================
# To add a new test module:
#   1. Create a directory under this folder (e.g., common/)
#   2. Add test_*.cpp files to the directory
#   3. Call dart_experimental_add_unit_test_dir() below

message(STATUS "simulation-experimental: Configuring unit tests...")

if(DART_EXPERIMENTAL_BUILD_TESTS)
  # Capture DART_ALL_TESTS before adding our tests
  set(_dart_tests_before ${DART_ALL_TESTS})

  # Register all unit test directories
  dart_experimental_add_unit_test_dir(common ${CMAKE_CURRENT_SOURCE_DIR}/common)
  dart_experimental_add_unit_test_dir(world ${CMAKE_CURRENT_SOURCE_DIR}/world)
  dart_experimental_add_unit_test_dir(frame ${CMAKE_CURRENT_SOURCE_DIR}/frame)
  dart_experimental_add_unit_test_dir(multi_body ${CMAKE_CURRENT_SOURCE_DIR}/multi_body)
  dart_experimental_add_unit_test_dir(space ${CMAKE_CURRENT_SOURCE_DIR}/space)

  # Calculate which tests were added by this module
  set(_dart_experimental_tests ${DART_ALL_TESTS})
  if(_dart_tests_before)
    list(REMOVE_ITEM _dart_experimental_tests ${_dart_tests_before})
  endif()

  # Create meta target for all unit tests
  add_custom_target(dart_experimental_tests
    DEPENDS ${_dart_experimental_tests}
    COMMENT "Building all simulation-experimental unit tests"
  )

  list(LENGTH _dart_experimental_tests num_tests)
  message(STATUS "simulation-experimental: Total ${num_tests} unit test(s)")
else()
  message(STATUS "simulation-experimental: Unit tests disabled (GTest not found)")
endif()
