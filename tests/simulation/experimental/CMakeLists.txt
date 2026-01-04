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

# Include dart_experimental utility functions (must come first to define dart_experimental_find_package)
include(dart_experimental_defs)

# Include dependency finding (makes GTest available)
include(dart_experimental_dependencies)

# Enable CTest for test registration
enable_testing()

# Initialize test list (will be populated by dart_experimental_add_test in subdirectories)
set(DART_EXPERIMENTAL_ALL_TESTS "" CACHE INTERNAL "List of all simulation-experimental tests")

#==============================================================================
# Unit Tests
#==============================================================================
# To add a new test module:
#   1. Create a directory under unit/ (e.g., unit/math/)
#   2. Add test_*.cpp files to the directory
#   3. Call dart_experimental_add_unit_test_dir() below

message(STATUS "simulation-experimental: Configuring unit tests...")

if(DART_EXPERIMENTAL_BUILD_TESTS)
  # Register all unit test directories
  dart_experimental_add_unit_test_dir(common ${CMAKE_CURRENT_SOURCE_DIR}/unit/common)
  dart_experimental_add_unit_test_dir(world ${CMAKE_CURRENT_SOURCE_DIR}/unit/world)
  dart_experimental_add_unit_test_dir(frame ${CMAKE_CURRENT_SOURCE_DIR}/unit/frame)
  dart_experimental_add_unit_test_dir(multibody ${CMAKE_CURRENT_SOURCE_DIR}/unit/multibody)
  dart_experimental_add_unit_test_dir(space ${CMAKE_CURRENT_SOURCE_DIR}/unit/space)

  # Create meta target for all unit tests
  add_custom_target(dart_experimental_tests
    DEPENDS ${DART_EXPERIMENTAL_ALL_TESTS}
    COMMENT "Building all simulation-experimental unit tests"
  )

  list(LENGTH DART_EXPERIMENTAL_ALL_TESTS num_tests)
  message(STATUS "simulation-experimental: Total ${num_tests} unit test(s)")
else()
  message(STATUS "simulation-experimental: Unit tests disabled (GTest not found)")
endif()
