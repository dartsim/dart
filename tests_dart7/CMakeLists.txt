# Copyright (c) 2011-2025, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

# DART 7.0 Tests
#
# This file only includes test subdirectories. Each subdirectory defines its own tests.
# Since tests_dart7/ is a sibling to dart7/ (not a subdirectory), we need to include
# dependencies and helper functions here.

cmake_minimum_required(VERSION 3.22.1)

# Include dart7 utility functions (must come first to define dart7_find_package)
include(dart7_defs)

# Include dependency finding (makes GTest available)
include(dart7_dependencies)

# Enable CTest for test registration
enable_testing()

# Initialize test list (will be populated by dart7_add_test in subdirectories)
set(DART7_ALL_TESTS "" CACHE INTERNAL "List of all DART7 tests")

#==============================================================================
# Unit Tests
#==============================================================================
# To add a new test module:
#   1. Create a directory under unit/ (e.g., unit/math/)
#   2. Add test_*.cpp files to the directory
#   3. Call dart7_add_unit_test_dir() below

message(STATUS "dart7: Configuring unit tests...")

if(DART7_BUILD_TESTS)
  # Register all unit test directories
  dart7_add_unit_test_dir(common ${CMAKE_CURRENT_SOURCE_DIR}/unit/common)
  dart7_add_unit_test_dir(world ${CMAKE_CURRENT_SOURCE_DIR}/unit/world)
  dart7_add_unit_test_dir(frame ${CMAKE_CURRENT_SOURCE_DIR}/unit/frame)
  dart7_add_unit_test_dir(multibody ${CMAKE_CURRENT_SOURCE_DIR}/unit/multibody)
  dart7_add_unit_test_dir(space ${CMAKE_CURRENT_SOURCE_DIR}/unit/space)

  # Create meta target for all unit tests
  add_custom_target(dart7_tests
    DEPENDS ${DART7_ALL_TESTS}
    COMMENT "Building all DART7 unit tests"
  )

  list(LENGTH DART7_ALL_TESTS num_tests)
  message(STATUS "dart7: Total ${num_tests} unit test(s)")
else()
  message(STATUS "dart7: Unit tests disabled (GTest not found)")
endif()
