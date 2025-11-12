# Copyright (c) 2011-2025, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

# DART 8.0 Tests
#
# This file only includes test subdirectories. Each subdirectory defines its own tests.
# Since tests_dart8/ is a sibling to dart8/ (not a subdirectory), we need to include
# dependencies and helper functions here.

cmake_minimum_required(VERSION 3.22.1)

# Include dart8 utility functions (must come first to define dart8_find_package)
include(dart8_defs)

# Include dependency finding (makes GTest available)
include(dart8_dependencies)

# Enable CTest for test registration
enable_testing()

# Initialize test list (will be populated by dart8_add_test in subdirectories)
set(DART8_ALL_TESTS "" CACHE INTERNAL "List of all DART8 tests")

#==============================================================================
# Unit Tests
#==============================================================================
# To add a new test module:
#   1. Create a directory under unit/ (e.g., unit/math/)
#   2. Add test_*.cpp files to the directory
#   3. Call dart8_add_unit_test_dir() below

message(STATUS "dart8: Configuring unit tests...")

if(DART8_BUILD_TESTS)
  # Register all unit test directories
  dart8_add_unit_test_dir(common ${CMAKE_CURRENT_SOURCE_DIR}/unit/common)
  dart8_add_unit_test_dir(world ${CMAKE_CURRENT_SOURCE_DIR}/unit/world)
  dart8_add_unit_test_dir(frame ${CMAKE_CURRENT_SOURCE_DIR}/unit/frame)
  dart8_add_unit_test_dir(multibody ${CMAKE_CURRENT_SOURCE_DIR}/unit/multibody)
  dart8_add_unit_test_dir(space ${CMAKE_CURRENT_SOURCE_DIR}/unit/space)

  # Create meta target for all unit tests
  add_custom_target(dart8_tests
    DEPENDS ${DART8_ALL_TESTS}
    COMMENT "Building all DART8 unit tests"
  )

  list(LENGTH DART8_ALL_TESTS num_tests)
  message(STATUS "dart8: Total ${num_tests} unit test(s)")
else()
  message(STATUS "dart8: Unit tests disabled (GTest not found)")
endif()
