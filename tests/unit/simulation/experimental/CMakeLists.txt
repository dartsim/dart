# Copyright (c) 2011, The DART development contributors
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
  # Reset DART_ALL_TESTS to avoid accumulating tests across reconfigurations.
  # This cache variable is populated by dart_add_simulation_test() and persists
  # between CMake runs, which would cause the meta target to have stale entries.
  set(DART_ALL_TESTS "" CACHE INTERNAL "List of all tests" FORCE)

  # Register all unit test directories
  dart_experimental_add_unit_test_dir(common ${CMAKE_CURRENT_SOURCE_DIR}/common)
  dart_experimental_add_unit_test_dir(compute ${CMAKE_CURRENT_SOURCE_DIR}/compute)
  dart_experimental_add_unit_test_dir(contact ${CMAKE_CURRENT_SOURCE_DIR}/contact)
  dart_experimental_add_unit_test_dir(
    elasticity ${CMAKE_CURRENT_SOURCE_DIR}/elasticity
  )
  if(TARGET test_continuous_collision_step)
    target_link_libraries(
      test_continuous_collision_step
      PRIVATE ${PROJECT_NAME}-collision-native
    )
  endif()
  dart_experimental_add_unit_test_dir(io ${CMAKE_CURRENT_SOURCE_DIR}/io)
  if(TARGET test_rigid_ipc_fixture)
    target_link_libraries(
      test_rigid_ipc_fixture
      PRIVATE ${PROJECT_NAME}-collision-native
    )
  endif()
  dart_experimental_add_unit_test_dir(deformable_vbd ${CMAKE_CURRENT_SOURCE_DIR}/deformable_vbd)
  dart_experimental_add_unit_test_dir(world ${CMAKE_CURRENT_SOURCE_DIR}/world)
  dart_experimental_add_unit_test_dir(frame ${CMAKE_CURRENT_SOURCE_DIR}/frame)
  dart_experimental_add_unit_test_dir(multibody ${CMAKE_CURRENT_SOURCE_DIR}/multibody)
  dart_experimental_add_unit_test_dir(space ${CMAKE_CURRENT_SOURCE_DIR}/space)

  if(TARGET ${PROJECT_NAME}-simulation-experimental-cuda)
    dart_experimental_add_test(
      test_rigid_body_state_batch_cuda
      ${CMAKE_CURRENT_SOURCE_DIR}/cuda/test_rigid_body_state_batch_cuda.cpp
      LIBRARY ${PROJECT_NAME}-simulation-experimental-cuda
      LABEL simulation-experimental-cuda
    )
    dart_experimental_add_test(
      test_deformable_psd_projection_cuda
      ${CMAKE_CURRENT_SOURCE_DIR}/cuda/test_deformable_psd_projection_cuda.cpp
      LIBRARY ${PROJECT_NAME}-simulation-experimental-cuda
      LABEL simulation-experimental-cuda
    )
    dart_experimental_add_test(
      test_vbd_block_descent_cuda
      ${CMAKE_CURRENT_SOURCE_DIR}/cuda/test_vbd_block_descent_cuda.cpp
      LIBRARY ${PROJECT_NAME}-simulation-experimental-cuda
      LABEL simulation-experimental-cuda
    )
  endif()

  # Create meta target for all unit tests
  # DART_ALL_TESTS now contains only the tests added by this module
  add_custom_target(
    dart_experimental_tests
    DEPENDS ${DART_ALL_TESTS}
    COMMENT "Building all simulation-experimental unit tests"
  )

  list(LENGTH DART_ALL_TESTS num_tests)
  message(STATUS "simulation-experimental: Total ${num_tests} unit test(s)")
else()
  message(
    STATUS
    "simulation-experimental: Unit tests disabled (GTest not found)"
  )
endif()
