#
# Copyright (c) 2013-2016, Graphics Lab, Georgia Tech Research Corporation
# Copyright (c) 2013-2016, Humanoid Lab, Georgia Tech Research Corporation
# Copyright (c) 2016, Personal Robotics Lab, Carnegie Mellon University
# All rights reserved.
#
# This file is provided under the following "BSD-style" License:
#   Redistribution and use in source and binary forms, with or
#   without modification, are permitted provided that the following
#   conditions are met:
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
#   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
#   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
#   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
#   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
#   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
#   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#   POSSIBILITY OF SUCH DAMAGE.
#

# GoogleTest setup
include_directories(SYSTEM ${CMAKE_SOURCE_DIR}/unittests/gtest/include)
include_directories(SYSTEM ${CMAKE_SOURCE_DIR}/unittests/gtest)
add_library(gtest STATIC gtest/src/gtest-all.cc)
add_library(gtest_main STATIC gtest/src/gtest_main.cc)
target_link_libraries(gtest_main gtest)
if(NOT WIN32)
  target_link_libraries(gtest pthread)
endif()
set_target_properties(gtest PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

#===============================================================================
# Deinfe helper functions for adding test
#===============================================================================
function(dart_add_unittest target_name)

  dart_property_add(DART_TESTS ${target_name})

  add_executable(${target_name} ${ARGN})
  add_test(${target_name} "${CMAKE_BINARY_DIR}/bin/tests/${target_name}")

  if(MSVC)
    target_link_libraries(
      ${target_name}
      dart
      dart-utils
      dart-utils-urdf
      optimized gtest debug gtestd
    )
  else()
    target_link_libraries(
      ${target_name}
      dart
      dart-utils
      dart-utils-urdf
      gtest
    )
  endif()

  set_target_properties(
    ${target_name} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/tests"
  )

endfunction(dart_add_unittest)

#===============================================================================
# This functiion uses following variables:
# - DART_TEST_OUT_DIR
# and uses following global properties:
# - DART_UNITTESTS
# - DART_${test_type}_TESTS
#
# Usage:
#   add_add_test("comprehensive" testTargetName testSource1.cpp testSource2.cpp)
#===============================================================================
function(dart_add_test test_type target_name) # ARGN for source files

  dart_property_add(DART_${test_type}_TESTS ${target_name})

  add_executable(${target_name} ${ARGN})
  add_test(
    ${target_name}
    "${DART_TEST_OUT_DIR}/${test_type}/${target_name}")

  if(MSVC)
    target_link_libraries(
      ${target_name}
      dart
      dart-utils
      dart-utils-urdf
      optimized gtest debug gtestd
    )
  else()
    target_link_libraries(
      ${target_name}
      dart
      dart-utils
      dart-utils-urdf
      gtest
    )
  endif()

  set_target_properties(
    ${target_name} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${DART_TEST_OUT_DIR}/${test_type}"
  )

endfunction()

#===============================================================================
# Usage:
#   dart_get_tests("comprehensive" compreshensive_tests)
#   foreach(test ${compreshensive_tests})
#     message(STATUS "Test: ${test})
#   endforeach()
#===============================================================================
function(dart_get_tests output_var test_type)
  get_property(var GLOBAL PROPERTY DART_${test_type}_TESTS)
  set(${output_var} ${var} PARENT_SCOPE)
endfunction()

# Set test binary out directory
set(DART_TEST_OUT_DIR "${CMAKE_BINARY_DIR}/bin/tests")

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Compile each test file
file(GLOB tests "test*.cpp")
foreach(test ${tests})
  # Get the name (i.e. bla.cpp => bla)
  get_filename_component(base ${test} NAME_WE)
  dart_add_unittest(${base} ${test})
endforeach(test)

if(TARGET dart-planning)
  target_link_libraries(testNearestNeighbor dart-planning)
endif()

if(TARGET dart-optimizer-ipopt)
  target_link_libraries(testOptimizer dart-optimizer-ipopt)
endif()

if(TARGET dart-optimizer-nlopt)
  target_link_libraries(testOptimizer dart-optimizer-nlopt)
endif()

if(TARGET dart-optimizer-snopt)
  target_link_libraries(testOptimizer dart-optimizer-snopt)
endif()

add_subdirectory(comprehensive)
add_subdirectory(regression)
add_subdirectory(unit)

# Print tests
get_property(tests GLOBAL PROPERTY DART_TESTS)
dart_get_tests(comprehensive_tests "comprehensive")
dart_get_tests(regression_tests "regression")
dart_get_tests(unit_tests "unit")

if(DART_VERBOSE)
  message(STATUS "")
  message(STATUS "[ Tests ]")
  foreach(test ${tests})
    message(STATUS "Adding test: ${test}")
  endforeach()
  foreach(test ${comprehensive_tests})
    message(STATUS "Adding test: comprehensive/${test}")
  endforeach()
  foreach(test ${regression_tests})
    message(STATUS "Adding test: regression/${test}")
  endforeach()
  foreach(test ${unit_tests})
    message(STATUS "Adding test: unit/${test}")
  endforeach()
else()
  list(LENGTH tests tests_length)
  list(LENGTH comprehensive_tests comprehensive_tests_length)
  list(LENGTH regression_tests regression_tests_length)
  list(LENGTH unit_tests unit_tests_length)
  message(STATUS "Adding ${tests_length} tests ("
      "comprehensive: ${comprehensive_tests_length}, "
      "regression: ${regression_tests_length}, "
      "unit: ${unit_tests_length}"
      ")"
  )
endif()

# Add custom target to build all the tests
add_custom_target(
  tests
  DEPENDS ${tests} ${comprehensive_tests} ${regression_tests} ${unit_tests}
)
