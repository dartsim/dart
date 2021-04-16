# Copyright (c) 2011-2021, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/master/LICENSE
#
# This file is provided under the "BSD-style" License

# GoogleTest setup
include_directories(BEFORE SYSTEM ${DART_SOURCE_DIR}/tests/gtest/include)
include_directories(BEFORE SYSTEM ${DART_SOURCE_DIR}/tests/gtest)
add_library(gtest STATIC gtest/src/gtest-all.cc)
add_library(gtest_main STATIC gtest/src/gtest_main.cc)
target_link_libraries(gtest_main gtest)
if(NOT WIN32)
  target_link_libraries(gtest pthread)
endif()
set_target_properties(
  gtest PROPERTIES
  ARCHIVE_OUTPUT_DIRECTORY ${DART_BINARY_DIR}/lib
)

#===============================================================================
# This function uses following global properties:
# - DART_UNITTESTS
# - DART_${test_type}_TESTS
#
# Usage:
#   dart_add_test("unit" test_UnitTestA) # assumed source is test_UnitTestA.cpp
#   dart_add_test("unit" test_UnitTestB test_SourceB1.cpp)
#   dart_add_test("unit" test_UnitTestA test_SourceC1.cpp test_SourceC2.cpp)
#===============================================================================
function(dart_add_test test_type target_name) # ARGN for source files

  dart_property_add(DART_${test_type}_TESTS ${target_name})

  if(${ARGC} GREATER 2)
    set(sources ${ARGN})
  else()
    set(sources "${target_name}.cpp")
  endif()

  add_executable(${target_name} ${sources})
  add_test(${target_name} ${target_name})
  target_link_libraries(${target_name}
    ${PROJECT_NAME}${DART_MAJOR_VERSION}-common
    ${PROJECT_NAME}${DART_MAJOR_VERSION}-math
    ${PROJECT_NAME}${DART_MAJOR_VERSION}-optimization
    ${PROJECT_NAME}${DART_MAJOR_VERSION}-collision
    ${PROJECT_NAME}${DART_MAJOR_VERSION}-dynamics
    ${PROJECT_NAME}${DART_MAJOR_VERSION}-simulation
    ${PROJECT_NAME}${DART_MAJOR_VERSION}-io
    ${PROJECT_NAME}${DART_MAJOR_VERSION}-planning
    gtest
    gtest_main
  )

  dart_format_add(${sources})

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

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_subdirectory(src)

# We categorize tests as:
# - "comprehensive": high level tests to verify the combination of several
#   components are correctly performs together
# - "regression": issue wise tests to verify that the GitHub issues are still
#   fixed even after further changes are made
# - "unit": low level tests for one or few classes and functions to verify that
#   they performs correctly as expected
if(NOT MSVC)
  # Disabled due to limitied disk space in CI image
  # add_subdirectory(comprehensive)
  # add_subdirectory(regression)
endif()
add_subdirectory(unit)

# Print tests
dart_get_tests(comprehensive_tests "comprehensive")
dart_get_tests(regression_tests "regression")
dart_get_tests(unit_tests "TEST_UNIT")

if(DART_VERBOSE)
  message(STATUS "")
  message(STATUS "[ Tests ]")
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
  list(LENGTH comprehensive_tests comprehensive_tests_len)
  list(LENGTH regression_tests regression_tests_len)
  list(LENGTH unit_tests unit_tests_len)
  math(
    EXPR tests_len
    "${comprehensive_tests_len} + ${regression_tests_len} + ${unit_tests_len}"
  )
  message(STATUS "Adding ${tests_len} tests ("
      "comprehensive: ${comprehensive_tests_len}, "
      "regression: ${regression_tests_len}, "
      "unit: ${unit_tests_len}"
      ")"
  )
endif()

# Add custom target to build all the tests as a single target
add_custom_target(
  tests
  DEPENDS ${comprehensive_tests} ${regression_tests} ${unit_tests}
)
