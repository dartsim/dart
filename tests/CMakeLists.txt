#
# Copyright (c) 2011, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
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

# Set up GoogleTest
if(DART_USE_SYSTEM_GOOGLETEST)
  find_package(GTest MODULE REQUIRED)
else()
  include(FetchContent)
  # Extract with current timestamps on CMake 3.24+ so archive updates rebuild.
  set(dart_googletest_timestamp)
  if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.24)
    set(dart_googletest_timestamp DOWNLOAD_EXTRACT_TIMESTAMP FALSE)
  endif()
  FetchContent_Declare(
    googletest
    URL
      https://github.com/google/googletest/releases/download/v1.17.0/googletest-1.17.0.tar.gz
    URL_HASH
      SHA256=65fab701d9829d38cb77c14acdc431d2108bfdbf8979e40eb8ae567edf10b27c
    TLS_VERIFY ON
    ${dart_googletest_timestamp}
  )
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  set(gtest_disable_pthreads ON CACHE BOOL "" FORCE)
  set(BUILD_GMOCK ON CACHE BOOL "" FORCE)
  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)
  dart_check_fetchcontent(googletest DART_USE_SYSTEM_GOOGLETEST)
endif()

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
  # Put fetched GTest headers before system headers inherited from dart.
  target_link_libraries(${target_name} GTest::gtest GTest::gtest_main dart)

  dart_format_add(${sources})
endfunction()

#===============================================================================
# Usage:
#   dart_get_tests("integration" compreshensive_tests)
#   foreach(test ${compreshensive_tests})
#     message(STATUS "Test: ${test})
#   endforeach()
#===============================================================================
function(dart_get_tests output_var test_type)
  get_property(var GLOBAL PROPERTY DART_${test_type}_TESTS)
  set(${output_var} ${var} PARENT_SCOPE)
endfunction()

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Source and generated dart/ headers have the installed include layout. Do not
# inherit dart's dependency includes, compile definitions, or C++17 requirement.
get_target_property(
  contact_surface_eigen_includes
  Eigen3::Eigen
  INTERFACE_INCLUDE_DIRECTORIES
)
if(MSVC)
  # Use the C++17 dialect and template parsing mode DART exports to MSVC users.
  set(
    contact_surface_probe_flags
    /std:c++17
    ${DART_MSVC_CXX_PARSING_OPTIONS}
    /Zs
  )
  set(contact_surface_include_flag /I)
else()
  set(contact_surface_probe_flags -std=c++14 -fsyntax-only)
  set(contact_surface_include_flag -I)
endif()
foreach(
  include_dir
  ${DART_SOURCE_DIR}
  ${DART_BINARY_DIR}
  ${contact_surface_eigen_includes}
)
  list(
    APPEND contact_surface_probe_flags
    "${contact_surface_include_flag}${include_dir}"
  )
endforeach()
add_test(
  NAME ContactSurfaceProbe
  COMMAND
    "${CMAKE_CXX_COMPILER}" ${contact_surface_probe_flags}
    "${PROJECT_SOURCE_DIR}/tests/cmake/ContactSurfaceProbe.cpp"
)
dart_format_add(cmake/ContactSurfaceProbe.cpp)

# We categorize tests as:
# - "integration": high level tests to verify the combination of several
#   components are correctly performs together
# - "regression": issue wise tests to verify that the GitHub issues are still
#   fixed even after further changes are made
# - "unit": low level tests for one or few classes and functions to verify that
#   they performs correctly as expected
add_subdirectory(baseline)
add_subdirectory(integration)
add_subdirectory(regression)
add_subdirectory(unit)

add_test(
  NAME test_DARTConfigVersion
  COMMAND
    "${CMAKE_COMMAND}" "-DSOURCE_DIR=${PROJECT_SOURCE_DIR}"
    "-DTEST_BINARY_DIR=${CMAKE_CURRENT_BINARY_DIR}/cmake/DARTConfigVersion"
    "-DDART_VERSION=${DART_VERSION}"
    "-DCMAKE_SIZEOF_VOID_P=${CMAKE_SIZEOF_VOID_P}" -P
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/test_DARTConfigVersion.cmake"
)

add_test(
  NAME test_HeadersComponent
  COMMAND
    "${CMAKE_COMMAND}" "-DDART_BINARY_DIR=${DART_BINARY_DIR}"
    "-DTEST_BINARY_DIR=${CMAKE_CURRENT_BINARY_DIR}/cmake/HeadersComponent"
    "-DTEST_CONFIG=$<CONFIG>" -P
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/test_HeadersComponent.cmake"
)

add_test(
  NAME test_StandaloneHeaders
  COMMAND
    "${CMAKE_COMMAND}" "-DDART_BINARY_DIR=${DART_BINARY_DIR}"
    "-DTEST_BINARY_DIR=${CMAKE_CURRENT_BINARY_DIR}/cmake/StandaloneHeaders"
    "-DTEST_CONFIG=$<CONFIG>" -P
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/test_StandaloneHeaders.cmake"
)
# It compiles on every core, so CTest must not run other tests beside it.
set_tests_properties(test_StandaloneHeaders PROPERTIES RUN_SERIAL TRUE)

if(TARGET dart-collision-ode)
  add_test(
    NAME test_DARTConfigODE
    COMMAND
      "${CMAKE_COMMAND}" "-DDART_BINARY_DIR=${DART_BINARY_DIR}"
      "-DTEST_BINARY_DIR=${CMAKE_CURRENT_BINARY_DIR}/cmake/DARTConfigODE"
      "-DTEST_CONFIG=$<CONFIG>" "-DTEST_PREFIX_PATH=${CMAKE_PREFIX_PATH}" -P
      "${CMAKE_CURRENT_SOURCE_DIR}/cmake/test_DARTConfigODE.cmake"
  )
endif()

# Print tests
dart_get_tests(integration_tests "integration")
dart_get_tests(regression_tests "regression")
dart_get_tests(unit_tests "unit")

if(DART_VERBOSE)
  message(STATUS "")
  message(STATUS "[ Tests ]")
  foreach(test ${integration_tests})
    message(STATUS "Adding test: integration/${test}")
  endforeach()
  foreach(test ${regression_tests})
    message(STATUS "Adding test: regression/${test}")
  endforeach()
  foreach(test ${unit_tests})
    message(STATUS "Adding test: unit/${test}")
  endforeach()
else()
  list(LENGTH integration_tests integration_tests_len)
  list(LENGTH regression_tests regression_tests_len)
  list(LENGTH unit_tests unit_tests_len)
  math(
    EXPR
    tests_len
    "${integration_tests_len} + ${regression_tests_len} + ${unit_tests_len}"
  )
  message(
    STATUS
    "Adding ${tests_len} tests ("
    "integration: ${integration_tests_len}, "
    "regression: ${regression_tests_len}, "
    "unit: ${unit_tests_len}"
    ")"
  )
endif()

# Add custom target to build all the tests as a single target
add_custom_target(
  tests
  DEPENDS ${integration_tests} ${regression_tests} ${unit_tests}
)

# Add custom target to build all the tests and run the tests
add_custom_target(
  tests_and_run
  # Rendering tests need a graphics device; `pixi run test-gui` runs them.
  COMMAND
    ${CMAKE_COMMAND} "-DDART_CTEST_COMMAND=${CMAKE_CTEST_COMMAND}"
    "-DDART_CTEST_CONFIGURATION=$<CONFIG>"
    "-DDART_CTEST_EXCLUDE_LABEL=gui-device" -P
    "${PROJECT_SOURCE_DIR}/cmake/DARTRunCTest.cmake"
  DEPENDS ${integration_tests} ${regression_tests} ${unit_tests}
  WORKING_DIRECTORY "${DART_BINARY_DIR}"
)

dart_format_add(GTestUtils.hpp cmake/gui_consumer/main.cpp)

add_subdirectory(benchmark)
