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
  FetchContent_Declare(googletest GIT_REPOSITORY https://github.com/google/googletest GIT_TAG v1.15.2)
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  set(gtest_disable_pthreads ON CACHE BOOL "" FORCE)
  set(BUILD_GMOCK ON CACHE BOOL "" FORCE)
  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)
endif()

#===============================================================================
# This function uses following global properties:
# - DART_UNITTESTS
# - DART_${test_type}_TESTS
#
# Usage:
#   dart_add_test("unit" test_unit_test_aNone) # assumed source is test_unit_test_a.cpp
#   dart_add_test("unit" test_unit_test_bNone test_source_b1.cpp)
#   dart_add_test("unit" test_unit_test_aNone test_source_c1.cpp test_source_c2.cpp)
#===============================================================================
function(dart_add_test test_type target_name) # ARGN for source files
  dart_property_add(DART_${test_type}_TESTS ${target_name})

  if(${ARGC} GREATER 2)
    set(sources ${ARGN})
  else()
    set(sources "${target_name}.cpp")
  endif()

  add_executable(${target_name} ${sources})
  add_test(NAME ${target_name} COMMAND $<TARGET_FILE:${target_name}>)
  target_link_libraries(${target_name} dart GTest::gtest GTest::gtest_main)

  # Set up library paths for tests to find shared libraries at runtime.
  # - Windows: Use PATH (required for DLL loading)
  # - Linux: Use LD_LIBRARY_PATH (required for .so loading)
  # - macOS: Set BUILD_RPATH explicitly to ensure the test finds the correct
  #          libraries. DYLD_LIBRARY_PATH is stripped by SIP so we cannot use it.
  if(APPLE)
    # Explicitly set BUILD_RPATH to include the dart library directory.
    # This ensures tests find the correct Debug/Release libraries.
    set_target_properties(${target_name} PROPERTIES BUILD_RPATH "$<TARGET_FILE_DIR:dart>")
  else()
    # Windows and Linux need environment variables for library paths
    if(WIN32)
      set(_dart_lib_path_var "PATH")
    else()
      set(_dart_lib_path_var "LD_LIBRARY_PATH")
    endif()

    if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.22")
      set(_dart_env_mods "${_dart_lib_path_var}=path_list_prepend:$<TARGET_FILE_DIR:dart>")
      list(APPEND _dart_env_mods "${_dart_lib_path_var}=path_list_prepend:$<TARGET_FILE_DIR:GTest::gtest>")
      set_tests_properties(${target_name} PROPERTIES ENVIRONMENT_MODIFICATION "${_dart_env_mods}")
    else()
      if(WIN32)
        # Escape the semicolon so CMake does not treat it as a list separator.
        set(_dart_test_path_sep "\\;")
      else()
        set(_dart_test_path_sep ":")
      endif()
      set(_dart_lib_paths "$<TARGET_FILE_DIR:dart>${_dart_test_path_sep}$<TARGET_FILE_DIR:GTest::gtest>")
      set_property(
        TEST ${target_name}
        PROPERTY ENVIRONMENT "${_dart_lib_path_var}=${_dart_lib_paths}${_dart_test_path_sep}$ENV{${_dart_lib_path_var}}"
      )
      unset(_dart_test_path_sep)
    endif()
    unset(_dart_lib_path_var)
  endif()
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

# Build baseline libraries for testing (ODE reference implementation)
set(_dart_prev_use_global_output_dirs ${DART_USE_GLOBAL_OUTPUT_DIRS})
set(DART_USE_GLOBAL_OUTPUT_DIRS OFF)
add_subdirectory(baseline)
if(DEFINED _dart_prev_use_global_output_dirs)
  set(DART_USE_GLOBAL_OUTPUT_DIRS ${_dart_prev_use_global_output_dirs})
else()
  unset(DART_USE_GLOBAL_OUTPUT_DIRS)
endif()

if(CMAKE_CONFIGURATION_TYPES)
  foreach(_dart_cfg ${CMAKE_CONFIGURATION_TYPES})
    string(TOUPPER "${_dart_cfg}" _dart_cfg_upper)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${_dart_cfg_upper} "${DART_BINARY_DIR}/bin/${_dart_cfg}")
  endforeach()
else()
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${DART_BINARY_DIR}/bin")
endif()

# Common test utilities shared across test types
add_subdirectory(common)

# We categorize tests as:
# - "integration": high level tests to verify the combination of several
#   components are correctly performs together
# - "unit": low level tests for one or few classes and functions to verify that
#   they performs correctly as expected
add_subdirectory(integration)
add_subdirectory(unit)

# Print tests
dart_get_tests(integration_tests "integration")
dart_get_tests(unit_tests "unit")

if(DART_VERBOSE)
  message(STATUS "")
  message(STATUS "[ Tests ]")
  foreach(test ${integration_tests})
    message(STATUS "Adding test: integration/${test}")
  endforeach()
  foreach(test ${unit_tests})
    message(STATUS "Adding test: unit/${test}")
  endforeach()
else()
  list(LENGTH integration_tests integration_tests_len)
  list(LENGTH unit_tests unit_tests_len)
  math(EXPR tests_len "${integration_tests_len} + ${unit_tests_len}")
  message(STATUS "Adding ${tests_len} tests (" "integration: ${integration_tests_len}, " "unit: ${unit_tests_len}" ")")
endif()

# Add custom target to build all the tests as a single target
add_custom_target(tests DEPENDS ${integration_tests} ${unit_tests})

# Add custom target to build all the tests and run the tests
if(MSVC)
  add_custom_target(
    tests_and_run
    COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure -C ${CMAKE_BUILD_TYPE}
    DEPENDS ${integration_tests} ${unit_tests}
  )
else()
  add_custom_target(tests_and_run COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure DEPENDS ${integration_tests} ${unit_tests})
endif()

dart_format_add(
  helpers/gtest_utils.hpp
  helpers/common_helpers.hpp
  helpers/dynamics_helpers.hpp
)

add_subdirectory(benchmark)
