# Copyright (c) The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/master/LICENSE
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

#==============================================================================
# Dependency Finding Helper
#==============================================================================

# Helper function to find and track dependencies (similar to find_package)
# Usage: dart_experimental_find_package(
#          NAME <name>
#          PACKAGE <package_name>
#          [REQUIRED]
#          [QUIET]
#          [VERSION <version>]
#          [COMPONENTS <components...>]
#          [SET_VAR <variable_name>]
#        )
#
# This function wraps find_package() and tracks found/missing dependencies
# for compact summary reporting. It's used by dart_experimental_dependencies.cmake.
#
function(dart_experimental_find_package)
  cmake_parse_arguments(
    ARG
    "REQUIRED;QUIET"
    "NAME;PACKAGE;VERSION;SET_VAR"
    "COMPONENTS"
    ${ARGN}
  )

  # Build find_package arguments
  set(find_args ${ARG_PACKAGE})
  if(ARG_VERSION)
    list(APPEND find_args ${ARG_VERSION})
  endif()
  if(ARG_COMPONENTS)
    list(APPEND find_args COMPONENTS ${ARG_COMPONENTS})
  endif()
  if(ARG_REQUIRED)
    list(APPEND find_args REQUIRED)
  elseif(ARG_QUIET)
    list(APPEND find_args QUIET)
  endif()

  # Find the package
  find_package(${find_args})

  # Check if found (using standard CMake convention)
  set(found_var "${ARG_PACKAGE}_FOUND")

  # Track dependency status
  if(${found_var})
    set(DART_EXPERIMENTAL_DEPS_FOUND ${DART_EXPERIMENTAL_DEPS_FOUND} ${ARG_NAME} CACHE INTERNAL "List of found dependencies")
    if(DART_EXPERIMENTAL_VERBOSE)
      # Get version if available
      set(version_var "${ARG_PACKAGE}_VERSION")
      if(DEFINED ${version_var})
        message(STATUS "  ✓ ${ARG_NAME} ${${version_var}}")
      else()
        message(STATUS "  ✓ ${ARG_NAME}")
      endif()
    endif()

    # Set optional variable to TRUE if requested
    if(ARG_SET_VAR)
      set(${ARG_SET_VAR} TRUE CACHE BOOL "${ARG_NAME} available" FORCE)
    endif()
  else()
    set(DART_EXPERIMENTAL_DEPS_MISSING ${DART_EXPERIMENTAL_DEPS_MISSING} ${ARG_NAME} CACHE INTERNAL "List of missing dependencies")
    if(DART_EXPERIMENTAL_VERBOSE)
      message(STATUS "  ✗ ${ARG_NAME} not found")
    endif()

    # Set optional variable to FALSE if requested
    if(ARG_SET_VAR)
      set(${ARG_SET_VAR} FALSE CACHE BOOL "${ARG_NAME} available" FORCE)
    endif()
  endif()
endfunction()

#==============================================================================
# Library Creation Helpers
#==============================================================================

# Utility function to create a simulation-experimental library with standard settings
#
# Usage:
#   dart_experimental_add_library(
#     NAME <library_name>
#     SOURCES <source_files...>
#     HEADERS <header_files...>
#     [INCLUDE_DIRS_PUBLIC <dirs...>]
#     [INCLUDE_DIRS_PRIVATE <dirs...>]
#     [LINK_LIBS_PUBLIC <libs...>]
#     [LINK_LIBS_PRIVATE <libs...>]
#     [VERSION <version>]
#   )
#
function(dart_experimental_add_library)
  # Parse arguments
  set(options "")
  set(oneValueArgs
    NAME
    VERSION
  )
  set(multiValueArgs
    SOURCES
    HEADERS
    INCLUDE_DIRS_PUBLIC
    INCLUDE_DIRS_PRIVATE
    LINK_LIBS_PUBLIC
    LINK_LIBS_PRIVATE
  )

  cmake_parse_arguments(
    ARG
    "${options}"
    "${oneValueArgs}"
    "${multiValueArgs}"
    ${ARGN}
  )

  # Validate required arguments
  if(NOT ARG_NAME)
    message(FATAL_ERROR "dart_experimental_add_library: NAME is required")
  endif()

  if(NOT ARG_SOURCES)
    message(FATAL_ERROR "dart_experimental_add_library: SOURCES is required")
  endif()

  # Expand glob patterns in SOURCES
  set(EXPANDED_SOURCES)
  foreach(source ${ARG_SOURCES})
    if(source MATCHES ".*[*?].*")
      # Contains glob pattern
      file(GLOB matched_files "${source}")
      list(APPEND EXPANDED_SOURCES ${matched_files})
    else()
      # Regular file
      list(APPEND EXPANDED_SOURCES ${source})
    endif()
  endforeach()
  set(ARG_SOURCES ${EXPANDED_SOURCES})

  # Expand glob patterns in HEADERS
  set(EXPANDED_HEADERS)
  foreach(header ${ARG_HEADERS})
    if(header MATCHES ".*[*?].*")
      # Contains glob pattern
      file(GLOB matched_files "${header}")
      list(APPEND EXPANDED_HEADERS ${matched_files})
    else()
      # Regular file
      list(APPEND EXPANDED_HEADERS ${header})
    endif()
  endforeach()
  set(ARG_HEADERS ${EXPANDED_HEADERS})

  # Use PROJECT_VERSION if VERSION not specified
  if(NOT ARG_VERSION)
    if(PROJECT_VERSION)
      set(ARG_VERSION ${PROJECT_VERSION})
    else()
      message(FATAL_ERROR "dart_experimental_add_library: VERSION is required (or set project VERSION)")
    endif()
  endif()

  # Extract version components for SOVERSION
  string(REGEX MATCH "^([0-9]+)\\.([0-9]+)\\.([0-9]+)" _ ${ARG_VERSION})
  set(VERSION_MAJOR ${CMAKE_MATCH_1})
  set(VERSION_MINOR ${CMAKE_MATCH_2})
  set(VERSION_PATCH ${CMAKE_MATCH_3})

  # Create library
  add_library(${ARG_NAME} SHARED ${ARG_SOURCES})

  # Set C++20 standard (modern CMake approach)
  target_compile_features(${ARG_NAME} PUBLIC cxx_std_20)

  # Add compile definition for source directory (used for relative paths in logging)
  target_compile_definitions(${ARG_NAME}
    PUBLIC DART_EXPERIMENTAL_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
  )

  # Include directories - default to parent of current source dir
  if(NOT ARG_INCLUDE_DIRS_PUBLIC)
    set(ARG_INCLUDE_DIRS_PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..)
  endif()

  target_include_directories(${ARG_NAME}
    PUBLIC
      $<BUILD_INTERFACE:${ARG_INCLUDE_DIRS_PUBLIC}>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/..>
      $<INSTALL_INTERFACE:include>
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}
      ${ARG_INCLUDE_DIRS_PRIVATE}
  )

  # Link dependencies (passed from call site)
  if(ARG_LINK_LIBS_PUBLIC)
    target_link_libraries(${ARG_NAME} PUBLIC ${ARG_LINK_LIBS_PUBLIC})
  endif()

  if(ARG_LINK_LIBS_PRIVATE)
    target_link_libraries(${ARG_NAME} PRIVATE ${ARG_LINK_LIBS_PRIVATE})
  endif()

  # Set library properties
  set_target_properties(${ARG_NAME} PROPERTIES
    VERSION ${ARG_VERSION}
    SOVERSION ${VERSION_MAJOR}
    OUTPUT_NAME ${ARG_NAME}
  )

  # Install rules
  install(TARGETS ${ARG_NAME}
    EXPORT ${ARG_NAME}Targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
  )

  # Install headers
  if(ARG_HEADERS)
    install(FILES ${ARG_HEADERS}
      DESTINATION include/${ARG_NAME}
    )
  endif()

  message(STATUS "${ARG_NAME}: Configured simulation-experimental library")
endfunction()

# Utility function to create a simulation-experimental Python binding module with nanobind
#
# Usage:
#   dart_experimental_add_python_module(
#     NAME <module_name>
#     SOURCES <source_files...>
#     [LINK_LIBS <libs...>]
#     [VERSION <version>]
#   )
#
function(dart_experimental_add_python_module)
  # Parse arguments
  set(options "")
  set(oneValueArgs
    NAME
    VERSION
  )
  set(multiValueArgs
    SOURCES
    LINK_LIBS
  )

  cmake_parse_arguments(
    ARG
    "${options}"
    "${oneValueArgs}"
    "${multiValueArgs}"
    ${ARGN}
  )

  # Validate required arguments
  if(NOT ARG_NAME)
    message(FATAL_ERROR "dart_experimental_add_python_module: NAME is required")
  endif()

  if(NOT ARG_SOURCES)
    message(FATAL_ERROR "dart_experimental_add_python_module: SOURCES is required")
  endif()

  # Expand glob patterns in SOURCES
  set(EXPANDED_SOURCES)
  foreach(source ${ARG_SOURCES})
    if(source MATCHES ".*[*?].*")
      # Contains glob pattern
      file(GLOB matched_files "${source}")
      list(APPEND EXPANDED_SOURCES ${matched_files})
    else()
      # Regular file
      list(APPEND EXPANDED_SOURCES ${source})
    endif()
  endforeach()
  set(ARG_SOURCES ${EXPANDED_SOURCES})

  # Use PROJECT_VERSION if VERSION not specified
  if(NOT ARG_VERSION)
    if(PROJECT_VERSION)
      set(ARG_VERSION ${PROJECT_VERSION})
    else()
      message(FATAL_ERROR "dart_experimental_add_python_module: VERSION is required (or set project VERSION)")
    endif()
  endif()

  # Find Python
  find_package(Python COMPONENTS Interpreter Development REQUIRED)

  # Try to find nanobind using Python
  execute_process(
    COMMAND ${Python_EXECUTABLE} -m nanobind --cmake_dir
    OUTPUT_VARIABLE nanobind_ROOT
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
    RESULT_VARIABLE nanobind_NOTFOUND
  )

  if(nanobind_NOTFOUND)
    message(STATUS "nanobind not found. Skipping ${ARG_NAME}.")
    message(STATUS "To enable ${ARG_NAME}, install nanobind: pip install nanobind")
    return()
  endif()

  # Find nanobind
  list(APPEND CMAKE_PREFIX_PATH "${nanobind_ROOT}")
  find_package(nanobind CONFIG REQUIRED)

  message(STATUS "Building ${ARG_NAME} with nanobind at ${nanobind_ROOT}")

  # Build Python extension module using nanobind
  nanobind_add_module(${ARG_NAME}
    NB_STATIC  # Build a static extension module
    ${ARG_SOURCES}
  )

  # Include directories
  target_include_directories(${ARG_NAME}
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}
      ${CMAKE_SOURCE_DIR}
      ${CMAKE_BINARY_DIR}
  )

  # Link libraries
  if(ARG_LINK_LIBS)
    target_link_libraries(${ARG_NAME} PRIVATE ${ARG_LINK_LIBS})
  endif()

  # Set version info
  target_compile_definitions(${ARG_NAME}
    PRIVATE ${ARG_NAME}_VERSION_INFO="${ARG_VERSION}-dev"
  )

  # Remove debug postfix
  set_target_properties(${ARG_NAME} PROPERTIES DEBUG_POSTFIX "")

  # Set output directory to match expected PYTHONPATH
  set_target_properties(${ARG_NAME} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/python"
    LIBRARY_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/Debug/python"
    LIBRARY_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/Release/python"
  )

  message(STATUS "${ARG_NAME}: Configured simulation-experimental Python bindings")
endfunction()

# Utility function to add a simulation-experimental test
# Automatically adds to CTest with simulation-experimental label and global test list
#
# Prerequisites: GTest must be found via dart_experimental_dependencies.cmake
#
# Usage:
#   dart_experimental_add_test(test_name path/to/test.cpp)
#
function(dart_experimental_add_test TEST_NAME TEST_PATH)
  if(NOT DART_EXPERIMENTAL_BUILD_TESTS)
    message(WARNING "dart_experimental_add_test called but DART_EXPERIMENTAL_BUILD_TESTS is FALSE")
    return()
  endif()

  add_executable(${TEST_NAME} ${TEST_PATH})
  target_link_libraries(${TEST_NAME}
    PRIVATE
      dart-simulation-experimental
      GTest::gtest
      GTest::gtest_main
  )

  # Add to CTest with simulation-experimental label for easy filtering
  add_test(NAME ${TEST_NAME} COMMAND $<TARGET_FILE:${TEST_NAME}>)
  set_tests_properties(
    ${TEST_NAME}
    PROPERTIES
      LABELS "simulation-experimental"
  )

  # Set up library paths for tests to find shared libraries at runtime.
  # - Windows: Use PATH (required for DLL loading)
  # - Linux: Use LD_LIBRARY_PATH (required for .so loading)
  # - macOS: Skip - CMake's RPATH handling works correctly, and DYLD_LIBRARY_PATH
  #          is stripped by System Integrity Protection (SIP) causing test hangs
  if(NOT APPLE)
    if(WIN32)
      set(_dart_lib_path_var "PATH")
    else()
      set(_dart_lib_path_var "LD_LIBRARY_PATH")
    endif()

    if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.22")
      set(_dart_env_mods "${_dart_lib_path_var}=path_list_prepend:$<TARGET_FILE_DIR:dart-simulation-experimental>")
      # Also include main dart library path (some tests link against it transitively)
      if(TARGET dart)
        list(APPEND _dart_env_mods "${_dart_lib_path_var}=path_list_prepend:$<TARGET_FILE_DIR:dart>")
      endif()
      list(APPEND _dart_env_mods "${_dart_lib_path_var}=path_list_prepend:$<TARGET_FILE_DIR:GTest::gtest>")
      set_tests_properties(
        ${TEST_NAME}
        PROPERTIES
          ENVIRONMENT_MODIFICATION "${_dart_env_mods}"
      )
    else()
      if(WIN32)
        # Escape the semicolon so CMake does not treat it as a list separator.
        set(_dart_experimental_test_path_sep "\\;")
      else()
        set(_dart_experimental_test_path_sep ":")
      endif()
      set(_dart_lib_paths "$<TARGET_FILE_DIR:dart-simulation-experimental>")
      # Also include main dart library path
      if(TARGET dart)
        set(_dart_lib_paths "${_dart_lib_paths}${_dart_experimental_test_path_sep}$<TARGET_FILE_DIR:dart>")
      endif()
      set(_dart_lib_paths "${_dart_lib_paths}${_dart_experimental_test_path_sep}$<TARGET_FILE_DIR:GTest::gtest>")
      set_property(
        TEST ${TEST_NAME}
        PROPERTY
          ENVIRONMENT
            "${_dart_lib_path_var}=${_dart_lib_paths}${_dart_experimental_test_path_sep}$ENV{${_dart_lib_path_var}}"
      )
      unset(_dart_experimental_test_path_sep)
    endif()
    unset(_dart_lib_path_var)
  endif()

  # Set target properties
  set_target_properties(${TEST_NAME} PROPERTIES
    FOLDER "simulation-experimental/tests"
  )

  # Add to global test list
  set(DART_EXPERIMENTAL_ALL_TESTS ${DART_EXPERIMENTAL_ALL_TESTS} ${TEST_NAME} CACHE INTERNAL "List of all simulation-experimental tests")
endfunction()

# Utility function to register all unit tests in a directory
# Automatically discovers test_*.cpp files and creates meta target
#
# Prerequisites: GTest must be found via dart_experimental_dependencies.cmake
#
# Usage:
#   dart_experimental_add_unit_test_dir(common ${CMAKE_CURRENT_SOURCE_DIR}/unit/common)
#   dart_experimental_add_unit_test_dir(world ${CMAKE_CURRENT_SOURCE_DIR}/unit/world)
#
# This will:
#   - Find all test_*.cpp files in the directory
#   - Register each as a test using dart_experimental_add_test()
#   - Create a meta target dart_experimental_tests_<module_name>
#   - Report number of tests found
#
function(dart_experimental_add_unit_test_dir MODULE_NAME MODULE_DIR)
  if(NOT DART_EXPERIMENTAL_BUILD_TESTS)
    return()
  endif()

  # Find all test files in the directory
  file(GLOB test_files "${MODULE_DIR}/test_*.cpp")

  if(NOT test_files)
    message(STATUS "  - ${MODULE_NAME}: no tests found")
    return()
  endif()

  # Register each test
  set(module_test_targets)
  foreach(test_file ${test_files})
    # Extract test name from filename (remove .cpp extension)
    get_filename_component(test_name ${test_file} NAME_WE)

    # Register the test
    dart_experimental_add_test(${test_name} ${test_file})

    # Add to this module's test list
    list(APPEND module_test_targets ${test_name})
  endforeach()

  # Create meta target for this module
  add_custom_target(dart_experimental_tests_${MODULE_NAME}
    DEPENDS ${module_test_targets}
    COMMENT "Building simulation-experimental ${MODULE_NAME} tests"
  )

  # Report
  list(LENGTH test_files num_tests)
  message(STATUS "  - ${MODULE_NAME}: ${num_tests} test(s)")
endfunction()

# Utility function to add a simulation-experimental benchmark
#
# Prerequisites: benchmark must be found via dart_experimental_dependencies.cmake
#
# Usage:
#   dart_experimental_add_benchmark(bm_name path/to/benchmark.cpp)
#
function(dart_experimental_add_benchmark BENCHMARK_NAME BENCHMARK_PATH)
  if(NOT DART_EXPERIMENTAL_BUILD_BENCHMARKS)
    message(WARNING "dart_experimental_add_benchmark called but DART_EXPERIMENTAL_BUILD_BENCHMARKS is FALSE")
    return()
  endif()

  add_executable(${BENCHMARK_NAME} ${BENCHMARK_PATH})
  target_link_libraries(${BENCHMARK_NAME}
    PRIVATE
      dart-simulation-experimental
      benchmark::benchmark
      benchmark::benchmark_main
  )

  # Set benchmark properties
  set_target_properties(${BENCHMARK_NAME} PROPERTIES
    FOLDER "simulation-experimental/benchmarks"
  )
endfunction()
