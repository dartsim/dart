# Copyright (c) 2011-2021, The DART development contributors

# ==============================================================================
# Basic Project Setup
# ==============================================================================

# CMake 3.10.2, the default version of Ubuntu 18.04 LTS
cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

# Policy settings
if(POLICY CMP0072)
  cmake_policy(SET CMP0072 NEW)
endif()
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()
if(POLICY CMP0076)
  cmake_policy(SET CMP0076 NEW)
endif()

# DART version
set(DART_VERSION_MAJOR 8)
set(DART_VERSION_MINOR 0)
set(DART_VERSION_PATCH 0)
set(DART_VERSION
    "${DART_VERSION_MAJOR}.${DART_VERSION_MINOR}.${DART_VERSION_PATCH}"
)
project(dart VERSION ${DART_VERSION} LANGUAGES CXX)

# Add <project>/cmake to CMAKE_MODULE_PATH
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

# Include utility modules
include(GNUInstallDirs)
include(DARTFunctions)

# ==============================================================================
# Build Options
# ==============================================================================

# Print the header
message(STATUS "[ Build Tools ]")

message(STATUS "- CMake: ${CMAKE_VERSION}")
message(STATUS "- C++  : ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "")

# Binary options
dart_option(BUILD_SHARED_LIBS "Build shared libraries" ON)
dart_option(DOWNLOAD_DEPENDENT_PACKAGES "Whether to download dependent package" ON)
dart_option(DART_DEBUG "Print detailed build settings" OFF)
dart_option(DART_TRACE "Print detailed build settings" OFF)
dart_option(
  DART_ENABLE_CODECOV "Enable code coverage reporting using Codecov" OFF
)
dart_option(DART_BUILD_TEMPLATE_CODE_FOR_DOUBLE "Build instantiation of template code for double" ON)
dart_option(DART_BUILD_TEMPLATE_CODE_FOR_FLOAT  "Build instantiation of template code for double" OFF)
dart_option(DART_ENABLE_NATIVE_ARCH  "Wheter to use -march=native" ON)
dart_print_options()

if(NOT CMAKE_BUILD_TYPE STREQUAL Debug AND DART_ENABLE_CODECOV)
  message(
    FATAL_ERROR
      "DART_ENABLE_CODECOV=${DART_ENABLE_CODECOV} and CMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} are in conflict. Change one of the option."
  )
endif()

# ==============================================================================
# Find Dependencies
#
# Note: If you add new dependencies here, then please make sure to update
# cmake/dart-config.cmake.in as well.
# ==============================================================================
include(DARTDependencies)

# ==============================================================================
# Compiler settings and compiler specific settings
# ==============================================================================

if(CMAKE_COMPILER_IS_GNUCXX)
  if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9.0.0)
    message("[DEBUG] GCC version is less than 9.0.0")
    set(DART_ENABLE_TBB FALSE)
  else()
    message("[DEBUG] GCC version is equal to or greater than 9.0.0")
    if(TBB_FOUND)
      set(DART_ENABLE_TBB TRUE)
    else()
      set(DART_ENABLE_TBB FALSE)
    endif()
  endif()
endif()

if(DART_ENABLE_NATIVE_ARCH)
  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
  else()
    message(WARNING "DART_ENABLE_NATIVE_ARCH is only supported for GCC and Clang when the detected compiler is ${CMAKE_CXX_COMPILER_ID}.")
  endif()
endif()

# ==============================================================================
# Definitions
# ==============================================================================
# Definitions intended to be used by only docs, examples, and tests, but
# not the library.
set(DART_RESOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/resource/")
add_definitions(-DDART_RESOURCE_DIR="${DART_RESOURCE_DIR}")

# ==============================================================================
# C++ Project Build
# ==============================================================================

dart_set_project(
  PROJECT_BASE_DIR
    ${CMAKE_CURRENT_SOURCE_DIR}
  INCLUDE_SOURCE_BASE_DIR
    ${CMAKE_CURRENT_SOURCE_DIR}/cpp/include
  INCLUDE_BINARY_BASE_DIR
    ${CMAKE_CURRENT_BINARY_DIR}/cpp/include
  SRC_SOURCE_BASE_DIR
    ${CMAKE_CURRENT_SOURCE_DIR}/cpp/src
)

# Coverage
if(DART_ENABLE_CODECOV)
  dart_coverage(
    INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/cpp/include
    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/cpp/src
  )
endif()

# Include the C++ project directory
include(CTest)
enable_testing()
add_subdirectory(cpp)

# ==============================================================================
# Python Binding
# ==============================================================================
add_subdirectory(python)

# ==============================================================================
# Code Formatting
# ==============================================================================
dart_clang_format_setup(VERSION 9)

# ==============================================================================
# "ALL" target
# ==============================================================================
# Add 'ALL' target that builds everything
set(all_target_candidates
  tests_and_run
  benchmarks
  dartpy${DART_VERSION_MAJOR}
  test-dartpy${DART_VERSION_MAJOR}
#  doc
#  coverage_html
)
foreach(target_candidate ${all_target_candidates})
  if(TARGET ${target_candidate})
    list(APPEND all_targets ${target_candidate})
  endif()
endforeach()
foreach(target_candidate ${examples})
  if(TARGET ${target_candidate})
    list(APPEND all_targets ${target_candidate})
  endif()
endforeach()
foreach(target_candidate ${tutorials})
  if(TARGET ${target_candidate})
    list(APPEND all_targets ${target_candidate})
  endif()
endforeach()
add_custom_target(ALL DEPENDS ${all_targets})

# ==============================================================================
# Print build instructions
# ==============================================================================
message(STATUS "")
message(STATUS "[ Build Instructions ]")
message(STATUS "Usage: make [<target> [, <target2>, ...]] [-j<num_core>]")
message(STATUS "Targets:")
message(STATUS "- 'ALL'  : build all the targets and run tests")
message(STATUS "- 'all'  : build core targets (w/o tests)")
if(TARGET tests)
  message(STATUS "- 'tests': build tests")
  message(STATUS "- 'test' : run tests (build 'tests' first)")
endif()
if(TARGET dartpy${DART_VERSION_MAJOR})
  message(STATUS "- 'dartpy${DART_VERSION_MAJOR}'        : build dartpy${DART_VERSION_MAJOR}")
  message(STATUS "- 'dartpy${DART_VERSION_MAJOR}-test'   : build and run dartpy${DART_VERSION_MAJOR} tests")
  message(STATUS "- 'dartpy${DART_VERSION_MAJOR}-install': install dartpy${DART_VERSION_MAJOR}")
endif()
if(TARGET doc)
  message(STATUS "- 'doc'     : generage API doc")
  message(STATUS "- 'doc_view': view generaged API doc in a browser")
endif()
if(TARGET coverage)
  message(STATUS "- 'coverage'     : generage coverage report")
  message(STATUS "- 'coverage_html': generage coverage report in html")
  message(STATUS "- 'coverage_view': view generaged coverage report in a browser")
endif()
message(STATUS "")
