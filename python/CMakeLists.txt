# Copyright (c) 2011, The DART development contributors
# All rights reserved.

if(NOT DART_BUILD_DARTPY)
  return()
endif()

set(DART_PYTHON_BUILD_DIR "${CMAKE_CURRENT_BINARY_DIR}")
# The dartpy package stages at <DART_DARTPY_BUILD_DIR>/dartpy, with the
# extension beside the pure-Python files. Multi-config generators keep one
# root per configuration, so tests import the selected configuration's build.
get_property(dartpy_multi_config GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
if(dartpy_multi_config)
  set(DART_DARTPY_BUILD_DIR "${DART_PYTHON_BUILD_DIR}/$<CONFIG>")
else()
  set(DART_DARTPY_BUILD_DIR "${DART_PYTHON_BUILD_DIR}")
endif()
if(NOT Python_EXECUTABLE AND Python3_EXECUTABLE)
  set(Python_EXECUTABLE "${Python3_EXECUTABLE}")
elseif(
  Python_EXECUTABLE
  AND Python3_EXECUTABLE
  AND NOT "${Python_EXECUTABLE}" STREQUAL "${Python3_EXECUTABLE}"
  AND NOT "${Python3_EXECUTABLE}" STREQUAL "${_DART_PYTHON3_EXECUTABLE}"
)
  message(
    WARNING
    "Python3_EXECUTABLE differs from Python_EXECUTABLE and will be ignored. "
    "Set Python_EXECUTABLE to select the Python interpreter."
  )
endif()
set(dartpy_python_components Interpreter Development.Module)
if(DART_DARTPY_STABLE_ABI)
  if(CMAKE_VERSION VERSION_LESS 3.26)
    message(FATAL_ERROR "DART_DARTPY_STABLE_ABI requires CMake 3.26 or newer")
  endif()
  list(APPEND dartpy_python_components Development.SABIModule)
endif()
# All builds need extension headers, not libpython (unavailable on manylinux).
find_package(Python 3.12 REQUIRED COMPONENTS ${dartpy_python_components})
# FindPython leaves Python_EXECUTABLE a normal variable; cache it so a later
# configure keeps the selection instead of re-seeding from Python3_EXECUTABLE.
set(
  Python_EXECUTABLE
  "${Python_EXECUTABLE}"
  CACHE FILEPATH
  "Python interpreter selected for dartpy"
  FORCE
)
# Share nanobind's selected interpreter with tests, examples, and tutorials.
set(
  Python3_EXECUTABLE
  "${Python_EXECUTABLE}"
  CACHE FILEPATH
  "Python interpreter shared with nanobind"
  FORCE
)
set(Python3_EXECUTABLE "${Python_EXECUTABLE}")
set(
  _DART_PYTHON3_EXECUTABLE
  "${Python_EXECUTABLE}"
  CACHE INTERNAL
  "Last Python3_EXECUTABLE forced by DART"
  FORCE
)
if(DART_USE_SYSTEM_NANOBIND)
  # An explicit nanobind_DIR serves builds whose Python cannot import
  # nanobind, such as cross builds and isolated pip builds.
  if(NOT nanobind_DIR)
    execute_process(
      COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
      OUTPUT_VARIABLE nanobind_cmake_dir
      OUTPUT_STRIP_TRAILING_WHITESPACE
      COMMAND_ERROR_IS_FATAL ANY
    )
  endif()
  find_package(nanobind 3.1 CONFIG REQUIRED HINTS "${nanobind_cmake_dir}")
else()
  if(POLICY CMP0135)
    cmake_policy(SET CMP0135 NEW)
  endif()
  include(FetchContent)
  # A bump updates the URL/hash, 3.1.x check, and internals version below;
  # see docs/onboarding/python-bindings.md, "Nanobind binder". The Eigen probe
  # in scripts/nanobind/eigen_probe repeats the version checks.
  FetchContent_Declare(
    nanobind
    URL
      https://files.pythonhosted.org/packages/c7/1f/e28fae715401e8f57400e72aeaff4fde12046d31828164f34f61f0816da8/nanobind-3.1.0.tar.gz
    URL_HASH
      SHA256=6661e3dc1434d78781ccf7913442d90361dab395c3424efadb63bbb6ddea7ea3
    TLS_VERIFY ON
  )
  FetchContent_MakeAvailable(nanobind)
endif()
# Factory attachment and the patched Eigen caster depend on nanobind internals.
if(
  NOT NB_DIR
  OR NOT EXISTS "${NB_DIR}/include/nanobind/nanobind.h"
  OR NOT EXISTS "${NB_DIR}/src/nb_internals.h"
)
  message(
    FATAL_ERROR
    "dartpy requires a complete nanobind 3.1.x package: NB_DIR must contain "
    "include/nanobind/nanobind.h and src/nb_internals.h. "
    "Install the complete package or set DART_USE_SYSTEM_NANOBIND=OFF."
  )
endif()
file(
  STRINGS "${NB_DIR}/include/nanobind/nanobind.h"
  dartpy_nanobind_version_header
  REGEX "^# *define NB_VERSION_(MAJOR|MINOR|PATCH) "
)
foreach(part MAJOR MINOR PATCH)
  # A failed match keeps the previous CMAKE_MATCH_1, so clear it first.
  set(CMAKE_MATCH_1 "")
  string(
    REGEX MATCH
    "NB_VERSION_${part} ([0-9]+)"
    _
    "${dartpy_nanobind_version_header}"
  )
  set(dartpy_nanobind_${part} "${CMAKE_MATCH_1}")
endforeach()
set(
  dartpy_nanobind_version
  "${dartpy_nanobind_MAJOR}.${dartpy_nanobind_MINOR}.${dartpy_nanobind_PATCH}"
)
if(DART_USE_SYSTEM_NANOBIND)
  set(
    dartpy_nanobind_hint
    "Use a compatible system package or set DART_USE_SYSTEM_NANOBIND=OFF."
  )
else()
  set(
    dartpy_nanobind_hint
    "The fetched nanobind does not match its pin; update the sdist URL, hash and checks together."
  )
endif()
if(NOT dartpy_nanobind_version MATCHES "^3[.]1[.][0-9]+$")
  message(
    FATAL_ERROR
    "dartpy requires nanobind 3.1.x (found ${dartpy_nanobind_version}). "
    "${dartpy_nanobind_hint}"
  )
endif()
file(
  STRINGS "${NB_DIR}/src/nb_internals.h"
  dartpy_nanobind_internals
  REGEX "^# *define NB_INTERNALS_VERSION "
)
if(NOT dartpy_nanobind_internals MATCHES "NB_INTERNALS_VERSION 22([^0-9]|$)")
  message(
    FATAL_ERROR
    "dartpy requires nanobind 3.1.x with NB_INTERNALS_VERSION=22. "
    "Audit python/dartpy/detail/construction.cpp before changing nanobind."
  )
endif()
add_subdirectory(dartpy)
set(DART_PYTHONPATH "${DART_DARTPY_BUILD_DIR}")

add_subdirectory(tests)
add_subdirectory(examples)
add_subdirectory(tutorials)

message(STATUS "")
message(STATUS "[ dartpy ]")
message(STATUS "- Python3_EXECUTABLE  : ${Python3_EXECUTABLE}")
message(STATUS "- PYTHON_SITE_PACKAGES: ${PYTHON_SITE_PACKAGES}")
message(STATUS "- DARTPY_PYTEST_FOUND : ${DARTPY_PYTEST_FOUND}")
