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
  include(FetchContent)
  FetchContent_Declare(
    nanobind
    GIT_REPOSITORY https://github.com/wjakob/nanobind.git
    GIT_TAG v3.1.0
    GIT_SHALLOW TRUE
    GIT_SUBMODULES ext/robin_map
  )
  FetchContent_MakeAvailable(nanobind)
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
