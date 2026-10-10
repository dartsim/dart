# Copyright (c) 2011, The DART development contributors
# All rights reserved.

if(NOT DART_BUILD_DARTPY)
  return()
endif()

set(DART_PYTHON_BUILD_DIR "${CMAKE_CURRENT_BINARY_DIR}")
if(NOT Python_EXECUTABLE AND Python3_EXECUTABLE)
  set(Python_EXECUTABLE "${Python3_EXECUTABLE}")
endif()
if(DART_DARTPY_STABLE_ABI)
  # nanobind silently builds a regular module on older interpreters.
  find_package(
    Python
    3.12
    REQUIRED
    COMPONENTS Interpreter Development.Module Development.SABIModule
  )
else()
  find_package(Python 3.10 REQUIRED COMPONENTS Interpreter Development.Module)
endif()
if(DART_USE_SYSTEM_NANOBIND)
  execute_process(
    COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
    OUTPUT_VARIABLE nanobind_cmake_dir
    OUTPUT_STRIP_TRAILING_WHITESPACE
    COMMAND_ERROR_IS_FATAL ANY
  )
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
add_subdirectory(dartpy_nanobind)
set(DART_DARTPY_BUILD_DIR "$<TARGET_FILE_DIR:dartpy>")
if(WIN32)
  # Use an escaped separator so the full value is passed as a single argument
  set(DART_PYTHONPATH "${DART_DARTPY_BUILD_DIR}\\;${DART_PYTHON_BUILD_DIR}")
else()
  set(DART_PYTHONPATH "${DART_DARTPY_BUILD_DIR}:${DART_PYTHON_BUILD_DIR}")
endif()

add_subdirectory(tests)
add_subdirectory(examples)
add_subdirectory(tutorials)

message(STATUS "")
message(STATUS "[ dartpy ]")
message(STATUS "- Python3_EXECUTABLE  : ${Python3_EXECUTABLE}")
message(STATUS "- PYTHON_SITE_PACKAGES: ${PYTHON_SITE_PACKAGES}")
message(STATUS "- DARTPY_PYTEST_FOUND : ${DARTPY_PYTEST_FOUND}")
