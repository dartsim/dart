# Copyright (c) 2011, The DART development contributors
# All rights reserved.

if(NOT DART_BUILD_DARTPY)
  return()
endif()

set(DART_PYTHON_BUILD_DIR "${CMAKE_CURRENT_BINARY_DIR}")
if(DART_DARTPY_BINDER STREQUAL "nanobind")
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
else()
  # Set up pybind11
  set(
    PYBIND11_FINDPYTHON
    ON
    CACHE BOOL
    "Use CMake FindPython mode for pybind11"
    FORCE
  )
  if(DART_USE_SYSTEM_PYBIND11)
    if(Python3_EXECUTABLE)
      execute_process(
        COMMAND
          ${Python3_EXECUTABLE} "-c"
          "import sysconfig, pathlib; path = pathlib.Path(sysconfig.get_path('platlib')) / 'pybind11' / 'share' / 'cmake' / 'pybind11'; print(path if path.exists() else '')"
        OUTPUT_VARIABLE PYBIND11_CONFIG_DIR
        OUTPUT_STRIP_TRAILING_WHITESPACE
      )
    endif()
    if(PYBIND11_CONFIG_DIR AND EXISTS "${PYBIND11_CONFIG_DIR}")
      set(
        pybind11_DIR
        "${PYBIND11_CONFIG_DIR}"
        CACHE PATH
        "Path to pybind11 CMake package discovered via Python"
        FORCE
      )
    endif()
    find_package(pybind11 3.0.3 CONFIG REQUIRED)
  else()
    include(FetchContent)
    FetchContent_Declare(
      pybind11
      GIT_REPOSITORY https://github.com/pybind/pybind11
      GIT_TAG v3.0.3
    )
    FetchContent_MakeAvailable(pybind11)
  endif()

  if(NOT pybind11_FOUND)
    message(WARNING "Disabling [dartpy] due to missing pybind11 >= 3.0.3.")
    return()
  endif()

  add_subdirectory(dartpy)
endif()
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
