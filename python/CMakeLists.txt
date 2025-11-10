# Copyright (c) 2011-2025, The DART development contributors
# All rights reserved.

if(NOT DART_BUILD_DARTPY)
  return()
endif()

# Set up pybind11
if(DART_USE_SYSTEM_PYBIND11)
  if(Python3_EXECUTABLE)
    execute_process(
      COMMAND
        ${Python3_EXECUTABLE}
        "-c"
        "import sysconfig, pathlib; path = pathlib.Path(sysconfig.get_path('platlib')) / 'pybind11' / 'share' / 'cmake' / 'pybind11'; print(path if path.exists() else '')"
      OUTPUT_VARIABLE PYBIND11_CONFIG_DIR
      OUTPUT_STRIP_TRAILING_WHITESPACE
    )
  endif()
  if(PYBIND11_CONFIG_DIR AND EXISTS "${PYBIND11_CONFIG_DIR}")
    set(pybind11_DIR "${PYBIND11_CONFIG_DIR}" CACHE PATH "Path to pybind11 CMake package discovered via Python" FORCE)
  endif()
  find_package(pybind11 CONFIG REQUIRED)
else()
  include(FetchContent)
  FetchContent_Declare(
    pybind11
    GIT_REPOSITORY https://github.com/pybind/pybind11
    GIT_TAG v2.13.6
  )
  FetchContent_MakeAvailable(pybind11)
endif()

if(NOT pybind11_FOUND)
  message(WARNING "Disabling [dartpy] due to missing pybind11 >= 2.2.0.")
  return()
endif()

set(DART_PYTHON_BUILD_DIR "${CMAKE_CURRENT_BINARY_DIR}")
set(DART_DARTPY_BUILD_DIR "${DART_PYTHON_BUILD_DIR}/dartpy")

add_subdirectory(dartpy)
add_subdirectory(tests)
add_subdirectory(examples)
add_subdirectory(tutorials)

message(STATUS "")
message(STATUS "[ dartpy ]")
message(STATUS "- Python3_EXECUTABLE  : ${Python3_EXECUTABLE}")
message(STATUS "- PYTHON_SITE_PACKAGES: ${PYTHON_SITE_PACKAGES}")
message(STATUS "- DARTPY_PYTEST_FOUND : ${DARTPY_PYTEST_FOUND}")
