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
set(DART_PYTHON_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
if(WIN32)
  set(DART_PYTHON_PATHS "${DART_PYTHON_BUILD_DIR};${DART_PYTHON_SOURCE_DIR}")
else()
  set(DART_PYTHON_PATHS "${DART_PYTHON_BUILD_DIR}:${DART_PYTHON_SOURCE_DIR}")
endif()
file(MAKE_DIRECTORY "${DART_DARTPY_BUILD_DIR}")
file(WRITE "${DART_DARTPY_BUILD_DIR}/__init__.py"
  "# Auto-generated so python/tests can import the build tree.\n"
  "import importlib.machinery\n"
  "import importlib.util\n"
  "import pathlib\n"
  "import sys\n"
  "\n"
  "_pkg_dir = pathlib.Path(__file__).resolve().parent\n"
  "_ext_path = None\n"
  "for _suffix in importlib.machinery.EXTENSION_SUFFIXES:\n"
  "    _candidate = _pkg_dir / f\"dartpy{_suffix}\"\n"
  "    if _candidate.exists():\n"
  "        _ext_path = _candidate\n"
  "        break\n"
  "\n"
  "if _ext_path is None:\n"
  "    raise ImportError(f\"Cannot find dartpy extension next to {_pkg_dir}\")\n"
  "\n"
  "_loader = importlib.machinery.ExtensionFileLoader(__name__, str(_ext_path))\n"
  "_spec = importlib.util.spec_from_loader(__name__, _loader)\n"
  "_module = importlib.util.module_from_spec(_spec)\n"
  "_loader.exec_module(_module)\n"
  "\n"
  "sys.modules[__name__] = _module\n"
  "globals().update(_module.__dict__)\n")

add_subdirectory(dartpy)
add_subdirectory(tests)
add_subdirectory(examples)
add_subdirectory(tutorials)

message(STATUS "")
message(STATUS "[ dartpy ]")
message(STATUS "- Python3_EXECUTABLE  : ${Python3_EXECUTABLE}")
message(STATUS "- PYTHON_SITE_PACKAGES: ${PYTHON_SITE_PACKAGES}")
message(STATUS "- DARTPY_PYTEST_FOUND : ${DARTPY_PYTEST_FOUND}")
