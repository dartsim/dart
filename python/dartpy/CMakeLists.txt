# Copyright (c) 2011-2021, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

if(NOT DARTPY_PYTHON_VERSION)
  set(DARTPY_PYTHON_VERSION 3.4 CACHE STRING "Choose the target Python version (e.g., 3.4, 2.7)" FORCE)
endif()

# Find pybind11
#
# Need to set PYBIND11_PYTHON_VERSION before finding pybind11
#
# No need to explicitly call find_package(PythonLibs) nor find_package(PythonInterp) since the package is found when
# calling find_package(pybind11)
# See: https://pybind11.readthedocs.io/en/stable/faq.html#inconsistent-detection-of-python-version-in-cmake-and-pybind11
set(PYBIND11_PYTHON_VERSION ${DARTPY_PYTHON_VERSION})
find_package(pybind11 2.4.3 QUIET)
if(NOT pybind11_FOUND)
  message(WARNING "DART_BUILD_DARTPY is ON, but failed to find pybind11 >= "
    "2.4.3. Disabling dartpy."
  )
  return()
endif()
if(NOT PYTHONINTERP_FOUND)
  message(WARNING "DART_BUILD_DARTPY is ON, but failed to find PythonInterp. "
    "Disabling dartpy."
  )
  return()
endif()
if(NOT PYTHONLIBS_FOUND)
  message(WARNING "DART_BUILD_DARTPY is ON, but failed to find PythonLibs. "
    "Disabling dartpy."
  )
  return()
endif()
execute_process(COMMAND ${PYTHON_EXECUTABLE} -c
  "from distutils.sysconfig import get_python_lib;\
  print(get_python_lib(plat_specific=True, prefix=''))"
  OUTPUT_VARIABLE PYTHON_SITE_PACKAGES
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
if(NOT IS_ABSOLUTE PYTHON_SITE_PACKAGES)
  set(PYTHON_SITE_PACKAGES "${CMAKE_INSTALL_PREFIX}/${PYTHON_SITE_PACKAGES}")
endif()

file(GLOB_RECURSE dartpy_headers "*.h" "*.hpp")
file(GLOB_RECURSE dartpy_sources "*.cpp")

# Python binding module name
set(pybind_module dartpy)

# Build a Python extension module:
# pybind11_add_module(<name> [MODULE | SHARED] [EXCLUDE_FROM_ALL]
#                     [NO_EXTRAS] [SYSTEM] [THIN_LTO] source1 [source2 ...])
#
pybind11_add_module(${pybind_module}
  MODULE
  ${dartpy_headers}
  ${dartpy_sources}
  EXCLUDE_FROM_ALL
)

target_include_directories(${pybind_module}
  SYSTEM PUBLIC
    ${PYTHON_INCLUDE_DIRS}
    ${pybind11_INCLUDE_DIRS}
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(${pybind_module}
  PUBLIC
    ${PROJECT_NAME}${DART_MAJOR_VERSION}-common
    ${PROJECT_NAME}${DART_MAJOR_VERSION}-math
    ${PROJECT_NAME}${DART_MAJOR_VERSION}-optimization
    ${PROJECT_NAME}${DART_MAJOR_VERSION}-collision
    ${PROJECT_NAME}${DART_MAJOR_VERSION}-dynamics
    ${PROJECT_NAME}${DART_MAJOR_VERSION}-simulation
    ${PROJECT_NAME}${DART_MAJOR_VERSION}-io
    ${PROJECT_NAME}${DART_MAJOR_VERSION}-gui
    ${PYTHON_LIBRARIES}
)

# Remove debug postfix for dartpy
set_target_properties(${pybind_module} PROPERTIES DEBUG_POSTFIX "")

# Get the path to the bind module
set(PYBIND_MODULE $<TARGET_FILE:${pybind_module}>)

# Custom target to install (copy) the bind module. This target may require
# `sudo` if the destination is a system directory.
set(install_comment "Installing ${pybind_module}...")
if(BUILD_SHARED_LIBS)
  string(CONCAT install_comment
    "${install_comment}\n"
    "NOTE: ${pybind_module} is built against the DART's shared libraries. "
    "Install the shared libraries to be able to import ${pybind_module}."
  )
endif()
add_custom_target(install-${pybind_module}
  COMMENT "${install_comment}"
  COMMAND ${CMAKE_COMMAND} -E copy ${PYBIND_MODULE} ${PYTHON_SITE_PACKAGES}
  DEPENDS ${install_dartpy_deps}
)

list(REMOVE_ITEM dartpy_headers
  ${CMAKE_CURRENT_LIST_DIR}/eigen_geometry_pybind.h
  ${CMAKE_CURRENT_LIST_DIR}/eigen_pybind.h
)
list(REMOVE_ITEM dartpy_sources
  ${CMAKE_CURRENT_LIST_DIR}/eigen_geometry_pybind.cpp
)
dart_format_add(${dartpy_headers} ${dartpy_sources})
