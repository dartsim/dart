# Copyright (c) 2011-2022, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

if(NOT DARTPY_PYTHON_VERSION)
  set(DARTPY_PYTHON_VERSION 3.4 CACHE STRING "Choose the target Python version (e.g., 3.4, 2.7)" FORCE)
endif()

# Find pybind11, including PythonInterp and PythonLibs
# Needs to set PYBIND11_PYTHON_VERSION before finding pybind11
set(PYBIND11_PYTHON_VERSION ${DARTPY_PYTHON_VERSION})
if(WIN32)
  set(PYBIND11_FINDPYTHON TRUE)
endif()
find_package(pybind11 2.2.0 QUIET)
if(NOT pybind11_FOUND)
  message(WARNING "Disabling [dartpy] due to missing pybind11 >= 2.2.0.")
  return()
endif()

execute_process(COMMAND ${PYTHON_EXECUTABLE} -c
  "from distutils.sysconfig import get_python_lib;\
  print(get_python_lib(plat_specific=True))"
  OUTPUT_VARIABLE PYTHON_SITE_PACKAGES
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
if(NOT IS_ABSOLUTE PYTHON_SITE_PACKAGES)
  set(PYTHON_SITE_PACKAGES "${CMAKE_INSTALL_PREFIX}/${PYTHON_SITE_PACKAGES}")
endif()
set(PYTHON_SITE_PACKAGES ${PYTHON_SITE_PACKAGES} PARENT_SCOPE)

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
    dart
    dart-utils
    dart-utils-urdf
    dart-gui
    dart-gui-osg
    ${PYTHON_LIBRARIES}
)
if(TARGET dart-optimizer-nlopt)
  target_link_libraries(${pybind_module} PUBLIC dart-optimizer-nlopt)
endif()
if(TARGET dart-collision-bullet)
  target_link_libraries(${pybind_module} PUBLIC dart-collision-bullet)
endif()
if(TARGET dart-collision-ode)
  target_link_libraries(${pybind_module} PUBLIC dart-collision-ode)
endif()

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
# Install the pybind module to site-packages directory
install(TARGETS ${pybind_module}
  LIBRARY DESTINATION "${PYTHON_SITE_PACKAGES}"
)

list(REMOVE_ITEM dartpy_headers
  ${CMAKE_CURRENT_LIST_DIR}/eigen_geometry_pybind.h
  ${CMAKE_CURRENT_LIST_DIR}/eigen_pybind.h
)
list(REMOVE_ITEM dartpy_sources
  ${CMAKE_CURRENT_LIST_DIR}/eigen_geometry_pybind.cpp
)
dart_format_add(${dartpy_headers} ${dartpy_sources})
