# Copyright (c) 2011-2025, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

# Determine Python site-packages directory
# Use sysconfig instead of distutils (removed in Python 3.12+)
execute_process(COMMAND ${Python3_EXECUTABLE} -c
  "import sysconfig; print(sysconfig.get_path('platlib'))"
  OUTPUT_VARIABLE PYTHON_SITE_PACKAGES
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
if(NOT IS_ABSOLUTE ${PYTHON_SITE_PACKAGES})
  set(PYTHON_SITE_PACKAGES "${CMAKE_INSTALL_PREFIX}/${PYTHON_SITE_PACKAGES}")
endif()
set(PYTHON_SITE_PACKAGES ${PYTHON_SITE_PACKAGES} PARENT_SCOPE)

file(GLOB_RECURSE dartpy_headers "*.h" "*.hpp")
file(GLOB_RECURSE dartpy_sources "*.cpp")

# Exclude GUI OSG bindings if GUI is not built
if(NOT DART_BUILD_GUI_OSG)
  list(FILTER dartpy_headers EXCLUDE REGEX ".*/gui/osg/.*")
  list(FILTER dartpy_sources EXCLUDE REGEX ".*/gui/osg/.*")
endif()

# Python binding module name
set(pybind_module dartpy)

# Build a Python extension module:
# pybind11_add_module(<name> [MODULE | SHARED] [EXCLUDE_FROM_ALL]
#                     [NO_EXTRAS] [SYSTEM] [THIN_LTO] source1 [source2 ...])
#
# When building wheels, don't use EXCLUDE_FROM_ALL so the module is built automatically
if(DART_BUILD_WHEELS)
  pybind11_add_module(${pybind_module}
    MODULE
    ${dartpy_headers}
    ${dartpy_sources}
  )
else()
  pybind11_add_module(${pybind_module}
    MODULE
    EXCLUDE_FROM_ALL
    ${dartpy_headers}
    ${dartpy_sources}
  )
endif()

target_include_directories(${pybind_module}
  SYSTEM PUBLIC
    ${PYTHON_INCLUDE_DIRS}
    ${pybind11_INCLUDE_DIRS}
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

set(dartpy_libraries
  dart
  dart-utils
  dart-utils-urdf
)
foreach(dartpy_library ${dartpy_libraries})
  if(NOT TARGET ${dartpy_library})
    message(FATAL_ERROR "Cannot find the library ${dartpy_library}. Please check the configuration of DART.")
  endif()
endforeach()

target_link_libraries(${pybind_module}
  PUBLIC ${dartpy_libraries}
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
if(TARGET dart-gui-osg)
  target_link_libraries(${pybind_module} PUBLIC dart-gui-osg)
endif()

target_compile_definitions(${pybind_module}
  PRIVATE DARTPY_VERSION_INFO="${DARTPY_VERSION_INFO}"
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

# Install the pybind module to the wheel
# For scikit-build-core, we use SKBUILD_PLATLIB_DIR if available,
# otherwise fall back to PYTHON_SITE_PACKAGES for traditional builds
if(DEFINED SKBUILD_PLATLIB_DIR)
  install(TARGETS ${pybind_module}
    LIBRARY DESTINATION "${SKBUILD_PLATLIB_DIR}"
  )
else()
  install(TARGETS ${pybind_module}
    LIBRARY DESTINATION "${PYTHON_SITE_PACKAGES}"
  )
endif()

list(REMOVE_ITEM dartpy_headers
  ${CMAKE_CURRENT_LIST_DIR}/eigen_geometry_pybind.h
  ${CMAKE_CURRENT_LIST_DIR}/eigen_pybind.h
)
list(REMOVE_ITEM dartpy_sources
  ${CMAKE_CURRENT_LIST_DIR}/eigen_geometry_pybind.cpp
)
dart_format_add(${dartpy_headers} ${dartpy_sources})
