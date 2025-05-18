# Copyright (c) 2011-2025, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

# Find imgui
#
# This sets the following variables:
#   imgui_FOUND
#   imgui_INCLUDE_DIRS
#   imgui_LIBRARIES
#   imgui_VERSION

find_package(PkgConfig QUIET)

# Check to see if pkgconfig is installed.
pkg_check_modules(PC_imgui imgui QUIET)

# Find the path containing imgui.h
find_path(imgui_INCLUDE_DIR
  NAMES imgui.h
  HINTS ${PC_imgui_INCLUDEDIR}
  PATHS
    "${CMAKE_INSTALL_PREFIX}/include"
    "${CMAKE_INSTALL_PREFIX}/include/imgui"
)

# Find the path containing imgui_impl_opengl2.h
find_path(imgui_backends_INCLUDE_DIR
  NAMES imgui_impl_opengl2.h
  HINTS
    ${PC_imgui_INCLUDEDIR}
    ${PC_imgui_INCLUDEDIR}/backends
  PATHS
    "${CMAKE_INSTALL_PREFIX}/include"
    "${CMAKE_INSTALL_PREFIX}/include/imgui/backends"
)

# Combine both paths into imgui_INCLUDE_DIRS
if (imgui_INCLUDE_DIR AND imgui_backends_INCLUDE_DIR)
  set(imgui_INCLUDE_DIRS ${imgui_INCLUDE_DIR} ${imgui_backends_INCLUDE_DIR})
else()
  message(FATAL_ERROR "Could not find the required imgui headers")
endif()

# Library
find_library(imgui_LIBRARIES
  NAMES imgui
  HINTS ${PC_imgui_LIBDIR}
)

# Version
if(PC_imgui_VERSION)
  set(imgui_VERSION ${PC_imgui_VERSION})
endif()

# Set (NAME)_FOUND if all the variables and the version are satisfied.
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(imgui
  FAIL_MESSAGE  DEFAULT_MSG
  REQUIRED_VARS imgui_INCLUDE_DIRS imgui_LIBRARIES
  VERSION_VAR   imgui_VERSION
)
