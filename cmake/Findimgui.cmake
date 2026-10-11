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

include("${CMAKE_CURRENT_LIST_DIR}/DARTFindPackageVersion.cmake")
find_package(PkgConfig 0.29.2 QUIET)

# Check to see if pkgconfig is installed.
if(PkgConfig_FOUND)
  pkg_check_modules(PC_imgui imgui QUIET)
endif()

# Find the path containing imgui.h
find_path(
  imgui_INCLUDE_DIR
  NAMES imgui.h
  HINTS ${PC_imgui_INCLUDEDIR}
  PATHS
    "${CMAKE_INSTALL_PREFIX}/include"
    "${CMAKE_INSTALL_PREFIX}/include/imgui"
)

set(imgui_INCLUDE_DIRS ${imgui_INCLUDE_DIR})

# Library
find_library(imgui_LIBRARIES NAMES imgui HINTS ${PC_imgui_LIBDIR})

# Version
if(PC_imgui_VERSION)
  set(imgui_VERSION ${PC_imgui_VERSION})
endif()
if(NOT imgui_VERSION)
  dart_read_header_version(imgui_VERSION imgui.h IMGUI_VERSION ${imgui_INCLUDE_DIRS})
endif()

# Set (NAME)_FOUND if all the variables and the version are satisfied.
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
  imgui
  FAIL_MESSAGE DEFAULT_MSG
  REQUIRED_VARS imgui_INCLUDE_DIRS imgui_LIBRARIES imgui_VERSION
  VERSION_VAR imgui_VERSION
)
