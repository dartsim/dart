# Copyright (c) 2011-2025, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

# Find ODE
#
# This sets the following variables:
#   ODE_FOUND
#   ODE_INCLUDE_DIRS
#   ODE_LIBRARIES
#   ODE_VERSION
#
# and the following targets:
#   ODE::ODE

include("${CMAKE_CURRENT_LIST_DIR}/DARTFindPackageVersion.cmake")
find_package(PkgConfig 0.29.2 QUIET)

# Check to see if pkgconfig is installed.
if(PkgConfig_FOUND)
  pkg_check_modules(PC_ODE ode QUIET)
endif()

# Include directories
find_path(
  ODE_INCLUDE_DIRS
  NAMES ode/collision.h
  HINTS ${PC_ODE_INCLUDEDIR}
  PATHS "${CMAKE_INSTALL_PREFIX}/include"
)

# Libraries
if(MSVC)
  set(ODE_LIBRARIES "ode$<$<CONFIG:Debug>:d>")
else()
  find_library(ODE_LIBRARIES NAMES ode HINTS ${PC_ODE_LIBDIR})
endif()

# Version
set(ODE_VERSION ${PC_ODE_VERSION})
if(NOT ODE_VERSION)
  dart_read_header_version(ODE_VERSION ode/version.h dODE_VERSION ${ODE_INCLUDE_DIRS})
endif()

# Set (NAME)_FOUND if all the variables and the version are satisfied.
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
  ODE
  FAIL_MESSAGE DEFAULT_MSG
  REQUIRED_VARS ODE_INCLUDE_DIRS ODE_LIBRARIES ODE_VERSION
  VERSION_VAR ODE_VERSION
)
