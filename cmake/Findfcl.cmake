# Copyright (c) 2011-2025, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

# Find FCL
#
# This sets the following variables:
#   FCL_FOUND
#   FCL_INCLUDE_DIRS
#   FCL_LIBRARIES
#   FCL_VERSION

include("${CMAKE_CURRENT_LIST_DIR}/DARTFindPackageVersion.cmake")
find_package(PkgConfig 0.29.2 QUIET)

# Check to see if pkgconfig is installed.
if(PkgConfig_FOUND)
  pkg_check_modules(PC_FCL fcl QUIET)
endif()

# Include directories
find_path(
  FCL_INCLUDE_DIRS
  NAMES
    fcl/collision.h # for FCL < 0.6
  NAMES fcl/narrowphase/collision.h
  HINTS ${PC_FCL_INCLUDEDIR}
  PATHS "${CMAKE_INSTALL_PREFIX}/include"
)

# Libraries
if(MSVC)
  find_package(fcl QUIET CONFIG)
  if(TARGET fcl)
    set(FCL_LIBRARIES fcl)
  endif()
else()
  # Give explicit precedence to ${PC_FCL_LIBDIR}
  find_library(
    FCL_LIBRARIES
    NAMES fcl
    HINTS ${PC_FCL_LIBDIR}
    NO_DEFAULT_PATH
    NO_CMAKE_PATH
    NO_CMAKE_ENVIRONMENT_PATH
    NO_SYSTEM_ENVIRONMENT_PATH
  )
  find_library(FCL_LIBRARIES NAMES fcl HINTS ${PC_FCL_LIBDIR})
endif()

# Version
if(PC_FCL_VERSION)
  set(FCL_VERSION ${PC_FCL_VERSION})
endif()
if(NOT FCL_VERSION AND fcl_VERSION)
  set(FCL_VERSION "${fcl_VERSION}")
endif()
if(NOT FCL_VERSION)
  dart_read_header_version(FCL_VERSION fcl/config.h FCL_VERSION ${FCL_INCLUDE_DIRS})
endif()

# Set (NAME)_FOUND if all the variables and the version are satisfied.
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
  fcl
  FAIL_MESSAGE DEFAULT_MSG
  REQUIRED_VARS FCL_INCLUDE_DIRS FCL_LIBRARIES FCL_VERSION
  VERSION_VAR FCL_VERSION
)
