# Copyright (c) 2015-2016, Georgia Tech Graphics Lab and Humanoid Robotics Lab
# This file is provided under the "BSD-style" License

# Find FCL
#
# This sets the following variables:
# FCL_FOUND
# FCL_INCLUDE_DIRS
# FCL_LIBRARIES
# FCL_VERSION

find_package(PkgConfig QUIET)

# Check to see if pkgconfig is installed.
pkg_check_modules(PC_FCL fcl QUIET)

# Include directories
find_path(FCL_INCLUDE_DIRS
    NAMES fcl/collision.h
    HINTS ${PC_FCL_INCLUDEDIR}
    PATHS "${CMAKE_INSTALL_PREFIX}/include")

# Libraries
if(MSVC)
  set(FCL_LIBRARIES optimized fcl debug fcld)
else()
  find_library(FCL_LIBRARIES
      NAMES fcl
      HINTS ${PC_FCL_LIBDIR})
endif()

# Version
set(FCL_VERSION ${PC_FCL_VERSION})

# Set (NAME)_FOUND if all the variables and the version are satisfied.
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(FCL
    FAIL_MESSAGE  DEFAULT_MSG
    REQUIRED_VARS FCL_INCLUDE_DIRS FCL_LIBRARIES
    VERSION_VAR   FCL_VERSION)

