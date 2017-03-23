# Copyright (c) 2015-2016, Humanoid Lab, Georgia Tech Research Corporation
# Copyright (c) 2015-2017, Graphics Lab, Georgia Tech Research Corporation
# Copyright (c) 2016-2017, Personal Robotics Lab, Carnegie Mellon University
# This file is provided under the "BSD-style" License

# Find ODE
#
# This sets the following variables:
# ODE_FOUND
# ODE_INCLUDE_DIRS
# ODE_LIBRARIES
# ODE_VERSION

find_package(PkgConfig QUIET)

# Check to see if pkgconfig is installed.
pkg_check_modules(PC_ODE ode QUIET)

# Include directories
find_path(ODE_INCLUDE_DIRS
    NAMES ode/collision.h
    HINTS ${PC_ODE_INCLUDEDIR}
    PATHS "${CMAKE_INSTALL_PREFIX}/include")

# Libraries
if(MSVC)
  set(ODE_LIBRARIES optimized ode debug oded)
else()
  find_library(ODE_LIBRARIES
      NAMES ode
      HINTS ${PC_ODE_LIBDIR})
endif()

# Version
set(ODE_VERSION ${PC_ODE_VERSION})

# Set (NAME)_FOUND if all the variables and the version are satisfied.
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(ODE
    FAIL_MESSAGE  DEFAULT_MSG
    REQUIRED_VARS ODE_INCLUDE_DIRS ODE_LIBRARIES
    VERSION_VAR   ODE_VERSION)

