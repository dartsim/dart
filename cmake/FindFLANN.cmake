# Copyright (c) 2011-2017, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/master/LICENSE
#
# This file is provided under the "BSD-style" License

# Find FLANN
#
# This sets the following variables:
# FLANN_FOUND
# FLANN_INCLUDE_DIRS
# FLANN_LIBRARIES
# FLANN_VERSION

find_package(PkgConfig QUIET)

# Check to see if pkgconfig is installed.
pkg_check_modules(PC_FLANN flann QUIET)

# Include directories
find_path(FLANN_INCLUDE_DIRS
    NAMES flann/flann.h
    HINTS ${PC_FLANN_INCLUDEDIR}
    PATHS "${CMAKE_INSTALL_PREFIX}/include")

# Libraries
find_library(FLANN_LIBRARIES flann_cpp
    HINTS ${PC_FLANN_LIBDIR})

# Version
set(FLANN_VERSION ${PC_FLANN_VERSION})

# Set (NAME)_FOUND if all the variables and the version are satisfied.
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(FLANN
    FAIL_MESSAGE  DEFAULT_MSG
    REQUIRED_VARS FLANN_INCLUDE_DIRS FLANN_LIBRARIES
    VERSION_VAR   FLANN_VERSION)

