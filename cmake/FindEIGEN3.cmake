# Copyright (c) 2011-2018, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/master/LICENSE
#
# This file is provided under the "BSD-style" License

# Find Eigen
#
# This sets the following variables:
# EIGEN3_FOUND
# EIGEN3_INCLUDE_DIRS
# EIGEN3_VERSION

find_package(PkgConfig QUIET)

# Check to see if pkgconfig is installed.
pkg_check_modules(PC_EIGEN3 eigen3 QUIET)

# Include directories
find_path(EIGEN3_INCLUDE_DIRS
    NAMES Eigen/Core
    PATHS "${CMAKE_INSTALL_PREFIX}/include"
    PATH_SUFFIXES eigen3 eigen)

# Version
set(EIGEN3_VERSION ${PC_EIGEN3_VERSION})

# Set (NAME)_FOUND if all the variables and the version are satisfied.
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(EIGEN3
    FAIL_MESSAGE  DEFAULT_MSG
    REQUIRED_VARS EIGEN3_INCLUDE_DIRS
    VERSION_VAR   EIGEN3_VERSION)

