# Find Eigen
#
# This sets the following variables:
# EIGEN3_FOUND
# Eigen3_INCLUDE_DIRS
# Eigen3_VERSION

# Copyright (c) 2015, Georgia Tech Graphics Lab and Humanoid Robotics Lab
# This file is provided under the "BSD-style" License

find_package(PkgConfig QUIET)

# Check to see if pkgconfig is installed.
pkg_check_modules(PC_Eigen3 eigen3 QUIET)

# Include directories
find_path(Eigen3_INCLUDE_DIRS
    NAMES Eigen/Core
    PATHS "${CMAKE_INSTALL_PREFIX}/include"
    PATH_SUFFIXES eigen3 eigen)

# Version
set(Eigen3_VERSION ${PC_Eigen3_VERSION})

# Set (NAME)_FOUND if all the variables and the version are satisfied.
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Eigen3
    #FOUND_VAR     Eigen3_FOUND
    FAIL_MESSAGE  DEFAULT_MSG
    REQUIRED_VARS Eigen3_INCLUDE_DIRS Eigen3_VERSION
    VERSION_VAR   Eigen3_VERSION)

