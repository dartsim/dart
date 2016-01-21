# Copyright (c) 2015, Georgia Tech Graphics Lab and Humanoid Robotics Lab
# This file is provided under the "BSD-style" License

# Find FLANN
#
# This sets the following variables:
# FLANN_FOUND
# FLANN_INCLUDE_DIRS
# FLANN_LIBRARIES
# FLANN_DEFINITIONS
# FLANN_VERSION

find_package(PkgConfig QUIET)

# Check to see if pkgconfig is installed.
pkg_check_modules(PC_FLANN flann QUIET)

# Include directories
find_path(FLANN_INCLUDE_DIRS
    NAMES flann/flann.h
    HINTS ${PC_FLANN_INCLUDEDIR} ${PC_FLANN_INCLUDE_DIRS}
    PATHS "${CMAKE_INSTALL_PREFIX}/include")

# Library
find_library(FLANN_LIBRARIES
    NAMES flann
    HINTS ${PC_FLANN_LIBDIR} ${PC_FLANN_LIBRARY_DIRS})

# Definitions
set(FLANN_DEFINITIONS ${PC_FLANN_CFLAGS_OTHER})

# Version
set(FLANN_VERSION ${PC_FLANN_VERSION})

# Set (NAME)_FOUND if all the variables and the version are satisfied.
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(FLANN
    FAIL_MESSAGE  DEFAULT_MSG
    REQUIRED_VARS FLANN_INCLUDE_DIRS FLANN_LIBRARIES
    VERSION_VAR   FLANN_VERSION)

