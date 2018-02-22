# Copyright (c) 2011-2018, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/master/LICENSE
#
# This file is provided under the "BSD-style" License

# Find SHARK
#
# This sets the following variables:
# SHARK_FOUND
# SHARK_INCLUDE_DIRS
# SHARK_LIBRARIES
# SHARK_DEFINITIONS
# SHARK_VERSION

find_package(PkgConfig QUIET)

# Check to see if pkgconfig is installed.
pkg_check_modules(PC_SHARK Shark QUIET)

# Definitions
set(SHARK_DEFINITIONS ${PC_SHARK_CFLAGS_OTHER})

# Include directories
find_path(SHARK_INCLUDE_DIRS
    NAMES shark/Core/Shark.h
    HINTS ${PC_SHARK_INCLUDEDIR}
    PATHS "${CMAKE_INSTALL_PREFIX}/include")

# Libraries
find_library(SHARK_LIBRARIES
    NAMES shark
    HINTS ${PC_SHARK_LIBDIR})

# Version
set(SHARK_VERSION ${PC_SHARK_VERSION})

# Set (NAME)_FOUND if all the variables and the version are satisfied.
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(SHARK
    FAIL_MESSAGE  DEFAULT_MSG
    REQUIRED_VARS SHARK_INCLUDE_DIRS SHARK_LIBRARIES
    VERSION_VAR   SHARK_VERSION)
