# Copyright (c) 2011-2018, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/master/LICENSE
#
# This file is provided under the "BSD-style" License

# Find SDL2
#
# This sets the following variables:
# SDL2_FOUND
# SDL2_INCLUDE_DIRS
# SDL2_LIBRARIES
# SDL2_DEFINITIONS
# SDL2_VERSION

find_package(PkgConfig QUIET)

# Check to see if pkgconfig is installed.
pkg_check_modules(PC_SDL2 sdl2 QUIET)

# Definitions
set(SDL2_DEFINITIONS ${PC_SDL2_CFLAGS_OTHER})

# Include directories
find_path(SDL2_INCLUDE_DIRS
    NAMES SDL2/SDL.h
    HINTS ${PC_SDL2_INCLUDEDIR}
    PATHS "${CMAKE_INSTALL_PREFIX}/include")

# Libraries
find_library(SDL2_LIBRARIES
    NAMES SDL2
    HINTS ${PC_SDL2_LIBDIR})

# Version
set(SDL2_VERSION ${PC_SDL2_VERSION})

# Set (NAME)_FOUND if all the variables and the version are satisfied.
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(SDL2
    FAIL_MESSAGE  DEFAULT_MSG
    REQUIRED_VARS SDL2_INCLUDE_DIRS SDL2_LIBRARIES
    VERSION_VAR   SDL2_VERSION)

