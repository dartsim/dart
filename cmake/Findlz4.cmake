# Copyright (c) 2011-2019, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/master/LICENSE
#
# This file is provided under the "BSD-style" License

# Find lz4
#
# This sets the following variables:
#   lz4_FOUND
#   lz4_INCLUDE_DIRS
#   lz4_LIBRARIES
#   lz4_VERSION
#
# and the following targets:
#   lz4

find_package(PkgConfig QUIET)

# Check to see if pkgconfig is installed.
pkg_check_modules(PC_lz4 lz4 QUIET)

# Include directories
find_path(lz4_INCLUDE_DIRS
    NAMES lz4.h
    HINTS ${PC_lz4_INCLUDEDIR}
    PATHS "${CMAKE_INSTALL_PREFIX}/include")

# Libraries
find_library(lz4_LIBRARIES lz4
    HINTS ${PC_lz4_LIBDIR})

# Version
set(lz4_VERSION ${PC_lz4_VERSION})

# Set (NAME)_FOUND if all the variables and the version are satisfied.
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(lz4
    FAIL_MESSAGE  DEFAULT_MSG
    REQUIRED_VARS lz4_INCLUDE_DIRS lz4_LIBRARIES
    VERSION_VAR   lz4_VERSION)

if(lz4_FOUND AND NOT TARGET lz4)
  add_library(lz4 INTERFACE IMPORTED)
  set_target_properties(lz4 PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${lz4_INCLUDE_DIRS}"
    INTERFACE_LINK_LIBRARIES "${lz4_LIBRARIES}"
  )
endif()
