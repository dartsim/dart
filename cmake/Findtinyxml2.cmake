# Copyright (c) 2011-2019, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/master/LICENSE
#
# This file is provided under the "BSD-style" License

# Find TINYXML2
#
# This sets the following variables:
#   TINYXML2_FOUND
#   TINYXML2_INCLUDE_DIRS
#   TINYXML2_LIBRARIES
#   TINYXML2_VERSION
#
# and the following targets:
#   tinyxml2::tinyxml2

find_package(PkgConfig QUIET)

# Check if the pkgconfig file is installed
pkg_check_modules(PC_TINYXML2 tinyxml2 QUIET)

# Include directories
find_path(TINYXML2_INCLUDE_DIRS
    NAMES tinyxml2.h
    HINTS ${PC_TINYXML2_INCLUDEDIR}
    PATHS "${CMAKE_INSTALL_PREFIX}/include")

# Libraries
if(MSVC)
  set(TINYXML2_LIBRARIES optimized tinyxml2 debug tinyxml2d)
else()
  find_library(TINYXML2_LIBRARIES
      NAMES tinyxml2
      HINTS ${PC_TINYXML2_LIBDIR})
endif()

# Version
set(TINYXML2_VERSION ${PC_TINYXML2_VERSION})

# Set (NAME)_FOUND if all the variables and the version are satisfied.
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(TINYXML2
    FAIL_MESSAGE  DEFAULT_MSG
    REQUIRED_VARS TINYXML2_INCLUDE_DIRS TINYXML2_LIBRARIES
    VERSION_VAR   TINYXML2_VERSION)
