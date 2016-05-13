# Copyright (c) 2015-2016, Graphics Lab, Georgia Tech Research Corporation
# Copyright (c) 2015-2016, Humanoid Lab, Georgia Tech Research Corporation
# Copyright (c) 2016, Personal Robotics Lab, Carnegie Mellon University
# This file is provided under the "BSD-style" License

# Find TINYXML
#
# This sets the following variables:
# TINYXML_FOUND
# TINYXML_INCLUDE_DIRS
# TINYXML_LIBRARIES
# TINYXML_VERSION

find_package(PkgConfig QUIET)

# Check to see if pkgconfig is installed.
pkg_check_modules(PC_TINYXML tinyxml QUIET)

# Include directories
find_path(TINYXML_INCLUDE_DIRS
    NAMES tinyxml.h
    HINTS ${PC_TINYXML_INCLUDEDIR}
    PATHS "${CMAKE_INSTALL_PREFIX}/include")

# Libraries
if(MSVC)
  set(TINYXML_LIBRARIES optimized tinyxml debug tinyxmld)
else()
  find_library(TINYXML_LIBRARIES
      NAMES tinyxml
      HINTS ${PC_TINYXML_LIBDIR})
endif()

# Version
set(TINYXML_VERSION ${PC_TINYXML_VERSION})

# Set (NAME)_FOUND if all the variables and the version are satisfied.
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(TINYXML
    FAIL_MESSAGE  DEFAULT_MSG
    REQUIRED_VARS TINYXML_INCLUDE_DIRS TINYXML_LIBRARIES
    VERSION_VAR   TINYXML_VERSION)

