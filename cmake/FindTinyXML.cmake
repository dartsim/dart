# Copyright (c) 2015, Georgia Tech Graphics Lab and Humanoid Robotics Lab
# This file is provided under the "BSD-style" License

# Find TinyXML
#
# This sets the following variables:
# TINYXML_FOUND
# TinyXML_INCLUDE_DIRS
# TinyXML_LIBRARIES
# TinyXML_VERSION

find_package(PkgConfig QUIET)

# Check to see if pkgconfig is installed.
pkg_check_modules(PC_TinyXML tinyxml QUIET)

# Include directories
find_path(TinyXML_INCLUDE_DIRS
    NAMES tinyxml.h
    HINTS ${PC_TinyXML_INCLUDEDIR}
    PATHS "${CMAKE_INSTALL_PREFIX}/include")

# Libraries
if(MSVC)
  set(TinyXML_LIBRARIES optimized tinyxml debug tinyxmld)
else()
  find_library(TinyXML_LIBRARIES
      NAMES tinyxml
      HINTS ${PC_TinyXML_LIBDIR})
endif()

# Version
set(TinyXML_VERSION ${PC_TinyXML_VERSION})

# Set (NAME)_FOUND if all the variables and the version are satisfied.
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(TinyXML
    #FOUND_VAR     TinyXML_FOUND
    FAIL_MESSAGE  DEFAULT_MSG
    REQUIRED_VARS TinyXML_INCLUDE_DIRS TinyXML_LIBRARIES
    VERSION_VAR   TinyXML_VERSION)

