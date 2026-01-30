# Copyright (c) 2011-2025, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
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
find_path(TINYXML2_INCLUDE_DIRS NAMES tinyxml2.h HINTS ${PC_TINYXML2_INCLUDEDIR} PATHS "${CMAKE_INSTALL_PREFIX}/include")

# Libraries
if(MSVC)
  set(TINYXML2_LIBRARIES "tinyxml2$<$<CONFIG:Debug>:d>")
else()
  find_library(TINYXML2_LIBRARIES NAMES tinyxml2 HINTS ${PC_TINYXML2_LIBDIR})
endif()

# Version
set(TINYXML2_VERSION ${PC_TINYXML2_VERSION})

# Set (NAME)_FOUND if all the variables and the version are satisfied.
include(FindPackageHandleStandardArgs)
set(_tinyxml2_package "${CMAKE_FIND_PACKAGE_NAME}")
find_package_handle_standard_args(
  "${_tinyxml2_package}"
  FAIL_MESSAGE DEFAULT_MSG
  REQUIRED_VARS TINYXML2_INCLUDE_DIRS TINYXML2_LIBRARIES
  VERSION_VAR TINYXML2_VERSION
)

# Mirror the FOUND result for both lower- and upper-case package names so callers
# using either `tinyxml2` or `TINYXML2` see a consistent flag.
if(NOT DEFINED tinyxml2_FOUND)
  set(tinyxml2_FOUND ${${_tinyxml2_package}_FOUND})
endif()
if(NOT DEFINED TINYXML2_FOUND)
  set(TINYXML2_FOUND ${${_tinyxml2_package}_FOUND})
endif()
