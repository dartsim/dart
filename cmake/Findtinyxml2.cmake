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

include("${CMAKE_CURRENT_LIST_DIR}/DARTFindPackageVersion.cmake")
find_package(PkgConfig 0.29.2 QUIET)

# Check if the pkgconfig file is installed
if(PkgConfig_FOUND)
  pkg_check_modules(PC_TINYXML2 tinyxml2 QUIET)
endif()

# Include directories
find_path(
  TINYXML2_INCLUDE_DIRS
  NAMES tinyxml2.h
  HINTS ${PC_TINYXML2_INCLUDEDIR}
  PATHS "${CMAKE_INSTALL_PREFIX}/include"
)

# Libraries
if(MSVC)
  set(TINYXML2_LIBRARIES "tinyxml2$<$<CONFIG:Debug>:d>")
else()
  find_library(TINYXML2_LIBRARIES NAMES tinyxml2 HINTS ${PC_TINYXML2_LIBDIR})
endif()

# Version
set(TINYXML2_VERSION ${PC_TINYXML2_VERSION})
if(NOT TINYXML2_VERSION)
  foreach(_tinyxml2_component MAJOR MINOR PATCH)
    dart_read_header_version(_tinyxml2_${_tinyxml2_component} tinyxml2.h
      TINYXML2_${_tinyxml2_component}_VERSION ${TINYXML2_INCLUDE_DIRS}
    )
  endforeach()
  if(
    _tinyxml2_MAJOR MATCHES "^[0-9]+$"
    AND _tinyxml2_MINOR MATCHES "^[0-9]+$"
    AND _tinyxml2_PATCH MATCHES "^[0-9]+$"
  )
    set(
      TINYXML2_VERSION
      "${_tinyxml2_MAJOR}.${_tinyxml2_MINOR}.${_tinyxml2_PATCH}"
    )
  endif()
endif()

# Set (NAME)_FOUND if all the variables and the version are satisfied.
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
  tinyxml2
  FAIL_MESSAGE DEFAULT_MSG
  REQUIRED_VARS TINYXML2_INCLUDE_DIRS TINYXML2_LIBRARIES TINYXML2_VERSION
  VERSION_VAR TINYXML2_VERSION
)
