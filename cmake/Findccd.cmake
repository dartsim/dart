# Copyright (c) 2011-2019, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/master/LICENSE
#
# This file is provided under the "BSD-style" License

# Find CCD
#
# This sets the following variables:
#   CCD_FOUND
#   CCD_INCLUDE_DIRS
#   CCD_LIBRARIES
#   CCD_VERSION
#
# and the following targets:
#   ccd

find_package(ccd QUIET CONFIG)
# Upstream provide ccd-config.cmake since 2.1.

if(NOT CCD_FOUND)

  find_package(PkgConfig QUIET)

  # Check to see if pkgconfig is installed.
  pkg_check_modules(PC_CCD ccd QUIET)

  # Include directories
  # Give explicit precedence to ${PC_CCD_INCLUDEDIR}
  find_path(CCD_INCLUDE_DIRS
      NAMES ccd/ccd.h
      HINTS ${PC_CCD_INCLUDEDIR}
      NO_DEFAULT_PATH
      NO_CMAKE_PATH
      NO_CMAKE_ENVIRONMENT_PATH
      NO_SYSTEM_ENVIRONMENT_PATH)
  find_path(CCD_INCLUDE_DIRS
      NAMES ccd/ccd.h
      HINTS ${PC_CCD_INCLUDEDIR}
      PATHS "${CMAKE_INSTALL_PREFIX}/include")

  # Libraries
  if(MSVC)
    set(CCD_LIBRARIES optimized ccd debug ccdd)
  else()
    # Give explicit precedence to ${PC_CCD_LIBDIR}
    find_library(CCD_LIBRARIES
        NAMES ccd
        HINTS ${PC_CCD_LIBDIR}
        NO_DEFAULT_PATH
        NO_CMAKE_PATH
        NO_CMAKE_ENVIRONMENT_PATH
        NO_SYSTEM_ENVIRONMENT_PATH)
    find_library(CCD_LIBRARIES
        NAMES ccd
        HINTS ${PC_CCD_LIBDIR})
  endif()

  # Version
  set(CCD_VERSION ${PC_CCD_VERSION})

  # Set (NAME)_FOUND if all the variables and the version are satisfied.
  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(CCD
      FAIL_MESSAGE  DEFAULT_MSG
      REQUIRED_VARS CCD_INCLUDE_DIRS CCD_LIBRARIES
      VERSION_VAR   CCD_VERSION)

endif()
