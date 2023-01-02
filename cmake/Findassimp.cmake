# Copyright (c) 2011-2023, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/master/LICENSE
#
# This file is provided under the "BSD-style" License

# Find ASSIMP
#
# This sets the following variables:
# ASSIMP_FOUND
# ASSIMP_INCLUDE_DIRS
# ASSIMP_LIBRARIES
# ASSIMP_VERSION

find_package(PkgConfig QUIET)

# Check to see if pkgconfig is installed.
pkg_check_modules(PC_ASSIMP assimp QUIET)

# Include directories
find_path(ASSIMP_INCLUDE_DIRS assimp/scene.h
    HINTS ${PC_ASSIMP_INCLUDEDIR}
    PATHS "${CMAKE_INSTALL_PREFIX}/include")

# Libraries
if(MSVC)
  find_package(assimp QUIET CONFIG)
  if(TARGET assimp::assimp)
    set(ASSIMP_LIBRARIES "assimp::assimp")
  endif()
else()
  find_library(ASSIMP_LIBRARIES
      NAMES assimp
      HINTS ${PC_ASSIMP_LIBDIR})
endif()

# Version
if(PC_ASSIMP_VERSION)
  set(ASSIMP_VERSION ${PC_ASSIMP_VERSION})
endif()

# Set (NAME)_FOUND if all the variables and the version are satisfied.
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(assimp
    FAIL_MESSAGE  DEFAULT_MSG
    REQUIRED_VARS ASSIMP_INCLUDE_DIRS ASSIMP_LIBRARIES
    VERSION_VAR   ASSIMP_VERSION)
