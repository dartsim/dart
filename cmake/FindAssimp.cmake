# Copyright (c) 2015, Georgia Tech Graphics Lab and Humanoid Robotics Lab
# This file is provided under the "BSD-style" License

# Find Assimp
#
# This sets the following variables:
# ASSIMP_FOUND
# Assimp_INCLUDE_DIRS
# Assimp_LIBRARIES
# Assimp_VERSION

find_package(PkgConfig QUIET)

# Check to see if pkgconfig is installed.
pkg_check_modules(PC_Assimp assimp QUIET)

# Include directories
find_path(Assimp_INCLUDE_DIRS assimp/scene.h
    HINTS ${PC_Assimp_INCLUDEDIR}
    PATHS "${CMAKE_INSTALL_PREFIX}/include")

# Libraries
if(MSVC)
  set(Assimp_LIBRARIES optimized assimp debug assimpd)
else()
  find_library(Assimp_LIBRARIES
      NAMES assimp
      HINTS ${PC_Assimp_LIBDIR})
endif()

# Version
set(Assimp_VERSION ${PC_Assimp_VERSION})

# Set (NAME)_FOUND if all the variables and the version are satisfied.
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Assimp
    #FOUND_VAR     Assimp_FOUND
    FAIL_MESSAGE  DEFAULT_MSG
    REQUIRED_VARS Assimp_INCLUDE_DIRS Assimp_LIBRARIES Assimp_VERSION
    VERSION_VAR   Assimp_VERSION)

