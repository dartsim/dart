# Copyright (c) 2011-2025, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

# Eigen 5's version file matches a requested single version only within that
# minor release, and Eigen 3.4's rejects version ranges that span majors, so
# find any Eigen3 package and check the supported range here.
find_package(Eigen3 REQUIRED CONFIG)
if(
  NOT Eigen3_VERSION
  OR Eigen3_VERSION VERSION_LESS 3.4.0
  OR NOT Eigen3_VERSION VERSION_LESS 6.0.0
)
  if(NOT Eigen3_VERSION)
    set(Eigen3_VERSION "unknown")
  endif()
  message(
    FATAL_ERROR
    "Eigen3 >= 3.4.0, < 6 is required; found version ${Eigen3_VERSION}"
  )
endif()

string(REGEX MATCH "^[0-9]+" _dart_eigen_major "${Eigen3_VERSION}")
if(
  DEFINED DART_BUILD_EIGEN_MAJOR
  AND NOT _dart_eigen_major STREQUAL DART_BUILD_EIGEN_MAJOR
)
  message(
    FATAL_ERROR
    "DART was built with Eigen major ${DART_BUILD_EIGEN_MAJOR}, but found Eigen ${Eigen3_VERSION}. "
    "Eigen majors are ABI-incompatible; use the same Eigen major or rebuild DART. "
    "If reusing a build directory, you may need to clear the cached Eigen3_DIR."
  )
endif()
