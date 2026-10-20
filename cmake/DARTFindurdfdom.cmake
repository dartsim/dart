# Copyright (c) 2011-2025, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

include("${CMAKE_CURRENT_LIST_DIR}/DARTFindPackageVersion.cmake")

# Discover without a version so config files for newer major releases work.
find_package(urdfdom QUIET CONFIG)
dart_check_package_version(urdfdom 4.0.0)
if(urdfdom_DIR AND NOT urdfdom_VERSION)
  message(STATUS "urdfdom older than 4.0.0 ships no CMake version file")
endif()

if(urdfdom_FOUND AND MSVC)
  # Remove invalid path (i.e., /include) from urdfdom_INCLUDE_DIRS. This happens
  # when it's installed by vcpkg on Windows. See:
  # - https://github.com/dartsim/dart/issues/1365
  # - https://github.com/ros/urdfdom/issues/140
  # Keep this strip until a vcpkg urdfdom 4.0+ port is checked.
  if("/include" IN_LIST urdfdom_INCLUDE_DIRS)
    list(REMOVE_ITEM urdfdom_INCLUDE_DIRS "/include")
  endif()
endif()

if(urdfdom_FOUND AND NOT TARGET urdfdom)
  add_library(urdfdom INTERFACE IMPORTED)
  set_target_properties(
    urdfdom
    PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${urdfdom_INCLUDE_DIRS}"
      INTERFACE_LINK_LIBRARIES "${urdfdom_LIBRARIES}"
  )
endif()
