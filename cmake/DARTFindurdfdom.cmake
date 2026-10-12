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
if(urdfdom_FOUND AND NOT urdfdom_VERSION)
  # urdfdom < 4 ships no CMake version file; use its own pkg-config metadata.
  get_filename_component(_urdfdom_config_dir "${urdfdom_DIR}" REALPATH)
  find_file(
    _urdfdom_pc
    urdfdom.pc
    PATHS
      "${_urdfdom_config_dir}/../../pkgconfig"
      "${_urdfdom_config_dir}/../../../libdata/pkgconfig"
    NO_DEFAULT_PATH
    NO_CACHE
  )
  if(_urdfdom_pc)
    file(STRINGS "${_urdfdom_pc}" _urdfdom_version_line REGEX "^Version:[ \t]*")
    if(_urdfdom_version_line MATCHES "^Version:[ \t]*([^ \t]+)")
      set(urdfdom_VERSION "${CMAKE_MATCH_1}")
    endif()
  endif()
  unset(_urdfdom_pc)
endif()
# Upstream urdfdom 3.0.1 still reports 3.0.0 (Ubuntu 22.04 ships it); the two
# differ only in MSVC debug library names, so 3.0.0 is the checkable floor.
dart_check_package_version(urdfdom 3.0.0)

if(urdfdom_FOUND AND MSVC)
  # Remove invalid path (i.e., /include) from urdfdom_INCLUDE_DIRS. This happens
  # when it's installed by vcpkg on Windows. See:
  # - https://github.com/dartsim/dart/issues/1365
  # - https://github.com/ros/urdfdom/issues/140
  if("/include" IN_LIST urdfdom_INCLUDE_DIRS)
    list(REMOVE_ITEM urdfdom_INCLUDE_DIRS "/include")
    find_package(TinyXML REQUIRED MODULE)
    list(APPEND urdfdom_INCLUDE_DIRS ${TinyXML_INCLUDE_DIRS})
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
