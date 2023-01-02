# Copyright (c) 2011-2023, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/master/LICENSE
#
# This file is provided under the "BSD-style" License

find_package(urdfdom QUIET CONFIG)

if(MSVC)
  # Remove invalid path (i.e., /include) from urdfdom_INCLUDE_DIRS. This happens
  # when it's installed by vcpkg on Windows. See:
  # - https://github.com/dartsim/dart/issues/1365
  # - https://github.com/ros/urdfdom/issues/140
  if ("/include" IN_LIST urdfdom_INCLUDE_DIRS)
    list(REMOVE_ITEM urdfdom_INCLUDE_DIRS "/include")
    find_package(TinyXML REQUIRED MODULE)
    list(APPEND urdfdom_INCLUDE_DIRS ${TinyXML_INCLUDE_DIRS})
  endif()
endif()

if(urdfdom_FOUND AND NOT TARGET urdfdom)
  add_library(urdfdom INTERFACE IMPORTED)
  set_target_properties(urdfdom PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${urdfdom_INCLUDE_DIRS}"
    INTERFACE_LINK_LIBRARIES "${urdfdom_LIBRARIES}"
  )
endif()
