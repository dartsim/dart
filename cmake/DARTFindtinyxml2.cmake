# Copyright (c) 2011-2019, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/master/LICENSE
#
# This file is provided under the "BSD-style" License

find_package(tinyxml2 QUIET CONFIG)
if(tinyxml2_FOUND)
  set(TINYXML2_FOUND        ${tinyxml2_FOUND})
  set(TINYXML2_INCLUDE_DIRS ${tinyxml2_INCLUDE_DIRS})
  set(TINYXML2_LIBRARIES    ${tinyxml2_LIBRARIES})
  set(TINYXML2_VERSION      ${tinyxml2_VERSION})
endif()

if(NOT tinyxml2_FOUND)
  find_package(tinyxml2 QUIET MODULE)
endif()

if((TINYXML2_FOUND OR tinyxml2_FOUND) AND NOT TARGET tinyxml2::tinyxml2)
  add_library(tinyxml2::tinyxml2 INTERFACE IMPORTED)
  set_target_properties(tinyxml2::tinyxml2 PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${TINYXML2_INCLUDE_DIRS}"
    INTERFACE_LINK_LIBRARIES "${TINYXML2_LIBRARIES}"
  )
endif()
