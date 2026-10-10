# Copyright (c) 2011-2025, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

include("${CMAKE_CURRENT_LIST_DIR}/DARTFindPackageVersion.cmake")

# Discover without a version so config files for newer major releases work.
find_package(tinyxml2 QUIET CONFIG)
set(_tinyxml2_config_found ${tinyxml2_FOUND})
if(tinyxml2_FOUND)
  set(TINYXML2_FOUND ${tinyxml2_FOUND})
  set(TINYXML2_INCLUDE_DIRS ${tinyxml2_INCLUDE_DIRS})
  set(TINYXML2_LIBRARIES ${tinyxml2_LIBRARIES})
  set(TINYXML2_VERSION ${tinyxml2_VERSION})
  if(NOT TINYXML2_VERSION)
    set(_tinyxml2_include_dirs ${TINYXML2_INCLUDE_DIRS})
    if(TARGET tinyxml2::tinyxml2)
      get_target_property(
        _tinyxml2_target_includes
        tinyxml2::tinyxml2
        INTERFACE_INCLUDE_DIRECTORIES
      )
      list(APPEND _tinyxml2_include_dirs ${_tinyxml2_target_includes})
    endif()
    foreach(_tinyxml2_component MAJOR MINOR PATCH)
      dart_read_header_version(_tinyxml2_${_tinyxml2_component} tinyxml2.h
        TINYXML2_${_tinyxml2_component}_VERSION ${_tinyxml2_include_dirs}
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
      set(tinyxml2_VERSION "${TINYXML2_VERSION}")
    endif()
  endif()
  dart_check_package_version(tinyxml2 9.0.0)
endif()

if(NOT _tinyxml2_config_found)
  find_package(tinyxml2 9.0.0 QUIET MODULE)
endif()
unset(_tinyxml2_config_found)

if((TINYXML2_FOUND OR tinyxml2_FOUND) AND NOT TARGET tinyxml2::tinyxml2)
  add_library(tinyxml2::tinyxml2 INTERFACE IMPORTED)
  set_target_properties(
    tinyxml2::tinyxml2
    PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${TINYXML2_INCLUDE_DIRS}"
      INTERFACE_LINK_LIBRARIES "${TINYXML2_LIBRARIES}"
  )
endif()
