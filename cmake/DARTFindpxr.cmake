# Copyright (c) 2011-2025, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

set(_dart_pxr_components
  usd
  usdGeom
  usdPhysics
  sdf
  tf
  usdUtils
)

# Prefer the upstream pxr CMake package configuration if it is available.
find_package(pxr QUIET CONFIG COMPONENTS ${_dart_pxr_components})

# Fall back to a generic lookup without explicit components (older USD builds)
if(NOT pxr_FOUND)
  find_package(pxr QUIET CONFIG)
endif()

if(pxr_FOUND AND NOT DEFINED PXR_VERSION)
  # pxr packages expose PXR_VERSION as either a cache entry or property.
  if(DEFINED pxr_VERSION)
    set(PXR_VERSION ${pxr_VERSION})
  elseif(TARGET pxr::tf)
    get_target_property(_pxr_tf_version pxr::tf INTERFACE_COMPILE_DEFINITIONS)
    set(PXR_VERSION "${_pxr_tf_version}")
  endif()
endif()

unset(_dart_pxr_components)

if(pxr_FOUND)
  set(_dart_pxr_aliases usd usdGeom usdPhysics sdf tf)
  foreach(_dart_pxr_target IN LISTS _dart_pxr_aliases)
    if(TARGET ${_dart_pxr_target} AND NOT TARGET pxr::${_dart_pxr_target})
      add_library(pxr::${_dart_pxr_target} ALIAS ${_dart_pxr_target})
    endif()
  endforeach()
  unset(_dart_pxr_aliases)
endif()
