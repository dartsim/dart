# Copyright (c) 2011-2025, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

# First try the canonical package name exported by VulkanSceneGraph.
find_package(vsg 1.1 QUIET CONFIG)

# Some distributions expose the project name instead of the CMake package name.
if(NOT vsg_FOUND)
  find_package(VulkanSceneGraph QUIET CONFIG)
  if(VulkanSceneGraph_FOUND)
    set(vsg_FOUND TRUE)
    if(NOT DEFINED vsg_VERSION AND DEFINED VulkanSceneGraph_VERSION)
      set(vsg_VERSION "${VulkanSceneGraph_VERSION}")
    endif()
  endif()
endif()

if(vsg_FOUND)
  set(VULKANSCENEGRAPH_FOUND TRUE)
  set(VulkanSceneGraph_FOUND TRUE)
  if(NOT DEFINED VulkanSceneGraph_VERSION AND DEFINED vsg_VERSION)
    set(VulkanSceneGraph_VERSION "${vsg_VERSION}")
  endif()

  # Upstream exports vsg::vsg; if that is missing, fall back to a simple
  # imported interface target using the legacy cache variables.
  if(NOT TARGET vsg::vsg AND VSG_LIBRARIES)
    add_library(vsg::vsg INTERFACE IMPORTED)
    target_link_libraries(vsg::vsg INTERFACE "${VSG_LIBRARIES}")
    if(VSG_INCLUDE_DIR)
      target_include_directories(
        vsg::vsg INTERFACE "${VSG_INCLUDE_DIR}" "${VSG_INCLUDE_DIR}/vsg")
    endif()
  endif()

  if(DART_VERBOSE)
    if(DEFINED VulkanSceneGraph_VERSION)
      message(
        STATUS
        "Looking for VulkanSceneGraph - ${VulkanSceneGraph_VERSION} found")
    else()
      message(STATUS "Looking for VulkanSceneGraph - found")
    endif()
  endif()
else()
  set(VULKANSCENEGRAPH_FOUND FALSE)
  set(VulkanSceneGraph_FOUND FALSE)
endif()
