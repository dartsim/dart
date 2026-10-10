# Copyright (c) 2011-2025, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

if(
  NOT CMAKE_CROSSCOMPILING
  AND
    (
      NOT Vulkan_LIBRARY
      OR Vulkan_LIBRARY MATCHES "^/usr/lib(/|$)"
      OR Vulkan_LIBRARY MATCHES "^/usr/lib64(/|$)"
    )
)
  # Prefer the Vulkan loader that ships with the active prefix (e.g., pixi/conda)
  # to avoid RPATH conflicts when the system loader is also present. Skip this
  # override entirely when cross-compiling so toolchains can provide their own loader.
  set(_dart_vulkan_search_prefixes)
  if(DEFINED ENV{CONDA_PREFIX})
    list(APPEND _dart_vulkan_search_prefixes "$ENV{CONDA_PREFIX}")
  endif()
  list(APPEND _dart_vulkan_search_prefixes ${CMAKE_PREFIX_PATH})

  set(_dart_vulkan_local_found FALSE)
  foreach(_dart_vulkan_prefix IN LISTS _dart_vulkan_search_prefixes)
    if(NOT _dart_vulkan_prefix)
      continue()
    endif()
    foreach(_dart_vulkan_candidate libvulkan.so libvulkan.so.1)
      set(
        _dart_vulkan_path
        "${_dart_vulkan_prefix}/lib/${_dart_vulkan_candidate}"
      )
      if(EXISTS "${_dart_vulkan_path}")
        set(
          Vulkan_LIBRARY
          "${_dart_vulkan_path}"
          CACHE FILEPATH
          "Path to the Vulkan loader library"
          FORCE
        )
        set(_dart_vulkan_local_found TRUE)
        break()
      endif()
    endforeach()
    # Only stop once the CURRENT prefix actually provided a loader. Do not rely on
    # the pre-existing cached Vulkan_LIBRARY value (which may point at a stale
    # system loader), otherwise later prefixes would be skipped and the intended
    # prefix-local preference would be lost.
    if(_dart_vulkan_local_found)
      break()
    endif()
  endforeach()
  unset(_dart_vulkan_local_found)
  unset(_dart_vulkan_candidate)
  unset(_dart_vulkan_path)
  unset(_dart_vulkan_prefix)
  unset(_dart_vulkan_search_prefixes)
endif()

include("${CMAKE_CURRENT_LIST_DIR}/DARTFindPackageVersion.cmake")
find_package(imgui CONFIG)
set(_dart_imgui_config_found ${imgui_FOUND})
if(imgui_FOUND)
  if(NOT imgui_VERSION)
    set(_dart_imgui_include_dirs ${imgui_INCLUDE_DIRS})
    if(TARGET imgui::imgui)
      get_target_property(
        _dart_imgui_target_includes
        imgui::imgui
        INTERFACE_INCLUDE_DIRECTORIES
      )
      list(APPEND _dart_imgui_include_dirs ${_dart_imgui_target_includes})
    endif()
    dart_read_header_version(imgui_VERSION imgui.h IMGUI_VERSION ${_dart_imgui_include_dirs})
  endif()
  dart_check_package_version(imgui 1.91.9)
  if(NOT imgui_FOUND)
    message(FATAL_ERROR "ImGui >= 1.91.9 with version information is required")
  endif()
endif()

if(NOT _dart_imgui_config_found)
  find_package(imgui 1.91.9 REQUIRED MODULE)
endif()
unset(_dart_imgui_config_found)

if(imgui_FOUND AND NOT TARGET imgui::imgui)
  add_library(imgui::imgui INTERFACE IMPORTED)
  set_target_properties(
    imgui::imgui
    PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${imgui_INCLUDE_DIRS}"
      INTERFACE_LINK_LIBRARIES "${imgui_LIBRARIES}"
  )
endif()
