# Copyright (c) 2011-2025, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

if(
  NOT CMAKE_CROSSCOMPILING
  AND (NOT Vulkan_LIBRARY OR Vulkan_LIBRARY MATCHES "^/usr/lib(/|$)" OR Vulkan_LIBRARY MATCHES "^/usr/lib64(/|$)")
)
  # Prefer the Vulkan loader that ships with the active prefix (e.g., pixi/conda)
  # to avoid RPATH conflicts when the system loader is also present. Skip this
  # override entirely when cross-compiling so toolchains can provide their own loader.
  set(_dart_vulkan_search_prefixes)
  if(DEFINED ENV{CONDA_PREFIX})
    list(APPEND _dart_vulkan_search_prefixes "$ENV{CONDA_PREFIX}")
  endif()
  list(APPEND _dart_vulkan_search_prefixes ${CMAKE_PREFIX_PATH})

  foreach(_dart_vulkan_prefix IN LISTS _dart_vulkan_search_prefixes)
    if(NOT _dart_vulkan_prefix)
      continue()
    endif()
    foreach(_dart_vulkan_candidate libvulkan.so libvulkan.so.1)
      set(_dart_vulkan_path "${_dart_vulkan_prefix}/lib/${_dart_vulkan_candidate}")
      if(EXISTS "${_dart_vulkan_path}")
        set(Vulkan_LIBRARY "${_dart_vulkan_path}" CACHE FILEPATH "Path to the Vulkan loader library" FORCE)
        break()
      endif()
    endforeach()
    if(Vulkan_LIBRARY)
      break()
    endif()
  endforeach()
  unset(_dart_vulkan_candidate)
  unset(_dart_vulkan_path)
  unset(_dart_vulkan_prefix)
  unset(_dart_vulkan_search_prefixes)
endif()

find_package(imgui CONFIG)

if(NOT imgui_FOUND)
  find_package(imgui REQUIRED MODULE)
endif()

if(imgui_FOUND AND NOT TARGET imgui::imgui)
  add_library(imgui::imgui INTERFACE IMPORTED)
  set_target_properties(
    imgui::imgui
    PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${imgui_INCLUDE_DIRS}" INTERFACE_LINK_LIBRARIES "${imgui_LIBRARIES}"
  )
endif()
