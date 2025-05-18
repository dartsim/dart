# Copyright (c) 2011-2025, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

find_package(imgui CONFIG)

if(NOT imgui_FOUND)
  find_package(imgui REQUIRED MODULE)
endif()

if(imgui_FOUND AND NOT TARGET imgui::imgui)
  add_library(imgui::imgui INTERFACE IMPORTED)
  set_target_properties(imgui::imgui PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${imgui_INCLUDE_DIRS}"
    INTERFACE_LINK_LIBRARIES "${imgui_LIBRARIES}"
  )
endif()