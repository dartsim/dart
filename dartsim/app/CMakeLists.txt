# Copyright (c) 2011, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License.

# The dartsim executable entry point. Reusable engine/UI code lives in
# ../engine and ../ui; this directory holds only main.cpp so it can use the
# backend-hidden GUI application macro (which requires a single main.cpp).

set(application_target dartsim)

if(NOT DART_IN_SOURCE_BUILD)
  list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../../cmake")
  find_package(DART 7.0.0 QUIET COMPONENTS gui CONFIG)
  if(NOT DART_FOUND)
    find_package(DART 7.0.0 REQUIRED COMPONENTS gui CONFIG)
  endif()
endif()

include("${CMAKE_CURRENT_LIST_DIR}/../../dart/gui/detail/backend_sources.cmake")

dart_gui_add_application(
  ${application_target}
  GENERATED_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated"
  OUTPUT_NAME "dartsim"
  SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}"
)

# Wire the editor: main.cpp calls dartsim::ui::runEditor from the dartsim_ui
# library (which links the headless dartsim_engine).
if(TARGET ${application_target} AND TARGET dartsim_ui)
  target_link_libraries(${application_target} PRIVATE dartsim_ui)
  # Build on demand (mirrors the prior application target); packaging builds it
  # explicitly when producing the runtime executable distribution.
  set_target_properties(${application_target} PROPERTIES EXCLUDE_FROM_ALL TRUE)
endif()
