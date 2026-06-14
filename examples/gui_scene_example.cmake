# Copyright (c) 2011, The DART development contributors

set(DART_GUI_SCENE_EXAMPLE_DIR "${CMAKE_CURRENT_LIST_DIR}")

function(dart_build_gui_example target_name)
  if(NOT DART_IN_SOURCE_BUILD)
    list(APPEND CMAKE_MODULE_PATH "${DART_GUI_SCENE_EXAMPLE_DIR}/../cmake")
    find_package(DART 7.0.0 REQUIRED COMPONENTS gui CONFIG)
  endif()

  if(NOT TARGET dart-gui)
    if(DART_VERBOSE OR NOT DART_IN_SOURCE_BUILD)
      message(STATUS "Skipping ${target_name} example (requires dart-gui)")
    endif()
    return()
  endif()

  add_executable(${target_name} ${ARGN})
  target_link_libraries(${target_name} PRIVATE dart-gui)

  if(DART_IN_SOURCE_BUILD)
    set_target_properties(
      ${target_name}
      PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${DART_BINARY_DIR}/bin"
    )
    dart_add_example(${target_name})
    dart_format_add(${ARGN})
  endif()
endfunction()
