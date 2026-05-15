# Copyright (c) 2011, The DART development contributors

set(DART_GUI_SCENE_EXAMPLE_DIR "${CMAKE_CURRENT_LIST_DIR}")
set(DART_GUI_SCENE_EXAMPLE_LAUNCHER
    "${DART_GUI_SCENE_EXAMPLE_DIR}/gui_scene_launcher.cpp"
)

function(dart_build_gui_scene_example target_name scene_name)
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

  add_executable(${target_name} "${DART_GUI_SCENE_EXAMPLE_LAUNCHER}")
  target_link_libraries(${target_name} PRIVATE dart-gui)
  target_compile_features(${target_name} PRIVATE cxx_std_20)
  target_compile_definitions(
    ${target_name}
    PRIVATE DART_GUI_DEFAULT_SCENE="${scene_name}"
  )

  if(DART_IN_SOURCE_BUILD)
    set_target_properties(
      ${target_name}
      PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${DART_BINARY_DIR}/bin"
    )
    dart_add_example(${target_name})
    dart_format_add("${DART_GUI_SCENE_EXAMPLE_LAUNCHER}")
  endif()
endfunction()
