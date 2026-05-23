cmake_minimum_required(VERSION 3.22.1)

get_filename_component(example_name ${CMAKE_CURRENT_LIST_DIR} NAME)
project(${example_name})

include("${CMAKE_CURRENT_LIST_DIR}/../gui_scene_example.cmake")
dart_build_gui_example(
  ${example_name}
  "${CMAKE_CURRENT_LIST_DIR}/main.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/pair_registry.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/pair_registry.hpp"
)

if(TARGET ${example_name})
  target_link_libraries(${example_name} PRIVATE dart-collision-native)

  if(DART_IN_SOURCE_BUILD AND BUILD_TESTING AND DART_BUILD_TESTS AND DART_ENABLE_GUI_FILAMENT_SMOKE_TESTS)
    add_test(
      NAME EXAMPLE_collision_sandbox_headless_sweep
      COMMAND
        ${CMAKE_COMMAND}
        "-DDART_COLLISION_SANDBOX_EXECUTABLE=$<TARGET_FILE:${example_name}>"
        "-DDART_COLLISION_SANDBOX_OUTPUT_DIR=${CMAKE_CURRENT_BINARY_DIR}/smoke"
        -P "${CMAKE_CURRENT_LIST_DIR}/run_headless_sweep.cmake"
    )
    set_tests_properties(
      EXAMPLE_collision_sandbox_headless_sweep
      PROPERTIES LABELS "gui;filament;collision-native"
    )
  endif()
endif()
