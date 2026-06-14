cmake_minimum_required(VERSION 4.2.3)

get_filename_component(example_name ${CMAKE_CURRENT_LIST_DIR} NAME)
project(${example_name})

if(NOT DART_IN_SOURCE_BUILD)
  list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../cmake")
  find_package(DART 7.0.0 QUIET COMPONENTS gui simulation CONFIG)
endif()

# This demo additionally requires the opt-in differentiable build
# (DART_BUILD_DIFF / DART_HAS_DIFF). The differentiable sources compile into the
# simulation target, so there is no separate diff target to query; the guard
# checks the DART_HAS_DIFF compile feature, which is set on
# dart-simulation when DART_BUILD_DIFF is ON. Guarding before
# registration keeps GUI-OFF, simulation-OFF, and diff-OFF CI configs clean.
set(_dart_diff_available FALSE)
if(TARGET dart-simulation)
  get_target_property(
    _dart_simulation_definitions
    dart-simulation
    INTERFACE_COMPILE_DEFINITIONS
  )
  if(_dart_simulation_definitions MATCHES "DART_HAS_DIFF")
    set(_dart_diff_available TRUE)
  endif()
endif()

if(
  NOT TARGET dart-gui
  OR NOT TARGET dart-simulation
  OR NOT _dart_diff_available
)
  if(DART_VERBOSE OR NOT DART_IN_SOURCE_BUILD)
    message(
      STATUS
      "Skipping ${example_name} example (requires dart-gui, dart-simulation, and DART_BUILD_DIFF)"
    )
  endif()
  return()
endif()

include("${CMAKE_CURRENT_LIST_DIR}/../gui_scene_example.cmake")
dart_build_gui_example(${example_name} main.cpp)

if(TARGET ${example_name})
  target_link_libraries(${example_name} PRIVATE dart-simulation)

  if(
    DART_IN_SOURCE_BUILD
    AND BUILD_TESTING
    AND DART_BUILD_TESTS
    AND DART_ENABLE_GUI_FILAMENT_SMOKE_TESTS
  )
    add_test(
      NAME EXAMPLE_differentiable_gui_headless_smoke
      COMMAND
        ${CMAKE_COMMAND}
        "-DDART_DIFFERENTIABLE_GUI_EXECUTABLE=$<TARGET_FILE:${example_name}>"
        "-DDART_DIFFERENTIABLE_GUI_OUTPUT_DIR=${CMAKE_CURRENT_BINARY_DIR}/smoke"
        -P "${CMAKE_CURRENT_LIST_DIR}/run_headless_smoke.cmake"
    )
    set_tests_properties(
      EXAMPLE_differentiable_gui_headless_smoke
      PROPERTIES LABELS "gui;filament;simulation;diff"
    )
  endif()
endif()
