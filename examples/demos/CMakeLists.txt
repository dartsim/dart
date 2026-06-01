cmake_minimum_required(VERSION 3.22.1)

project(dart-demos)

include("${CMAKE_CURRENT_LIST_DIR}/../gui_scene_example.cmake")

if(NOT TARGET dart-gui)
  return()
endif()

# Scene modules + the ordered registry, compiled into a static library so the
# dart-demos executable and tests/unit/gui/UNIT_demos_golden_parity can both
# consume the same scene set without rebuilding it.
set(
  demos_scene_sources
  "${CMAKE_CURRENT_LIST_DIR}/registry.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/hello_world.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/empty.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/shapes.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/simple_frames.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/drag_and_drop.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/polyhedron_visual.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/boxes.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/rigid_cubes.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/add_delete_skels.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/capsule_ground_contact.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/hardcoded_design.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/box_stacking.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/hybrid_dynamics.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/vehicle.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/imgui.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/simulation_event_handler.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/mixed_chain.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/coupler_constraint.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/mimic_pendulums.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/rigid_chain.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/rigid_loop.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/free_joint_cases.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/joint_constraints.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/soft_bodies.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/rigid_shapes.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/heightmap.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/lcp_physics.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/biped_stand.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/tinkertoy.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/operational_space_control.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/point_cloud.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/fetch.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/atlas_puppet.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/hubo_puppet.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/g1_puppet.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/human_joint_limits.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/collision_sandbox.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/collision_sandbox/pair_registry.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/atlas_simbicon.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/atlas_simbicon/controller.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/atlas_simbicon/state.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/atlas_simbicon/state_machine.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/atlas_simbicon/terminal_condition.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/wam_ikfast.cpp"
)

# Scenes that depend on the experimental simulation module (dart::simulation::
# experimental). They are compiled and registered only when that optional module
# is built, since CI and downstreams can configure with
# DART_BUILD_SIMULATION_EXPERIMENTAL=OFF.
set(
  demos_experimental_scene_sources
  "${CMAKE_CURRENT_LIST_DIR}/scenes/experimental_rigid_body_gui.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/experimental_deformable.cpp"
)

add_library(demos_scenes STATIC ${demos_scene_sources})
target_include_directories(demos_scenes PUBLIC "${CMAKE_CURRENT_LIST_DIR}")
target_link_libraries(
  demos_scenes
  PUBLIC dart-gui dart-io dart-collision-native
)
if(TARGET dart-simulation-experimental)
  target_sources(demos_scenes PRIVATE ${demos_experimental_scene_sources})
  target_link_libraries(demos_scenes PUBLIC dart-simulation-experimental)
  target_compile_definitions(
    demos_scenes
    PUBLIC DART_DEMOS_HAS_SIMULATION_EXPERIMENTAL
  )
endif()
target_compile_features(demos_scenes PUBLIC cxx_std_20)
set_target_properties(demos_scenes PROPERTIES POSITION_INDEPENDENT_CODE ON)
if(DART_IN_SOURCE_BUILD AND COMMAND dart_format_add)
  dart_format_add(
    ${demos_scene_sources} ${demos_experimental_scene_sources}
  )
endif()

# Thin app: only the entry point, linked against the scene library.
dart_build_gui_example(dart-demos "${CMAKE_CURRENT_LIST_DIR}/app/main.cpp")

if(TARGET dart-demos)
  target_link_libraries(dart-demos PRIVATE demos_scenes)

  # Cross-language golden-set parity smoke (PLAN-103 Phase 2): asserts the
  # C++ hello_world demo scene matches the shared fixture also asserted by
  # python/tests/unit/test_golden_parity.py. Plain main() so no GTest scope
  # dependency from examples/.
  if(BUILD_TESTING AND DART_BUILD_TESTS)
    add_executable(
      UNIT_gui_DemosGoldenParity
      "${CMAKE_SOURCE_DIR}/tests/unit/gui/test_demos_golden_parity.cpp"
    )
    target_link_libraries(UNIT_gui_DemosGoldenParity PRIVATE demos_scenes)
    target_compile_features(UNIT_gui_DemosGoldenParity PRIVATE cxx_std_20)
    add_test(NAME UNIT_gui_DemosGoldenParity COMMAND UNIT_gui_DemosGoldenParity)
    set_tests_properties(
      UNIT_gui_DemosGoldenParity
      PROPERTIES LABELS "unit;gui;demos"
    )
    if(NOT APPLE)
      if(WIN32)
        set(_demos_golden_lib_path_var "PATH")
        set(_demos_golden_runtime_paths)
        if(DEFINED ENV{CONDA_PREFIX})
          list(
            APPEND _demos_golden_runtime_paths
            "$ENV{CONDA_PREFIX}/Library/bin"
            "$ENV{CONDA_PREFIX}/bin"
          )
        endif()
      else()
        set(_demos_golden_lib_path_var "LD_LIBRARY_PATH")
        set(_demos_golden_runtime_paths)
        if(DEFINED ENV{CONDA_PREFIX})
          list(APPEND _demos_golden_runtime_paths "$ENV{CONDA_PREFIX}/lib")
        endif()
      endif()
      list(APPEND _demos_golden_runtime_paths "$<TARGET_FILE_DIR:dart>")
      set(_demos_golden_env_mods)
      foreach(_demos_golden_runtime_path IN LISTS _demos_golden_runtime_paths)
        if(_demos_golden_runtime_path)
          list(
            APPEND _demos_golden_env_mods
            "${_demos_golden_lib_path_var}=path_list_prepend:${_demos_golden_runtime_path}"
          )
        endif()
      endforeach()
      set_tests_properties(
        UNIT_gui_DemosGoldenParity
        PROPERTIES ENVIRONMENT_MODIFICATION "${_demos_golden_env_mods}"
      )
    endif()
    # The aggregate `tests` target (tests/CMakeLists.txt) is finalized before
    # examples/ is added, so it cannot pick up this example-tree test through the
    # DART_unit_TESTS global property. Attach it explicitly so `pixi run
    # build-tests` (`--target tests`) builds the binary; otherwise ctest reports
    # it "Not Run".
    foreach(_demos_golden_aggregate tests tests_and_run)
      if(TARGET ${_demos_golden_aggregate})
        add_dependencies(${_demos_golden_aggregate} UNIT_gui_DemosGoldenParity)
      endif()
    endforeach()
  endif()

  # Headless smoke: cycle through every registered scene a few frames each,
  # exercising scene build, render, and the runtime scene switch/teardown. Uses
  # the same software-GL environment as the other GUI Filament smoke tests.
  if(
    BUILD_TESTING
    AND DART_BUILD_TESTS
    AND DART_ENABLE_GUI_FILAMENT_SMOKE_TESTS
  )
    add_test(
      NAME EXAMPLE_dart_demos_cycle_headless_smoke
      COMMAND dart-demos --cycle-scenes --headless --width 320 --height 240
    )
    set_tests_properties(
      EXAMPLE_dart_demos_cycle_headless_smoke
      PROPERTIES LABELS "filament;gui;smoke;graphics"
    )
    if(UNIX AND NOT APPLE)
      set_tests_properties(
        EXAMPLE_dart_demos_cycle_headless_smoke
        PROPERTIES
          ENVIRONMENT
            "LIBGL_ALWAYS_SOFTWARE=1;MESA_LOADER_DRIVER_OVERRIDE=llvmpipe"
      )
    endif()
  endif()
endif()
