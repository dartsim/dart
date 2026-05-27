cmake_minimum_required(VERSION 3.22.1)

project(dart-examples)

include("${CMAKE_CURRENT_LIST_DIR}/gui_scene_example.cmake")

add_subdirectory(unified_loading)
if(TARGET dart-gui OR NOT DART_IN_SOURCE_BUILD)
  add_subdirectory(gui_scene_diagnostics)
endif()
add_subdirectory(headless_simulation)
add_subdirectory(speed_test)
add_subdirectory(csv_logger)

if(DART_BUILD_GUI OR NOT DART_IN_SOURCE_BUILD)
  # hello_world stays standalone as the minimal "link DART in a CMake project"
  # template. Other GUI examples are consolidated into the dart-demos app
  # (examples/demos); each is a registered scene switchable at runtime.
  add_subdirectory(hello_world)
  add_subdirectory(demos)

  # Kept standalone (not in dart-demos). Tracked in docs/dev_tasks/demos_app/:
  #   - atlas_simbicon / collision_sandbox: their controller and pair-registry
  #     sources are reused by tests (tests/integration, tests/unit/collision),
  #     so they stay multi-file standalone examples.
  #   - wam_ikfast: depends on a generated IKFast library selected via
  #     DART_WAM_IKFAST_LIB_PATH at configure time.
  #   - experimental_deformable_gui: newly landed upstream (PR #2711);
  #     migrating it into dart-demos is tracked in
  #     docs/dev_tasks/examples_strategy/RESUME.md.
  add_subdirectory(atlas_simbicon)
  add_subdirectory(collision_sandbox)
  add_subdirectory(experimental_deformable_gui)
  add_subdirectory(wam_ikfast)
endif()
