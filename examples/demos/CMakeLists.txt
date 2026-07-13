project(dart-demos)

include("${CMAKE_CURRENT_LIST_DIR}/../gui_scene_example.cmake")

if(NOT TARGET dart-gui)
  return()
endif()

if(NOT TARGET dart-simulation)
  return()
endif()

# The diagnostic session, process probes, layout model, and panel are excluded
# from ordinary runtime artifacts. Test builds still compile the implementation
# so the default CI configuration retains its model/collector coverage, but the
# OFF dart-demos executable never links or calls it.
if(DART_BUILD_DEMOS_MEMORY_DIAGNOSTICS OR (BUILD_TESTING AND DART_BUILD_TESTS))
  set(
    demos_memory_diagnostics_sources
    "${CMAKE_CURRENT_LIST_DIR}/memory_diagnostics.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/memory_diagnostics_model.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/memory_map_model.cpp"
  )
  add_library(
    demos_memory_diagnostics
    STATIC
    ${demos_memory_diagnostics_sources}
  )
  target_include_directories(
    demos_memory_diagnostics
    PUBLIC "${CMAKE_CURRENT_LIST_DIR}"
  )
  target_link_libraries(
    demos_memory_diagnostics
    PUBLIC dart-gui dart-simulation
  )
  if(WIN32)
    target_link_libraries(demos_memory_diagnostics PRIVATE Psapi)
  endif()
  target_compile_features(demos_memory_diagnostics PUBLIC cxx_std_23)
  set_target_properties(
    demos_memory_diagnostics
    PROPERTIES POSITION_INDEPENDENT_CODE ON
  )
  if(DART_IN_SOURCE_BUILD AND COMMAND dart_format_add)
    dart_format_add(${demos_memory_diagnostics_sources})
  endif()
endif()

# Scene modules + the ordered registry, compiled into a static library so the
# dart-demos executable and cycle smoke consume the same scene set.
set(
  demos_scene_sources
  "${CMAKE_CURRENT_LIST_DIR}/registry.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/rigid_body.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/deformable_body.cpp"
)

add_library(demos_scenes STATIC ${demos_scene_sources})
target_include_directories(demos_scenes PUBLIC "${CMAKE_CURRENT_LIST_DIR}")
target_link_libraries(
  demos_scenes
  PUBLIC dart-gui dart-io dart-collision-native dart-simulation
)
if(DART_BUILD_DEMOS_MEMORY_DIAGNOSTICS)
  target_link_libraries(demos_scenes PRIVATE demos_memory_diagnostics)
  target_compile_definitions(
    demos_scenes
    PRIVATE DART_BUILD_DEMOS_MEMORY_DIAGNOSTICS=1
  )
endif()
# deformable_body.cpp compiles an internal (non-promoted) experimental header,
# detail/world_registry_access.hpp, which includes <entt/entt.hpp>. EnTT is a
# PRIVATE dependency of dart-simulation (the public surface is
# entt-free) and so is not propagated to consumers, so this scene fails to
# compile in clean builds where EnTT is supplied only via
# dart_find_entt.cmake/FetchContent rather than a global include directory.
# Link EnTT explicitly (PRIVATE: an implementation detail of this scene, not
# part of the demos_scenes interface) when it is available in this in-tree build.
if(TARGET EnTT::EnTT)
  target_link_libraries(demos_scenes PRIVATE EnTT::EnTT)
endif()
target_compile_definitions(
  demos_scenes
  PUBLIC DART_DEMOS_HAS_SIMULATION_EXPERIMENTAL
)
target_compile_features(demos_scenes PUBLIC cxx_std_23)
set_target_properties(demos_scenes PROPERTIES POSITION_INDEPENDENT_CODE ON)
if(DART_IN_SOURCE_BUILD AND COMMAND dart_format_add)
  dart_format_add(${demos_scene_sources})
endif()

# Thin app: only the entry point, linked against the scene library.
dart_build_gui_example(dart-demos "${CMAKE_CURRENT_LIST_DIR}/app/main.cpp")

if(TARGET dart-demos)
  target_link_libraries(dart-demos PRIVATE demos_scenes)

  if(BUILD_TESTING AND DART_BUILD_TESTS)
    add_test(NAME EXAMPLE_dart_demos_list COMMAND dart-demos --list)
    # dart-demos links Filament, whose prebuilt runtime dlopen's SDL3 at process
    # startup. Under AddressSanitizer that load fails ("Failed loading SDL3
    # library") and aborts before `--list` can print, so disable this runtime
    # smoke test in ASan builds. It still runs in Release/Debug, and dart-demos
    # is still built under ASan (the failure is purely at runtime startup).
    set_tests_properties(
      EXAMPLE_dart_demos_list
      PROPERTIES LABELS "examples;gui;catalog" DISABLED ${DART_ENABLE_ASAN}
    )
    if(NOT APPLE)
      if(WIN32)
        set(_dart_demos_path_var "PATH")
      else()
        set(_dart_demos_path_var "LD_LIBRARY_PATH")
      endif()
      set(
        _dart_demos_list_env_mods
        "${_dart_demos_path_var}=path_list_prepend:${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}"
        "${_dart_demos_path_var}=path_list_prepend:$<TARGET_FILE_DIR:dart>"
      )
      set_tests_properties(
        EXAMPLE_dart_demos_list
        PROPERTIES ENVIRONMENT_MODIFICATION "${_dart_demos_list_env_mods}"
      )
      unset(_dart_demos_list_env_mods)
      unset(_dart_demos_path_var)
    endif()
    if(DART_BUILD_DEMOS_MEMORY_DIAGNOSTICS)
      set(
        _dart_demos_memory_artifact_test
        EXAMPLE_dart_demos_memory_diagnostics_included
      )
      set(_dart_demos_memory_artifact_label "feature-enabled")
    else()
      set(
        _dart_demos_memory_artifact_test
        EXAMPLE_dart_demos_memory_diagnostics_excluded
      )
      set(_dart_demos_memory_artifact_label "zero-overhead")
    endif()
    add_test(
      NAME ${_dart_demos_memory_artifact_test}
      COMMAND
        "${CMAKE_COMMAND}" "-DDART_DEMOS_EXECUTABLE=$<TARGET_FILE:dart-demos>"
        "-DDART_EXPECT_MEMORY_DIAGNOSTICS=${DART_BUILD_DEMOS_MEMORY_DIAGNOSTICS}"
        "-DDART_NM_EXECUTABLE=${CMAKE_NM}" -P
        "${CMAKE_CURRENT_LIST_DIR}/tests/check_memory_diagnostics_artifact.cmake"
    )
    set_tests_properties(
      ${_dart_demos_memory_artifact_test}
      PROPERTIES
        LABELS
          "examples;gui;memory-diagnostics;${_dart_demos_memory_artifact_label}"
    )
    unset(_dart_demos_memory_artifact_label)
    unset(_dart_demos_memory_artifact_test)
    foreach(test_target IN ITEMS tests tests_and_run)
      if(TARGET ${test_target})
        add_dependencies(${test_target} dart-demos)
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

    # WP-ASV.4 camera-control smoke: one dart-demos process captures two distinct
    # --views plus a --turntable, asserting every per-view PPM is valid and
    # non-blank and that the two views are framed distinctly
    # (analyze_headless_smoke.py compare mode). dart-demos routes through
    # runGuiBackendApplication, the demos path that owns the camera flags.
    find_package(Python3 COMPONENTS Interpreter QUIET)
    if(Python3_Interpreter_FOUND)
      set(
        _dart_demos_camera_testing_dir
        "${DART_SOURCE_DIR}/dart/gui/detail/testing"
      )
      add_test(
        NAME EXAMPLE_dart_demos_camera_control_headless_smoke
        COMMAND
          "${CMAKE_COMMAND}"
          "-DDART_GUI_FILAMENT_EXECUTABLE=$<TARGET_FILE:dart-demos>"
          "-DDART_GUI_FILAMENT_SCREENSHOT=${CMAKE_CURRENT_BINARY_DIR}/dart_demos_camera_control.ppm"
          -DDART_GUI_FILAMENT_WIDTH=640 -DDART_GUI_FILAMENT_HEIGHT=480
          -DDART_GUI_FILAMENT_FRAMES=6
          -DDART_GUI_FILAMENT_VIEWS=front,three-quarter
          -DDART_GUI_FILAMENT_TURNTABLE=4
          "-DDART_GUI_FILAMENT_PYTHON=${Python3_EXECUTABLE}"
          "-DDART_GUI_FILAMENT_ANALYZER=${_dart_demos_camera_testing_dir}/analyze_headless_smoke.py"
          -P "${_dart_demos_camera_testing_dir}/run_camera_smoke.cmake"
      )
      set_tests_properties(
        EXAMPLE_dart_demos_camera_control_headless_smoke
        PROPERTIES LABELS "filament;gui;smoke;graphics"
      )
      if(UNIX AND NOT APPLE)
        set_tests_properties(
          EXAMPLE_dart_demos_camera_control_headless_smoke
          PROPERTIES
            ENVIRONMENT
              "LIBGL_ALWAYS_SOFTWARE=1;MESA_LOADER_DRIVER_OVERRIDE=llvmpipe"
        )
      endif()
    endif()
  endif()
endif()

if(
  BUILD_TESTING
  AND DART_BUILD_TESTS
  AND COMMAND dart_add_test
  AND EXISTS "${CMAKE_CURRENT_LIST_DIR}/tests/test_memory_diagnostics_model.cpp"
)
  # System-GTest imported targets are directory-scoped and the tests tree is
  # configured before examples, so recreate them in this directory when needed.
  if(NOT TARGET GTest::gtest)
    find_package(GTest MODULE REQUIRED)
  endif()
  dart_add_test(
    "unit"
    UNIT_examples_demos_MemoryDiagnostics
    "${CMAKE_CURRENT_LIST_DIR}/tests/test_memory_diagnostics_model.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/tests/test_memory_diagnostics.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/tests/test_memory_map_model.cpp"
  )
  target_link_libraries(
    UNIT_examples_demos_MemoryDiagnostics
    demos_memory_diagnostics
  )
  set_tests_properties(
    UNIT_examples_demos_MemoryDiagnostics
    PROPERTIES LABELS "examples;gui;memory-diagnostics;simulation"
  )
  foreach(test_target IN ITEMS tests tests_and_run)
    if(TARGET ${test_target})
      add_dependencies(${test_target} UNIT_examples_demos_MemoryDiagnostics)
    endif()
  endforeach()
endif()
