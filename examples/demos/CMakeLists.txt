project(dart-demos)

include("${CMAKE_CURRENT_LIST_DIR}/../gui_scene_example.cmake")

if(NOT TARGET dart-gui)
  return()
endif()

if(NOT TARGET dart-simulation)
  return()
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
  endif()
endif()
