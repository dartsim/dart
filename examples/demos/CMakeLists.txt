cmake_minimum_required(VERSION 3.22.1)

set(example_name dart-demos)

project(${example_name})

function(dart_demos_check_memory_diagnostics_compiled_out target)
  if(NOT DART_BUILD_DEMOS_MEMORY_DIAGNOSTICS)
    add_custom_command(
      TARGET ${target}
      POST_BUILD
      COMMAND
        ${CMAKE_COMMAND} "-DDART_DEMOS_EXECUTABLE=$<TARGET_FILE:${target}>" -P
        "${DART_SOURCE_DIR}/dart/gui/app/tests/check_memory_diagnostics_compiled_out.cmake"
      COMMENT "Verifying memory diagnostics are absent from ${target}"
      VERBATIM
    )
  endif()
endfunction()

set(
  required_libraries
  dart
  dart-utils
  dart-utils-urdf
  dart-gui
  dart-demo-models
  dart-gui-app
)

function(dart_demos_collect_scene_srcs out_var)
  file(GLOB _srcs "scenes/*.cpp" "scenes/*.hpp")
  if(NOT DART_DEMOS_HAVE_MODEL_ASSETS)
    list(FILTER _srcs EXCLUDE REGEX "ModernHumanoidsScene\\.cpp$")
  endif()
  file(GLOB _tinkertoy_srcs "scenes/tinkertoy/*.cpp" "scenes/tinkertoy/*.hpp")
  list(APPEND _srcs ${_tinkertoy_srcs})
  file(
    GLOB _wam_ikfast_srcs
    "scenes/wam_ikfast/*.cpp"
    "scenes/wam_ikfast/*.hpp"
  )
  list(APPEND _srcs ${_wam_ikfast_srcs})
  file(
    GLOB _atlas_simbicon_srcs
    "scenes/atlas_simbicon/*.cpp"
    "scenes/atlas_simbicon/*.hpp"
  )
  file(
    GLOB _soft_foot_simbicon_srcs
    "scenes/soft_foot_simbicon/*.cpp"
    "scenes/soft_foot_simbicon/*.hpp"
  )
  list(APPEND _srcs ${_atlas_simbicon_srcs})
  list(APPEND _srcs ${_soft_foot_simbicon_srcs})
  if(DART_DEMOS_HAVE_TINY_DNN)
    file(
      GLOB _human_joint_limits_srcs
      "scenes/human_joint_limits/*.cpp"
      "scenes/human_joint_limits/*.hpp"
    )
    list(APPEND _srcs ${_human_joint_limits_srcs})
  endif()
  if(DART_DEMOS_HAVE_PYTHON)
    file(
      GLOB _ssik_ik_gui_srcs
      "scenes/ssik_ik_gui/*.cpp"
      "scenes/ssik_ik_gui/*.hpp"
    )
    list(APPEND _srcs ${_ssik_ik_gui_srcs})
  endif()
  set(${out_var} ${_srcs} PARENT_SCOPE)
endfunction()

if(DART_IN_SOURCE_BUILD AND NOT TARGET dart-gui)
  # Without the GUI, lint still formats the demos and their tests, including
  # optional scenes whose dependencies formatting does not need.
  file(GLOB _demos_format_files "*.cpp" "*.hpp" "tests/*.cpp" "tests/*.hpp")
  set(DART_DEMOS_HAVE_MODEL_ASSETS TRUE)
  set(DART_DEMOS_HAVE_TINY_DNN TRUE)
  set(DART_DEMOS_HAVE_PYTHON TRUE)
  dart_demos_collect_scene_srcs(_demos_scene_format_files)
  dart_format_add(${_demos_format_files} ${_demos_scene_format_files})
  return()
endif()

if(NOT DART_IN_SOURCE_BUILD)
  message(
    STATUS
    "dart-demos builds only in-source because it links the private dart-demo-models"
  )
  return()
endif()

# Installed targets can exist even when their optional dependencies are missing.
set(DART_DEMOS_HAVE_MODEL_ASSETS FALSE)
if(
  TARGET dart-utils-assets
  AND (DART_IN_SOURCE_BUILD OR DART_utils-assets_FOUND)
)
  set(DART_DEMOS_HAVE_MODEL_ASSETS TRUE)
endif()

# Link the Bullet collision backend when it is available so the scenes that opt
# into it at runtime (canCreate("bullet"): AddDeleteSkelsScene,
# RigidShapesScene, and shapes.skel's own <collision_detector>bullet</...>)
# actually get a Bullet detector; the scenes still guard the lookup, so this
# stays optional. In an in-source build the backend shows up as the
# dart-collision-bullet target; standalone, it is an optional DART component.
#
# dart-demos reaches Bullet only through the runtime, string-keyed
# collision-detector factory -- it never references a Bullet symbol -- so a
# default --as-needed link would drop the library and its self-registration
# would never run (canCreate("bullet") would stay false). The caller links the
# dart-collision-bullet target (using whichever target_link_libraries signature
# it already uses for the executable); this helper adds the flag that forces the
# library to stay linked on GNU/Clang. Apple/MSVC keep an explicitly linked
# library regardless, and target_link_options is signature-neutral so it is safe
# to mix with either a plain or keyword target_link_libraries call.
function(force_link_collision_backend target)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang" AND NOT APPLE)
    target_link_options(${target} PRIVATE "LINKER:--no-as-needed")
  endif()
endfunction()

function(dart_demos_link_optional_collision_backends target)
  foreach(collision_backend dart-collision-bullet dart-collision-ode)
    if(TARGET ${collision_backend})
      target_link_libraries(${target} ${collision_backend})
      force_link_collision_backend(${target})
    endif()
  endforeach()
endfunction()

# Optional TinyDNN + Threads dependency for the human_joint_limits scene
# (scenes/human_joint_limits/), mirroring examples/human_joint_limits/
# CMakeLists.txt's own optional-dependency guard. When unavailable, that
# scene's sources are excluded from the build entirely and the registry
# (Registry.cpp, behind the same DART_DEMOS_HAVE_TINY_DNN definition) omits
# the entry rather than showing a broken row -- the same pattern used
# elsewhere in this catalog for optional deps (e.g. HAVE_OCTOMAP, ikfast).
set(TinyDNN_USE_SERIALIZER ON)
find_package(TinyDNN QUIET)
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads QUIET)
set(DART_DEMOS_HAVE_TINY_DNN FALSE)
if(TinyDNN_FOUND AND Threads_FOUND)
  set(DART_DEMOS_HAVE_TINY_DNN TRUE)
endif()

# Optional Python3 embedding for the ssik_ik_gui scene, mirroring
# examples/ssik_ik_gui/CMakeLists.txt's own optional-dependency guard. The
# top-level DART configure already searches for Python3 (see
# cmake/DARTFindDependencies.cmake), so this just re-reads that cached
# result; when absent, the scene's sources are excluded from the build
# entirely and the registry (behind the same DART_DEMOS_HAVE_PYTHON
# definition) omits the row rather than showing a broken one -- the same
# pattern used above for TinyDNN.
find_package(Python3 3.10 COMPONENTS Interpreter Development QUIET)
set(DART_DEMOS_HAVE_PYTHON FALSE)
if(Python3_FOUND)
  set(DART_DEMOS_HAVE_PYTHON TRUE)
endif()

# Globs every scene source, including one level of scene subdirectories (for
# multi-file scenes, e.g. scenes/tinkertoy/), and conditionally adds
# scenes/human_joint_limits/ only when TinyDNN + Threads were found. Used by
# the in-source build below.

# Links the optional TinyDNN + Threads and Python3 dependencies and defines
# DART_DEMOS_HAVE_TINY_DNN / DART_DEMOS_HAVE_PYTHON when found. Used by the
# in-source build below.
function(dart_demos_configure_scenes_target target)
  if(DART_DEMOS_HAVE_MODEL_ASSETS)
    target_link_libraries(${target} PUBLIC dart-utils-assets)
    target_compile_definitions(${target} PUBLIC DART_DEMOS_HAVE_MODEL_ASSETS)
  endif()
  if(DART_DEMOS_HAVE_TINY_DNN)
    target_link_libraries(
      ${target}
      PUBLIC ${TinyDNN_LIBRARIES} Threads::Threads
    )
    target_compile_definitions(${target} PUBLIC DART_DEMOS_HAVE_TINY_DNN)
  endif()
  if(DART_DEMOS_HAVE_PYTHON)
    target_link_libraries(${target} PUBLIC Python3::Python)
    target_compile_definitions(${target} PUBLIC DART_DEMOS_HAVE_PYTHON)
  endif()
endfunction()

file(GLOB srcs "*.cpp" "*.hpp")
# The scene ports (examples/demos/scenes/*.cpp) live in a subdirectory, so they
# cannot be picked up by dart_build_target_in_source()'s non-recursive
# `file(GLOB srcs "*.cpp" "*.hpp")`. Compile them into a small static library
# instead and link it into the dart-demos executable below, so the mandated
# dart_build_example_in_source() in-source path can still be used unmodified for
# the executable itself.
if(DART_IN_SOURCE_BUILD)
  foreach(dep_target ${required_libraries})
    if(NOT TARGET ${dep_target})
      if(DART_VERBOSE)
        message(
          WARNING
          "Skipping ${example_name} because required target '${dep_target}' not found"
        )
      endif()
      return()
    endif()
  endforeach()

  dart_demos_collect_scene_srcs(demos_scene_srcs)
  add_library(dart-demos-scenes STATIC ${demos_scene_srcs})
  target_link_libraries(dart-demos-scenes PUBLIC ${required_libraries})
  target_include_directories(
    dart-demos-scenes
    PUBLIC "${CMAKE_CURRENT_LIST_DIR}"
  )
  dart_demos_configure_scenes_target(dart-demos-scenes)
  dart_format_add(${demos_scene_srcs})

  dart_build_example_in_source(
    ${example_name}
    LINK_LIBRARIES ${required_libraries} dart-demos-scenes
  )

  if(TARGET ${example_name})
    dart_demos_link_optional_collision_backends(${example_name})
    dart_demos_check_memory_diagnostics_compiled_out(${example_name})
  endif()

  foreach(alias_scene soft_bodies soft_cubes soft_open_chain)
    add_executable(${alias_scene} ${srcs})
    set_target_properties(
      ${alias_scene}
      PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    )
    target_compile_definitions(
      ${alias_scene}
      PRIVATE DART_DEMOS_DEFAULT_SCENE="${alias_scene}"
    )
    target_link_libraries(
      ${alias_scene}
      ${required_libraries}
      dart-demos-scenes
    )
    dart_demos_link_optional_collision_backends(${alias_scene})
    dart_demos_check_memory_diagnostics_compiled_out(${alias_scene})
  endforeach()

  if(TARGET examples)
    add_dependencies(examples soft_bodies soft_cubes soft_open_chain)
  endif()
endif()

if(BUILD_TESTING AND TARGET dart-demos)
  # Its only remaining case needs Linux and the model assets; elsewhere the
  # executable would run zero tests.
  if(
    COMMAND dart_add_test
    AND CMAKE_SYSTEM_NAME STREQUAL "Linux"
    AND DART_DEMOS_HAVE_MODEL_ASSETS
  )
    if(NOT TARGET GTest::gtest)
      find_package(GTest MODULE REQUIRED)
    endif()
    dart_add_test(
      "unit"
      UNIT_examples_demos_SceneCycle
      tests/test_scene_cycle.cpp
    )
    target_link_libraries(UNIT_examples_demos_SceneCycle dart-demos-scenes)
    dart_demos_link_optional_collision_backends(UNIT_examples_demos_SceneCycle)
    set_tests_properties(
      UNIT_examples_demos_SceneCycle
      PROPERTIES LABELS "examples;gui"
    )
    if(TARGET tests)
      add_dependencies(tests UNIT_examples_demos_SceneCycle)
    endif()
    if(TARGET tests_and_run)
      add_dependencies(tests_and_run UNIT_examples_demos_SceneCycle)
    endif()
  elseif(COMMAND dart_format_add)
    dart_format_add(tests/test_scene_cycle.cpp)
  endif()
  if(TARGET tests)
    add_dependencies(tests dart-demos)
  endif()
  if(TARGET tests_and_run)
    add_dependencies(tests_and_run dart-demos)
  endif()
  add_test(
    NAME examples_demos_cycle
    COMMAND dart-demos --cycle-scenes --frames 2
  )
  add_test(
    NAME examples_demos_cycle_render
    COMMAND
      dart-demos --cycle-scenes --headless --frames 2 --width 160 --height 120
  )
  add_test(
    NAME examples_demos_capture
    COMMAND
      dart-demos --headless --scene hello_world --steps 5 --width 320 --height
      240 --shot "${CMAKE_CURRENT_BINARY_DIR}/hello-world.png"
  )
  set_tests_properties(examples_demos_cycle PROPERTIES LABELS "examples;gui")
  set_tests_properties(
    examples_demos_capture
    examples_demos_cycle_render
    PROPERTIES LABELS "examples;gui;gui-device;headless"
  )
endif()
