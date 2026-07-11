cmake_minimum_required(VERSION 3.22.1)

set(example_name dart-demos)

project(${example_name})

set(required_components utils-urdf gui-osg)
set(
  required_libraries
  dart
  dart-utils
  dart-utils-urdf
  dart-gui-osg
  dart_demos_wam_ikfast
)

if(DART_IN_SOURCE_BUILD AND NOT TARGET dart-gui-osg)
  dart_build_example_in_source(
    ${example_name}
    LINK_LIBRARIES ${required_libraries}
  )
  return()
endif()

if(NOT DART_IN_SOURCE_BUILD)
  find_package(DART 6.17 REQUIRED COMPONENTS ${required_components} CONFIG)
  find_package(DART 6.17 QUIET COMPONENTS collision-bullet collision-ode CONFIG)
endif()

# Single owner of where the wam_ikfast scene's dlopen'd IK shared library is
# written: this demos project's own build dir. Used both as the ikfast target's
# LIBRARY_OUTPUT_DIRECTORY (in scenes/wam_ikfast/ikfast/CMakeLists.txt, which
# inherits this variable) and as WamIkFastScene.cpp's DART_DEMOS_IKFAST_DIR
# compile definition, so the two are identical by construction in both the
# in-source and standalone build paths -- no dependence on DART_BINARY_DIR
# (which DARTConfig.cmake does not export, so it would be undefined standalone,
# collapsing the compile definition to "" and the lookup to "/libdartDemosWamIk.so").
# Defined before the add_subdirectory below so the child scope inherits it.
set(dart_demos_ikfast_output_dir "${CMAKE_CURRENT_BINARY_DIR}")

# Builds the wam_ikfast scene's dlopen'd analytical-IK shared library. Added
# unconditionally (no optional-dependency guard): it only depends on `dart`
# itself (transitively bringing in LAPACK). Standalone builds discover DART
# before adding this subdirectory so target usage requirements are available to
# the generated source, same as examples/wam_ikfast's own ikfast subdirectory,
# which this mirrors -- see
# scenes/wam_ikfast/ikfast/CMakeLists.txt for why it is a standalone copy
# rather than a dependency on that example.
add_subdirectory(scenes/wam_ikfast/ikfast)

# DynamicJointConstraintsScene.cpp draws its on-screen catalog labels with
# osgText::Text directly (ported from examples/dynamic_joint_constraints,
# which links osgText the same way -- dart-gui-osg only links the core
# osg::osg component, not osgText). Mirrors that example's own CMake pattern
# for locating/wrapping the osgText library as an imported target.
find_package(OpenSceneGraph 3.0.0 REQUIRED COMPONENTS osgText)
set(dart_demos_osg_text_library ${OSGTEXT_LIBRARY})
if(NOT dart_demos_osg_text_library)
  set(dart_demos_osg_text_library ${OSGTEXT_LIBRARY_RELEASE})
endif()
if(NOT dart_demos_osg_text_library)
  message(FATAL_ERROR "Could NOT find osgText")
endif()
if(NOT TARGET osgText::osgText)
  add_library(osgText::osgText UNKNOWN IMPORTED)
  set_target_properties(
    osgText::osgText
    PROPERTIES
      IMPORTED_LOCATION "${dart_demos_osg_text_library}"
      INTERFACE_INCLUDE_DIRECTORIES "${OSGTEXT_INCLUDE_DIR}"
  )
endif()
list(APPEND required_libraries osgText::osgText)

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

function(dart_demos_link_memory_diagnostics_platform target)
  if(WIN32)
    target_link_libraries(${target} Psapi)
  endif()
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
find_package(Python3 COMPONENTS Interpreter Development QUIET)
set(DART_DEMOS_HAVE_PYTHON FALSE)
if(Python3_FOUND)
  set(DART_DEMOS_HAVE_PYTHON TRUE)
endif()

# Globs every scene source, including one level of scene subdirectories (for
# multi-file scenes, e.g. scenes/tinkertoy/), and conditionally adds
# scenes/human_joint_limits/ only when TinyDNN + Threads were found. Shared by
# both the in-source and standalone build paths below.
function(dart_demos_collect_scene_srcs out_var)
  file(GLOB _srcs "scenes/*.cpp" "scenes/*.hpp")
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
  list(APPEND _srcs ${_atlas_simbicon_srcs})
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

# Links the optional TinyDNN + Threads and Python3 dependencies and defines
# DART_DEMOS_HAVE_TINY_DNN / DART_DEMOS_HAVE_PYTHON when found. Shared by both
# build paths below.
function(dart_demos_configure_scenes_target target)
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

  # Bakes in an absolute path to where scenes/wam_ikfast/ikfast's shared
  # library lands (dart_demos_ikfast_output_dir, the same variable that sets
  # its LIBRARY_OUTPUT_DIRECTORY) so WamIkFastScene.cpp's SharedLibraryIkFast
  # lookup does not depend on the running process's current working directory:
  # dart::common::SharedLibraryManager::load() resolves the name it is given
  # via std::ifstream(path).good() rather than dlopen()'s own RPATH-aware
  # search, so a bare filename ("libdartDemosWamIk.so") only resolves when
  # the process happens to be launched from that directory itself.
  target_compile_definitions(
    ${target}
    PRIVATE DART_DEMOS_IKFAST_DIR="${dart_demos_ikfast_output_dir}"
  )
endfunction()

file(GLOB srcs "*.cpp" "*.hpp")

# The demo helper globs top-level sources internally, so these sources compile
# directly into each executable. This small library gives the focused tests a
# single link owner for the same collector/view and shared C++17 model.
set(
  demos_memory_diagnostics_sources
  "${CMAKE_CURRENT_LIST_DIR}/memory_diagnostics.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/memory_diagnostics_model.cpp"
)
add_library(demos_memory_diagnostics STATIC ${demos_memory_diagnostics_sources})
target_include_directories(
  demos_memory_diagnostics
  PUBLIC "${CMAKE_CURRENT_LIST_DIR}"
)
target_link_libraries(demos_memory_diagnostics PUBLIC dart dart-gui-osg)
if(WIN32)
  target_link_libraries(demos_memory_diagnostics PRIVATE Psapi)
endif()
target_compile_features(demos_memory_diagnostics PUBLIC cxx_std_17)
set_target_properties(
  demos_memory_diagnostics
  PROPERTIES POSITION_INDEPENDENT_CODE ON
)
if(DART_IN_SOURCE_BUILD AND COMMAND dart_format_add)
  dart_format_add(${demos_memory_diagnostics_sources})
endif()

if(
  BUILD_TESTING
  AND COMMAND dart_add_test
  AND EXISTS "${CMAKE_CURRENT_LIST_DIR}/tests/test_memory_diagnostics_model.cpp"
)
  if(NOT TARGET GTest::gtest)
    find_package(GTest MODULE REQUIRED)
  endif()
  dart_add_test(
    "unit"
    UNIT_examples_demos_MemoryDiagnostics
    "${CMAKE_CURRENT_LIST_DIR}/tests/test_memory_diagnostics_model.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/tests/test_memory_diagnostics.cpp"
  )
  target_link_libraries(
    UNIT_examples_demos_MemoryDiagnostics
    demos_memory_diagnostics
  )
  set_tests_properties(
    UNIT_examples_demos_MemoryDiagnostics
    PROPERTIES LABELS "examples;gui;memory-diagnostics"
  )
  if(TARGET tests)
    add_dependencies(tests UNIT_examples_demos_MemoryDiagnostics)
  endif()
  if(TARGET tests_and_run)
    add_dependencies(tests_and_run UNIT_examples_demos_MemoryDiagnostics)
  endif()
endif()

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
  dart_demos_configure_scenes_target(dart-demos-scenes)
  dart_format_add(${demos_scene_srcs})

  dart_build_example_in_source(
    ${example_name}
    LINK_LIBRARIES ${required_libraries} dart-demos-scenes
  )

  if(TARGET ${example_name})
    dart_demos_link_optional_collision_backends(${example_name})
    dart_demos_link_memory_diagnostics_platform(${example_name})
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
    dart_demos_link_memory_diagnostics_platform(${alias_scene})
  endforeach()

  if(TARGET examples)
    add_dependencies(examples soft_bodies soft_cubes soft_open_chain)
  endif()
  return()
endif()

dart_demos_collect_scene_srcs(demos_scene_srcs)
add_library(dart-demos-scenes STATIC ${demos_scene_srcs})
target_link_libraries(dart-demos-scenes PUBLIC ${required_libraries})
dart_demos_configure_scenes_target(dart-demos-scenes)

add_executable(${example_name} ${srcs})
target_link_libraries(${example_name} ${required_libraries} dart-demos-scenes)
dart_demos_link_optional_collision_backends(${example_name})
dart_demos_link_memory_diagnostics_platform(${example_name})

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
  target_link_libraries(${alias_scene} ${required_libraries} dart-demos-scenes)
  dart_demos_link_optional_collision_backends(${alias_scene})
  dart_demos_link_memory_diagnostics_platform(${alias_scene})
endforeach()
