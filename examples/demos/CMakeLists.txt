cmake_minimum_required(VERSION 3.22.1)

set(example_name dart-demos)

project(${example_name})

set(required_components utils-urdf gui-osg)
set(required_libraries dart dart-utils dart-utils-urdf dart-gui-osg)

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
function(force_link_bullet_backend target)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang" AND NOT APPLE)
    target_link_options(${target} PRIVATE "LINKER:--no-as-needed")
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

# Globs every scene source, including one level of scene subdirectories (for
# multi-file scenes, e.g. scenes/tinkertoy/), and conditionally adds
# scenes/human_joint_limits/ only when TinyDNN + Threads were found. Shared by
# both the in-source and standalone build paths below.
function(dart_demos_collect_scene_srcs out_var)
  file(GLOB _srcs "scenes/*.cpp" "scenes/*.hpp")
  file(GLOB _tinkertoy_srcs "scenes/tinkertoy/*.cpp" "scenes/tinkertoy/*.hpp")
  list(APPEND _srcs ${_tinkertoy_srcs})
  if(DART_DEMOS_HAVE_TINY_DNN)
    file(
      GLOB _human_joint_limits_srcs
      "scenes/human_joint_limits/*.cpp"
      "scenes/human_joint_limits/*.hpp"
    )
    list(APPEND _srcs ${_human_joint_limits_srcs})
  endif()
  set(${out_var} ${_srcs} PARENT_SCOPE)
endfunction()

# Links the optional TinyDNN + Threads dependency and defines
# DART_DEMOS_HAVE_TINY_DNN when found. Shared by both build paths below.
function(dart_demos_configure_scenes_target target)
  if(DART_DEMOS_HAVE_TINY_DNN)
    target_link_libraries(
      ${target}
      PUBLIC ${TinyDNN_LIBRARIES} Threads::Threads
    )
    target_compile_definitions(${target} PUBLIC DART_DEMOS_HAVE_TINY_DNN)
  endif()
endfunction()

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
  # dart_build_example_in_source() returns early (skipping the example) unless
  # every LINK_LIBRARIES entry is a target, and it links with the plain
  # target_link_libraries() signature, so the Bullet backend is attached to the
  # created executable here (matching that plain signature) instead of being
  # passed through the macro.
  if(TARGET ${example_name} AND TARGET dart-collision-bullet)
    target_link_libraries(${example_name} dart-collision-bullet)
    force_link_bullet_backend(${example_name})
  endif()
  return()
endif()

find_package(DART 6.17 REQUIRED COMPONENTS ${required_components} CONFIG)
find_package(DART 6.17 QUIET COMPONENTS collision-bullet CONFIG)

dart_demos_collect_scene_srcs(demos_scene_srcs)
add_library(dart-demos-scenes STATIC ${demos_scene_srcs})
target_link_libraries(dart-demos-scenes PUBLIC ${required_libraries})
dart_demos_configure_scenes_target(dart-demos-scenes)

file(GLOB srcs "*.cpp" "*.hpp")
add_executable(${example_name} ${srcs})
target_link_libraries(
  ${example_name}
  PUBLIC ${required_libraries} dart-demos-scenes
)
if(TARGET dart-collision-bullet)
  target_link_libraries(${example_name} PUBLIC dart-collision-bullet)
  force_link_bullet_backend(${example_name})
endif()
