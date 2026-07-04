cmake_minimum_required(VERSION 3.22.1)

set(example_name dart-demos)

project(${example_name})

set(required_components utils-urdf gui-osg)
set(required_libraries dart dart-utils dart-utils-urdf dart-gui-osg)

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

  file(GLOB demos_scene_srcs "scenes/*.cpp" "scenes/*.hpp")
  add_library(dart-demos-scenes STATIC ${demos_scene_srcs})
  target_link_libraries(dart-demos-scenes PUBLIC ${required_libraries})
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

file(GLOB demos_scene_srcs "scenes/*.cpp" "scenes/*.hpp")
add_library(dart-demos-scenes STATIC ${demos_scene_srcs})
target_link_libraries(dart-demos-scenes PUBLIC ${required_libraries})

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
