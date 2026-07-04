cmake_minimum_required(VERSION 3.22.1)

set(example_name dart-demos)

project(${example_name})

set(required_components utils-urdf gui-osg)
set(required_libraries dart dart-utils dart-utils-urdf dart-gui-osg)

# The scene ports (examples/demos/scenes/*.cpp) live in a subdirectory, so
# they cannot be picked up by dart_build_target_in_source()'s non-recursive
# `file(GLOB srcs "*.cpp" "*.hpp")`. Compile them into a small static library
# instead and link it into the dart-demos executable below, so the mandated
# dart_build_example_in_source() in-source path can still be used unmodified
# for the executable itself.
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
  return()
endif()

find_package(DART 6.17 REQUIRED COMPONENTS ${required_components} CONFIG)

file(GLOB demos_scene_srcs "scenes/*.cpp" "scenes/*.hpp")
add_library(dart-demos-scenes STATIC ${demos_scene_srcs})
target_link_libraries(dart-demos-scenes PUBLIC ${required_libraries})

file(GLOB srcs "*.cpp" "*.hpp")
add_executable(${example_name} ${srcs})
target_link_libraries(
  ${example_name}
  PUBLIC ${required_libraries} dart-demos-scenes
)
