cmake_minimum_required(VERSION 3.22.1)

get_filename_component(example_name ${CMAKE_CURRENT_LIST_DIR} NAME)

project(${example_name})

set(required_components utils-urdf gui-osg)
set(required_libraries dart dart-utils-urdf dart-gui-osg)

if(DART_IN_SOURCE_BUILD)
  dart_build_example_in_source(${example_name} LINK_LIBRARIES ${required_libraries})
  foreach(scene_target soft_cubes soft_open_chain)
    add_executable(${scene_target} main.cpp)
    target_link_libraries(${scene_target} ${required_libraries})
    target_compile_definitions(
      ${scene_target}
      PRIVATE DART_SOFT_BODIES_DEFAULT_SCENE="${scene_target}"
    )
    set_target_properties(
      ${scene_target}
      PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${DART_BINARY_DIR}/bin"
    )
    dart_add_example(${scene_target})
  endforeach()
  return()
endif()

find_package(DART 6.17 REQUIRED COMPONENTS ${required_components} CONFIG)

file(GLOB srcs "*.cpp" "*.hpp")
add_executable(${example_name} ${srcs})
target_link_libraries(${example_name} PUBLIC ${required_libraries})

foreach(scene_target soft_cubes soft_open_chain)
  add_executable(${scene_target} ${srcs})
  target_link_libraries(${scene_target} PUBLIC ${required_libraries})
  target_compile_definitions(
    ${scene_target}
    PRIVATE DART_SOFT_BODIES_DEFAULT_SCENE="${scene_target}"
  )
endforeach()
