cmake_minimum_required(VERSION 3.22.1)

get_filename_component(example_name ${CMAKE_CURRENT_LIST_DIR} NAME)

project(${example_name})

set(required_components utils gui-osg)
set(required_libraries dart dart-utils dart-gui-osg)

if(DART_IN_SOURCE_BUILD)
  dart_build_example_in_source(${example_name} LINK_LIBRARIES ${required_libraries})
  return()
endif()

# ContactInverseDynamics ships in DART 6.19, so standalone builds need a
# 6.19+ install; the in-tree package version catches up in the 6.19.0 release
# commit (same convention as the other 6.19-only examples).
find_package(DART 6.19 REQUIRED COMPONENTS ${required_components} CONFIG)

file(GLOB srcs "*.cpp" "*.hpp")
add_executable(${example_name} ${srcs})
target_link_libraries(${example_name} PUBLIC ${required_libraries})
