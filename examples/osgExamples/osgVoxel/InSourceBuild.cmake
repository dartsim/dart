get_filename_component(example_name ${CMAKE_CURRENT_LIST_DIR} NAME)

if(NOT TARGET dart-utils-urdf OR NOT TARGET dart-gui-osg)
  return()
endif()

file(GLOB srcs "*.cpp" "*.hpp")

add_executable(${example_name} ${srcs})
target_link_libraries(${example_name} dart dart-utils-urdf dart-gui-osg)
set_target_properties(${example_name}
  PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

dart_add_example(${example_name})

dart_format_add(${srcs})
