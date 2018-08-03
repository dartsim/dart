get_filename_component(tutorial_name ${CMAKE_CURRENT_LIST_DIR} NAME)

if(NOT TARGET dart-collision-bullet
  OR NOT TARGET dart-io-urdf
  OR NOT TARGET dart-gui)
  return()
endif()

file(GLOB srcs "*.cpp" "*.hpp")

add_executable(${tutorial_name} ${srcs})
target_link_libraries(${tutorial_name}
  dart dart-collision-bullet dart-io-urdf dart-gui
)
set_target_properties(${tutorial_name}
  PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

dart_add_tutorial(${tutorial_name})
