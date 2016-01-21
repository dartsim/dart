file(GLOB osgkido_examples_src "*.cpp")
list(SORT osgkido_examples_src)

message(STATUS "")
message(STATUS "[ kido-gui-osg Examples ]")

foreach(example ${osgkido_examples_src})

  get_filename_component(example_base ${example} NAME_WE)
  message(STATUS "Adding example: " ${example_base})
  add_executable(${example_base} ${example})
  target_link_libraries(${example_base} kido-gui-osg ${OPENSCENEGRAPH_LIBRARIES})

endforeach(example)
