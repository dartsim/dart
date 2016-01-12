file(GLOB osgkido_examples_src "*.cpp")
list(SORT osgkido_examples_src)

message(STATUS "")
message(STATUS "[ osgDart Examples ]")

foreach(example ${osgkido_examples_src})

  get_filename_component(example_base ${example} NAME_WE)
  message(STATUS "Adding example: " ${example_base})
  add_executable(${example_base} ${example})
  target_link_libraries(${example_base} dart osgDart ${OPENSCENEGRAPH_LIBRARIES})

endforeach(example)
