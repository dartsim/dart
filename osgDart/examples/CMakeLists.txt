file(GLOB osgdart_examples_src "*.cpp")
list(SORT osgdart_examples_src)

foreach(example ${osgdart_examples_src})
  get_filename_component(example_base ${example} NAME_WE)

  add_executable(${example_base} ${example})
  target_link_libraries(${example_base} dart osgDart ${OPENSCENEGRAPH_LIBRARIES})

  dart_add_example(${example_base})
endforeach(example)
