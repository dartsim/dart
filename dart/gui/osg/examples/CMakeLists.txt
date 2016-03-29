file(GLOB dart_gui_osg_examples_src "*.cpp")
list(SORT dart_gui_osg_examples_src)

foreach(example ${dart_gui_osg_examples_src})
  get_filename_component(example_base ${example} NAME_WE)

  add_executable(${example_base} ${example})
  target_link_libraries(${example_base} dart)

  dart_add_example(${example_base})
endforeach(example)
