file(GLOB tutorials_src "*.cpp")
list(SORT tutorials_src)

foreach(tutorial ${tutorials_src})
  get_filename_component(tutorial_base ${tutorial} NAME_WE)

  add_executable(${tutorial_base} ${tutorial})
  target_link_libraries(${tutorial_base} dart dart-gui dart-utils-urdf)

  dart_add_tutorial(${tutorial_base})
endforeach(tutorial)
