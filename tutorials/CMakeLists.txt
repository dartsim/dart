file(GLOB tutorials_src "*.cpp")
list(SORT tutorials_src)

message(STATUS "")
message(STATUS "[ Tutorials ]")

foreach(tutorial ${tutorials_src})
  get_filename_component(tutorial_base ${tutorial} NAME_WE)
  message(STATUS "Adding tutorial: ${tutorial_base}")
  add_executable(${tutorial_base} ${tutorial})
  target_link_libraries(${tutorial_base} dart dart-gui dart-utils-urdf)
endforeach(tutorial)
