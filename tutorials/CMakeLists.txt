file(GLOB tutorials_src "*.cpp")
list(SORT tutorials_src)

message(STATUS "")
message(STATUS "[ Tutorials ]")

foreach(tutorial ${tutorials_src})
  get_filename_component(tutorial_base ${tutorial} NAME_WE)
  list(APPEND all_tutorials ${tutorial_base})

  message(STATUS "Adding tutorial: ${tutorial_base}")
  add_executable(${tutorial_base} ${tutorial})
  target_link_libraries(${tutorial_base} dart)
endforeach(tutorial)

add_custom_target(tutorials DEPENDS ${all_tutorials})
