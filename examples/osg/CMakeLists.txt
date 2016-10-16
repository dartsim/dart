# This directory depends on OpenSceneGraph
if(HAVE_OPENSCENEGRAPH)

file(GLOB dart_examples_osg_src "*.cpp")
list(SORT dart_examples_osg_src)

foreach(example ${dart_examples_osg_src})
  get_filename_component(example_base ${example} NAME_WE)

  add_executable(${example_base} ${example})
  target_link_libraries(${example_base} dart dart-gui-osg dart-utils-urdf)

  dart_add_example(${example_base})
endforeach(example)

# Automatically identify all subdirectories that should be included in the build system
file(GLOB children RELATIVE ${CMAKE_CURRENT_LIST_DIR} "*")
set(directories "")
foreach(child ${children})
  if(IS_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/${child}")
    if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/${child}/CMakeLists.txt")
      list(APPEND directories ${child})
    endif(EXISTS "${CMAKE_CURRENT_LIST_DIR}/${child}/CMakeLists.txt")
  endif(IS_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/${child}")
endforeach(child)

foreach(EXAMPLE_DIR ${directories})
  add_subdirectory(${EXAMPLE_DIR})
endforeach(EXAMPLE_DIR)

endif(HAVE_OPENSCENEGRAPH)
