cmake_minimum_required(VERSION 3.22.1)

project(dart-examples)

# None GUI examples
add_subdirectory(hello_world)
add_subdirectory(cylindrical_constraint)
add_subdirectory(speed_test)

# OSG renderer examples
if(TARGET dart-gui-osg)
  add_subdirectory(contact_benchmark)
  add_subdirectory(demos)
endif()
