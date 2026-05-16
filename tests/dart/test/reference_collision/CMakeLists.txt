# Copyright (c) 2011, The DART development contributors

if(NOT _dart_build_collision_references)
  return()
endif()

add_subdirectory(fcl)
add_subdirectory(bullet)
add_subdirectory(ode)
