# Copyright (c) 2011, The DART development contributors

if(NOT _dart_build_collision_references)
  return()
endif()

dart_format_add(dart_query_helper.cpp dart_query_helper.hpp)

add_subdirectory(fcl)
add_subdirectory(bullet)
add_subdirectory(ode)
