# Copyright (c) 2011-2023, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/master/LICENSE
#
# This file is provided under the "BSD-style" License

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${DART_BINARY_DIR}/lib")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${DART_BINARY_DIR}/lib")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${DART_BINARY_DIR}/bin")

#===============================================================================
# Components: (dependency component), {external dependency}, [optional external dependency]
#===============================================================================
#
# external-imgui: OpenGL
#
# common: {Eigen3}
# math: (common), {Eigen3}
# optimization: (common), (math), {Eigen3}, [NLOPT], [IPOPT], [pagmo]
# collision: <todo>
# dynamics: (common), (math), (optimization), (collision), {assimp}, {octomap}, [Bullet], {fcl}, [ODE]
# simulation: (common), (math), (optimization), (collision), (dynamics)
# io: (common), (math), (optimization), (collision), (dynamics), (simulation), {tinyxml2}, [urdfdom]
# gui: (external-imgui), (common), (math), (optimization), (collision), (dynamics), (simulation), (io), {OpenGL}, [GLUT], [OpenSceneGraph]

set(org_name dartsim)
set(project_name ${PROJECT_NAME})
set(project_version_major ${DART_MAJOR_VERSION})

set(include_base_path include/${org_name}/${project_name}${project_version_major})
set(include_path ${include_base_path}/${project_name})

add_subdirectory(external)

dart_add_project(
  PROJECT_NAME ${PROJECT_NAME}
  PROJECT_VERSION_MAJOR ${DART_MAJOR_VERSION}
  COMPONENTS
    common
    math
    optimization
    collision
    dynamics
    simulation
    io
    gui
  GENERATE_META_HEADER
)

set(DART_CONFIG_HPP_IN ${DART_SOURCE_DIR}/dart/config.hpp.in)
set(DART_CONFIG_HPP_OUT ${DART_BINARY_DIR}/dart/config.hpp)
if(DART_VERBOSE)
  message(STATUS ${DART_CONFIG_HPP_OUT})
endif()
configure_file(${DART_CONFIG_HPP_IN} ${DART_CONFIG_HPP_OUT} @ONLY)
install(FILES ${DART_CONFIG_HPP_OUT} DESTINATION ${include_path})

# Print building component
get_property(components GLOBAL PROPERTY ${PROJECT_NAME}_COMPONENTS)
if(DART_VERBOSE)
  message(STATUS "")
  message(STATUS "[ Components ]")
  foreach(component ${components})
    message(STATUS "Adding component: ${component}")
  endforeach()
else()
  list(LENGTH components components_length)
  message(STATUS "Adding ${components_length} components")
endif()

dart_format_add(${dart_core_headers} ${dart_core_sources})
