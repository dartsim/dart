# Copyright (c) 2011-2021, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/master/LICENSE
#
# This file is provided under the "BSD-style" License

# Enable multi-threaded compilation.
# We do this here and not in the root folder since the examples and the
# tutorials do not have enough source files to benefit from this.
if(MSVC AND ${CMAKE_CXX_COMPILER_ID} STREQUAL "MSVC")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
endif()

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${DART_BINARY_DIR}/lib")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${DART_BINARY_DIR}/lib")

#===============================================================================
# Targets - {dependency targets}, source directories, [external dependency libs]
#===============================================================================
# dart - common, math, optimization, dynamics, collision,
#        collision/dart, collision/fcl, simulation, [eigen3, libccd,
#        fcl, assimp, boost]
# dart-optimization-ipopt - {dart}, optimization/ipopt, [ipopt]
# dart-optimization-nlopt - {dart}, optimization/nlopt, [nlopt]
# dart-collision-bullet - {dart}, collision/bullet, [bullet]
# dart-io - {dart}, io, io/sdf, [tinyxml2]
# dart-io-urdf - {dart-io}, io/urdf, [urdfdom]
# dart-gui - {dart}, gui, [opengl]
# dart-gui-glut - {dart-gui}, gui/glut, [glut]
# dart-gui-osg - {dart-gui}, gui/osg, gui/osg/render, [openscenegraph]
# dart-planning - {dart}, planning, [flann]

#===============================================================================
# Components: (dependency component), {external dependency}, [optional external dependency]
#===============================================================================
#
# external-imgui: OpenGL
#
# common: {Boost}, {Eigen3}
# math: (common), {Boost}, {Eigen3}
# optimization: (common), (math), {Boost}, {Eigen3}, [NLOPT], [IPOPT], [pagmo]
# collision: <todo>
# dynamics: (common), (math), (optimization), (collision), {assimp}, {octomap}, [Bullet], {fcl}, [ODE]
# simulation: (common), (math), (optimization), (collision), (dynamics)
# io: (common), (math), (optimization), (collision), (dynamics), (simulation), {tinyxml2}, [urdfdom]
# gui: (external-imgui), (common), (math), (optimization), (collision), (dynamics), (simulation), (io), {OpenGL}, [GLUT], [OpenSceneGraph]
# planning: (common), (math), (optimization), (collision), {flann}, {lz4}

# dart - {dart}
# optimization-ipopt - (dart), {dart-optimization-ipopt}
# optimization-nlopt - (dart), {dart-optimization-nlopt}
# collision-bullet - (dart), {dart-collision-bullet}
# io - (collision-bullet), {dart-io}
# io-urdf - (io), {dart-io-urdf}
# gui - (io), {dart-gui}
# gui-glut - (gui), {dart-gui-glut}
# gui-osg - (gui), {dart-gui-osg}
# planning - (dart), {dart-planning}

set(org_name dartsim)
set(project_name ${PROJECT_NAME})
set(project_version_major ${DART_MAJOR_VERSION})

set(include_base_path include/${org_name}/${project_name}${project_version_major})
set(include_path ${include_base_path}/${project_name})

set(DART_CONFIG_HPP_IN ${DART_SOURCE_DIR}/dart/config.hpp.in)
set(DART_CONFIG_HPP_OUT ${DART_BINARY_DIR}/dart/config.hpp)
if(DART_VERBOSE)
  message(STATUS ${DART_CONFIG_HPP_OUT})
endif()
configure_file(${DART_CONFIG_HPP_IN} ${DART_CONFIG_HPP_OUT} @ONLY)
install(FILES ${DART_CONFIG_HPP_OUT} DESTINATION ${include_path})

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
    planning
  GENERATE_META_HEADER
)

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
