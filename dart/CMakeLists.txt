add_subdirectory(external)

# Enable multi-threaded compilation.
# We do this here and not in the root folder since the examples and the
# tutorials do not have enough source files to benefit from this.
if(MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
endif()

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

set(DART_CONFIG_HPP_IN ${CMAKE_SOURCE_DIR}/dart/config.hpp.in)
set(DART_CONFIG_HPP_OUT ${CMAKE_BINARY_DIR}/dart/config.hpp)
if(DART_VERBOSE)
  message(STATUS ${DART_CONFIG_HPP_OUT})
endif()
configure_file(${DART_CONFIG_HPP_IN} ${DART_CONFIG_HPP_OUT} @ONLY)
install(FILES ${DART_CONFIG_HPP_OUT} DESTINATION include/dart)

#===============================================================================
# Source directories
#===============================================================================
# common
# math
# integration
# lcpsolver
# optimizer
#   ipopt
#   nlopt
# dynamics
# collision
#   dart
#   fcl
#   bullet
# constraint
# simulation
# utils
#   sdf
#   urdf
# gui
#   osg
#     render
# planning

#===============================================================================
# Targets - {dependency targets}, source directories, [external dependency libs]
#===============================================================================
# dart - common, math, integration, lcpsolver, optimizer, dynamics, collision,
#        collision/dart, collision/fcl, constraint, simulation, [eigen3, libccd,
#        fcl, assimp, boost]
# dart-optimizer-ipopt - {dart}, optimizer/ipopt, [ipopt]
# dart-optimizer-nlopt - {dart}, optimizer/nlopt, [nlopt]
# dart-collision-bullet - {dart}, collision/bullet, [bullet]
# dart-utils - {dart-collision-bullet}, utils, utils/sdf, [tinyxml, tinyxml2]
# dart-utils-urdf - {dart-utils}, utils/urdf, [urdfdom]
# dart-gui - {dart}, gui, [opengl, glut]
# dart-gui-osg - {dart-gui}, gui/osg, gui/osg/render, [openscenegraph]
# dart-planning - {dart}, planning, [flann]

#===============================================================================
# Components - (dependency component), {dependency targets}
#===============================================================================
# dart - {dart}
# optimizer-ipopt - (dart), {dart-optimizer-ipopt}
# optimizer-nlopt - (dart), {dart-optimizer-nlopt}
# collision-bullet - (dart), {dart-collision-bullet}
# utils - (collision-bullet), {dart-utils}
# utils-urdf - (utils), {dart-utils-urdf}
# gui - (utils), {dart-gui}
# gui-osg - (gui), {dart-gui-osg}
# planning - (dart), {dart-planning}

# Component. This should be called before adding subdirectories.
add_component(${PROJECT_NAME} dart)

function(dart_add_core_headers)
  dart_property_add(DART_CORE_HEADERS ${ARGN})
endfunction(dart_add_core_headers)

function(dart_add_core_sources)
  dart_property_add(DART_CORE_SOURCES ${ARGN})
endfunction(dart_add_core_sources)

# Add subdirectories
add_subdirectory(common)
add_subdirectory(math)
add_subdirectory(integration)
add_subdirectory(lcpsolver)
add_subdirectory(optimizer)
add_subdirectory(dynamics)
add_subdirectory(collision)
add_subdirectory(constraint)
add_subdirectory(simulation)

if(FLANN_FOUND)
  add_subdirectory(planning)
endif()

if(TINYXML_FOUND AND TINYXML2_FOUND AND BULLET_FOUND)
  add_subdirectory(utils)
endif()

if(OPENGL_FOUND AND GLUT_FOUND AND BULLET_FOUND)
  add_subdirectory(gui)
endif()

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
  message(STATUS "Adding ${components_length} components including "
    "the default component 'dart'")
endif()

# Set header and source files
get_filename_component(dart_hpp "dart.hpp" ABSOLUTE)
dart_add_core_headers(${dart_hpp})
get_property(dart_core_headers GLOBAL PROPERTY DART_CORE_HEADERS)
get_property(dart_core_sources GLOBAL PROPERTY DART_CORE_SOURCES)

# Add target
dart_add_library(dart ${dart_core_headers} ${dart_core_sources})
target_include_directories(
  dart SYSTEM
  PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    ${CCD_INCLUDE_DIRS}
    ${FCL_INCLUDE_DIRS}
    ${ASSIMP_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)
target_link_libraries(
  dart
  PUBLIC
    ${CCD_LIBRARIES}
    ${FCL_LIBRARIES}
    ${ASSIMP_LIBRARIES}
    ${Boost_LIBRARIES}
    ${PROJECT_NAME}-external-odelcpsolver
)

# Build DART with all available SIMD instructions
if(DART_ENABLE_SIMD AND NOT MSVC)
  target_compile_options(dart PUBLIC -march=native)
endif()

# Default component
add_component_targets(
  ${PROJECT_NAME}
  dart # component name
  dart # target name
)
add_component_dependencies(
  ${PROJECT_NAME}
  dart
  external-odelcpsolver
)

# Coverage test
dart_coveralls_add(${dart_core_headers} ${dart_core_sources})

if(MSVC)
  set_target_properties(
    ${target} PROPERTIES
    STATIC_LIBRARY_FLAGS_RELEASE "/LTCG"
  )
endif()

install(FILES dart.hpp DESTINATION include/dart/ COMPONENT headers)
