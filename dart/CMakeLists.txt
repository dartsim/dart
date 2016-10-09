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
message(STATUS ${DART_CONFIG_HPP_OUT})
install(FILES ${DART_CONFIG_HPP_OUT} DESTINATION include/dart)

define_property(GLOBAL PROPERTY DART_CORE_HEADERS
  BRIEF_DOCS "Header files of DART core."
  FULL_DOCS "Header files of DART core."
)
define_property(GLOBAL PROPERTY DART_CORE_SOURCES
  BRIEF_DOCS "Source files of DART core."
  FULL_DOCS "Source files of DART core."
)

function(dart_add_headers)
  foreach(headers ${ARGN})
    set_property(GLOBAL APPEND PROPERTY DART_CORE_HEADERS "${headers}")
  endforeach()
endfunction()

function(dart_add_sources)
  foreach(source ${ARGN})
    set_property(GLOBAL APPEND PROPERTY DART_CORE_SOURCES "${source}")
  endforeach()
endfunction()

# Component
add_component(${PROJECT_NAME} dart)

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

# Set header and source files
get_filename_component(dart_hpp "dart.hpp" ABSOLUTE)
dart_add_headers(${dart_hpp})
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
  ${CCD_LIBRARIES}
  ${FCL_LIBRARIES}
  ${ASSIMP_LIBRARIES}
  ${Boost_LIBRARIES})

# Component
add_component_targets(${PROJECT_NAME} dart dart)

# Coverage test
coveralls_add_sources(${dart_core_headers} ${dart_core_sources})

if(MSVC)
  set_target_properties(
    ${target} PROPERTIES
    STATIC_LIBRARY_FLAGS_RELEASE "/LTCG"
  )
endif()

install(FILES dart.hpp DESTINATION include/dart/ COMPONENT headers)

if(FLANN_FOUND)
  add_subdirectory(planning)
endif()

if(TINYXML_FOUND AND TINYXML2_FOUND)
  add_subdirectory(utils)
endif()

if(OPENGL_FOUND AND GLUT_FOUND)
  add_subdirectory(gui)
endif()
