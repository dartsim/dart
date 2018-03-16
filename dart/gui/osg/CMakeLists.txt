# Dependency checks
if (NOT TARGET dart-gui)
  return()
endif()

# OpenSceneGraph
if(DART_BUILD_GUI_OSG)

  find_package(OpenSceneGraph 3.0 QUIET
    COMPONENTS osg osgViewer osgManipulator osgGA osgDB osgShadow)

  # It seems that OPENSCENEGRAPH_FOUND will inadvertently get set to true when
  # OpenThreads is found, even if OpenSceneGraph is not installed. This is quite
  # possibly a bug in OSG's cmake configuration file. For now, it seems that
  # requiring OSG_FOUND to be true as well fixes this.
  if(OPENSCENEGRAPH_FOUND AND OSG_FOUND)
    if(DART_VERBOSE)
      message(STATUS "Looking for OpenSceneGraph - ${OPENSCENEGRAPH_VERSION} found")
    endif()
  else()
    # dart-gui-osg requires both OSG and OpenThreads. This section attempts to
    # identify which of those are missing from the building machine and offer
    # advice to the user for getting dart-gui-osg to build.
    find_package(OpenThreads QUIET)
    if(OPENTHREADS_FOUND)
      set(warning_msg "Could NOT find OpenSceneGraph")
    else()
      if(OSG_LIBRARY)
        set(warning_msg "Could NOT find OpenThreads")
      else()
        set(warning_msg "Could NOT find OpenSceneGraph nor OpenThreads")
      endif()
    endif()
    message(STATUS "${warning_msg} -- we will skip dart-gui-osg\n"
            "If you believe you do have both OSG and OpenThreads installed, try setting OSG_DIR")
    return()
  endif()

else()

  message(STATUS "Skipping OpenSceneGraph (DART_BUILD_GUI_OSG == ${DART_BUILD_GUI_OSG})")
  return()

endif()

# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")

set(dart_gui_osg_hdrs ${hdrs})
set(dart_gui_osg_srcs ${srcs})

add_subdirectory(render)

# Set local target name
set(target_name ${PROJECT_NAME}-gui-osg)
set(component_name gui-osg)

# Add target
dart_add_library(${target_name} ${hdrs} ${srcs} ${dart_gui_osg_hdrs} ${dart_gui_osg_srcs})
target_include_directories(
  ${target_name} SYSTEM
  PUBLIC ${OPENSCENEGRAPH_INCLUDE_DIRS}
)
target_link_libraries(
  ${target_name}
  dart-gui
  ${OPENSCENEGRAPH_LIBRARIES}
)

# Component
add_component(${PROJECT_NAME} ${component_name})
add_component_targets(${PROJECT_NAME} ${component_name} ${target_name})
add_component_dependencies(${PROJECT_NAME} ${component_name} gui)

# Generate header for this namespace
dart_get_filename_components(header_names "gui osg headers" ${hdrs})
list(APPEND header_names "render/render.hpp")
dart_generate_include_header_list(
  gui_osg_headers
  "dart/gui/osg/"
  "gui osg headers"
  ${header_names}
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/osg.hpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/osg.hpp
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/osg.hpp
  DESTINATION include/dart/gui/osg
  COMPONENT headers
)
