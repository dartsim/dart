# Set local target name
set(target_name ${PROJECT_NAME}-gui)

# OpenSceneGraph
dart_find_package(OpenSceneGraph)
if(
  (NOT OPENSCENEGRAPH_FOUND AND NOT OpenSceneGraph_FOUND)
  OR NOT TARGET osg::osg
  OR NOT TARGET osg::osgText
)
  message(
    FATAL_ERROR
    "dart-gui requires OpenSceneGraph >= 3.6.5; install it or set DART_BUILD_GUI=OFF."
  )
endif()

# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")
file(GLOB detail_hdrs "detail/*.hpp")
file(GLOB detail_srcs "detail/*.cpp")

set(dart_gui_osg_hdrs ${hdrs} ${detail_hdrs})
set(dart_gui_osg_srcs ${srcs} ${detail_srcs})

add_subdirectory(render)

target_sources(${target_name} PRIVATE ${dart_gui_osg_hdrs} ${dart_gui_osg_srcs})
target_link_libraries(
  ${target_name}
  PUBLIC
    $<BUILD_INTERFACE:osg::osg>
    $<BUILD_INTERFACE:osg::osgText>
    $<INSTALL_INTERFACE:$<1:osg::osg>>
    $<INSTALL_INTERFACE:$<1:osg::osgText>>
)
if(APPLE)
  target_compile_definitions(${target_name} PRIVATE GL_SILENCE_DEPRECATION)
endif()

add_component_dependency_packages(${PROJECT_NAME} gui OpenSceneGraph)

# Generate header for this namespace
dart_get_filename_components(header_names "gui osg headers" ${hdrs})
list(APPEND header_names "render/render.hpp")
dart_generate_include_header_file(
  "${CMAKE_CURRENT_BINARY_DIR}/osg.hpp"
  "dart/gui/osg/"
  ${header_names}
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/osg.hpp
  DESTINATION include/dart/gui/osg
  COMPONENT headers
)
install(
  FILES ${detail_hdrs}
  DESTINATION include/dart/gui/osg/detail
  COMPONENT headers
)

dart_format_add(${hdrs} ${srcs} ${dart_gui_osg_hdrs} ${dart_gui_osg_srcs})
