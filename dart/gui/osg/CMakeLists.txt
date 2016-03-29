include_directories(SYSTEM ${OPENSCENEGRAPH_INCLUDE_DIRS})

# Search all header and source files
file(GLOB hdrs "*.h")
file(GLOB srcs "*.cpp")

add_subdirectory(render)

set(dart_gui_hdrs ${dart_gui_hdrs} ${hdrs} PARENT_SCOPE)
set(dart_gui_srcs ${dart_gui_srcs} ${srcs} PARENT_SCOPE)

# Generate header for this namespace
dart_get_filename_components(header_names "gui osg headers" ${hdrs})
list(APPEND header_names "render/render.h")
dart_generate_include_header_list(
  gui_osg_headers
  "dart/gui/osg/"
  "gui osg headers"
  ${header_names}
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/osg.h.in
  ${CMAKE_CURRENT_BINARY_DIR}/osg.h
)

if(${DART_BUILD_EXAMPLES})
  add_subdirectory(examples)
endif(${DART_BUILD_EXAMPLES})

install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/osg.h
  DESTINATION include/dart/gui/osg
  COMPONENT headers
)
#install(TARGETS osgDart EXPORT osgDartTargets DESTINATION lib)
