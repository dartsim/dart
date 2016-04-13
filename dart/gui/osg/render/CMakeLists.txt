# Search all header and source files
file(GLOB hdrs "*.h")
file(GLOB srcs "*.cpp")

set(dart_gui_osg_srcs ${dart_gui_osg_srcs} ${srcs} PARENT_SCOPE)
set(dart_gui_osg_hdrs ${dart_gui_osg_hdrs} ${hdrs} PARENT_SCOPE)

dart_get_filename_components(header_names "gui osg render headers" ${hdrs})
dart_generate_include_header_list(
  gui_osg_render_headers
  "dart/gui/osg/render/"
  "gui osg render headers"
  ${header_names}
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/render.h.in
  ${CMAKE_CURRENT_BINARY_DIR}/render.h
)

install(
  FILES ${dart_gui_osg_hdrs} ${CMAKE_CURRENT_BINARY_DIR}/render.h
  DESTINATION include/dart/gui/osg/render
  COMPONENT headers
)

# Coverage test files
set(dart_coveralls_files "${dart_coveralls_files};${hdrs};${srcs}"
    CACHE INTERNAL "files for coverage tests")
