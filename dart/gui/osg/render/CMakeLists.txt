# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")

set(dart_gui_osg_hdrs ${dart_gui_osg_hdrs} ${hdrs} PARENT_SCOPE)
set(dart_gui_osg_srcs ${dart_gui_osg_srcs} ${srcs} PARENT_SCOPE)

dart_get_filename_components(header_names "gui osg render headers" ${hdrs})
dart_generate_include_header_list(
  gui_osg_render_headers
  "dart/gui/osg/render/"
  "gui osg render headers"
  ${header_names}
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/render.hpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/render.hpp
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/render.hpp
  DESTINATION include/dart/gui/osg/render
  COMPONENT headers
)
