# Search all header and source files
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.hpp")

set(kido_gui_osg_srcs ${kido_gui_osg_srcs} ${srcs} PARENT_SCOPE)
set(kido_gui_osg_hdrs ${kido_gui_osg_hdrs} ${hdrs} PARENT_SCOPE)

kido_get_filename_components(header_names "gui osg render headers" ${hdrs})
kido_generate_include_header_list(
  gui_osg_render_headers
  "kido/gui/osg/render/"
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
  DESTINATION include/kido/gui/osg/render
  COMPONENT headers
)
