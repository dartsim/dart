file(GLOB osgDart_render_hdrs "*.h")

kido_get_filename_components(header_names "osgDart render headers" ${osgDart_render_hdrs})
kido_generate_include_header_list(
  osgDart_render_headers
  "osgDart/render/"
  "osgDart render headers"
  ${header_names}
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/render.h.in
  ${CMAKE_CURRENT_BINARY_DIR}/render.h
)

install(
  FILES ${osgDart_render_hdrs} ${CMAKE_CURRENT_BINARY_DIR}/render.h
  DESTINATION include/osgDart/render
  COMPONENT headers
)
