file(GLOB osgKido_render_hdrs "*.h")

kido_get_filename_components(header_names "osgKido render headers" ${osgKido_render_hdrs})
kido_generate_include_header_list(
  osgKido_render_headers
  "osgKido/render/"
  "osgKido render headers"
  ${header_names}
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/render.h.in
  ${CMAKE_CURRENT_BINARY_DIR}/render.h
)

install(
  FILES ${osgKido_render_hdrs} ${CMAKE_CURRENT_BINARY_DIR}/render.h
  DESTINATION include/osgKido/render
  COMPONENT headers
)
