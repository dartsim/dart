find_package(filament REQUIRED)  # required for now, but should be optional in the end

# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")
dart_add_gui_headers(${hdrs} ${detail_hdrs})
dart_add_gui_sources(${srcs} ${detail_srcs})

# Generate header for this namespace
dart_get_filename_components(header_names "gui_filament_headers headers" ${hdrs})
dart_generate_include_header_list(
  gui_filament_headers
  "dart/gui/filament/"
  "gui_filament_headers headers"
  ${header_names}
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/filament.hpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/filament.hpp
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/filament.hpp
  DESTINATION include/dart/gui/filament
  COMPONENT headers
)

dart_format_add(${hdrs} ${detail_hdrs} ${srcs} ${detail_srcs})
