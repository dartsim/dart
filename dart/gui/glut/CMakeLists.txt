# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")
dart_add_gui_headers(${hdrs} ${detail_hdrs})
dart_add_gui_sources(${srcs} ${detail_srcs})

# Generate header for this namespace
dart_get_filename_components(header_names "gui_glut_headers headers" ${hdrs})
dart_generate_include_header_file(
  "${CMAKE_CURRENT_BINARY_DIR}/glut.hpp"
  "dart/gui/glut/"
  ${header_names}
)

# Install
if(NOT DART_BUILD_DARTPY)
  install(
    FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/glut.hpp
    DESTINATION include/dart/gui/glut
    COMPONENT headers
  )
endif()
