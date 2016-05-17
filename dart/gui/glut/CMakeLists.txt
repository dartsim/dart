include_directories(SYSTEM ${glut_INCLUDE_DIRS})

# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")

set(dart_gui_glut_hdrs ${hdrs})
set(dart_gui_glut_srcs ${srcs})

# Library
dart_add_library(dart-gui-glut ${dart_gui_glut_srcs} ${dart_gui_glut_hdrs})
target_link_libraries(dart-gui-glut dart-gui)

# Generate header for this namespace
dart_get_filename_components(header_names "gui glut headers" ${hdrs})
dart_generate_include_header_list(
  gui_glut_headers
  "dart/gui/glut/"
  "gui glut headers"
  ${header_names}
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/glut.hpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/glut.hpp
)

install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/glut.hpp
  DESTINATION include/dart/gui/glut
  COMPONENT headers
)
install(TARGETS dart-gui-glut EXPORT DARTTargets DESTINATION lib)

# Coverage test files
set(dart_coveralls_files "${dart_coveralls_files};${hdrs};${srcs}"
    CACHE INTERNAL "files for coverage tests")
