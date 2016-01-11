# Search all header and source files
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.h")

set(dart_gui_hdrs ${hdrs} PARENT_SCOPE)
set(dart_gui_srcs ${srcs} PARENT_SCOPE)

# Library
#dart_add_library(dart_gui ${srcs} ${hdrs})
#target_link_libraries(
#  dart_gui
#  dart_collision
#  dart_simulation
#  ${KIDO_DEPENDENCIES}
#)

# Generate header for this namespace
dart_get_filename_components(header_names "gui headers" ${hdrs})
dart_generate_include_header_list(
  gui_headers
  "dart/gui/"
  "gui headers"
  ${header_names}
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/gui.h.in
  ${CMAKE_CURRENT_BINARY_DIR}/gui.h
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/gui.h
  DESTINATION include/dart/gui
  COMPONENT headers
)
#install(TARGETS dart_gui EXPORT DARTTargets DESTINATION lib)

