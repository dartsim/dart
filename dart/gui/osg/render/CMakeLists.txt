# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")

set(dart_gui_osg_hdrs ${dart_gui_osg_hdrs} ${hdrs} PARENT_SCOPE)
set(dart_gui_osg_srcs ${dart_gui_osg_srcs} ${srcs} PARENT_SCOPE)

dart_get_filename_components(header_names "gui osg render headers" ${hdrs})
dart_generate_component_headers(
  COMPONENT_NAME render
  TARGET_DIR "dart/gui/osg/render/"
  OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}"
  HEADERS ${header_names}
  SOURCE_HEADERS ${hdrs}
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/All.hpp ${CMAKE_CURRENT_BINARY_DIR}/render.hpp ${DART_GENERATED_COMPAT_HEADERS}
  DESTINATION include/dart/gui/osg/render
  COMPONENT headers
)
