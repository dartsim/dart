# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")

set(dart_gui_hdrs ${dart_gui_hdrs} ${hdrs} PARENT_SCOPE)
set(dart_gui_srcs ${dart_gui_srcs} ${srcs} PARENT_SCOPE)

dart_get_filename_components(header_names "gui render headers" ${hdrs})
dart_generate_component_headers(
  COMPONENT_NAME render
  TARGET_DIR "dart/gui/render/"
  OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}"
  HEADERS ${header_names}
  SOURCE_HEADERS ${hdrs}
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/All.hpp ${CMAKE_CURRENT_BINARY_DIR}/render.hpp ${DART_GENERATED_COMPAT_HEADERS}
  DESTINATION include/dart/gui/render
  COMPONENT headers
)

dart_format_add(
  detail/heightmap_shape_geometry.hpp
  heightmap_shape_node.hpp
  point_cloud_shape_node.hpp
  point_cloud_shape_node.cpp
  voxel_grid_shape_node.hpp
  voxel_grid_shape_node.cpp
)
