# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")

set(dart_gui_osg_hdrs ${dart_gui_osg_hdrs} ${hdrs} PARENT_SCOPE)
set(dart_gui_osg_srcs ${dart_gui_osg_srcs} ${srcs} PARENT_SCOPE)

dart_get_filename_components(header_names "gui osg render headers" ${hdrs})
dart_generate_include_header_file(
  "${CMAKE_CURRENT_BINARY_DIR}/render.hpp"
  "dart/gui/osg/render/"
  ${header_names}
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/render.hpp
  DESTINATION include/dart/gui/osg/render
  COMPONENT headers
)

dart_format_add(
  HeightmapShapeNode.hpp
  PointCloudShapeNode.hpp
  PointCloudShapeNode.cpp
  VoxelGridShapeNode.hpp
  VoxelGridShapeNode.cpp
)
