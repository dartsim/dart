# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")

set(dart_gui_osg_hdrs ${hdrs})
set(dart_gui_osg_srcs ${srcs})

add_subdirectory(render)

# Set local target name
set(target_name ${PROJECT_NAME}-gui-osg)
set(component_name gui-osg)

# Add target
dart_add_library(${target_name} ${hdrs} ${srcs} ${dart_gui_osg_hdrs} ${dart_gui_osg_srcs})
target_include_directories(
  ${target_name} SYSTEM
  PUBLIC ${OPENSCENEGRAPH_INCLUDE_DIRS}
)
target_link_libraries(
  ${target_name}
  dart-gui
  ${OPENSCENEGRAPH_LIBRARIES}
)

# Component
add_component(${PROJECT_NAME} ${component_name})
add_component_targets(${PROJECT_NAME} ${component_name} ${target_name})
add_component_dependencies(${PROJECT_NAME} ${component_name} gui)

# Coverage test
dart_coveralls_add(${hdrs} ${srcs} ${dart_gui_osg_hdrs} ${dart_gui_osg_srcs})

# Generate header for this namespace
dart_get_filename_components(header_names "gui osg headers" ${hdrs})
list(APPEND header_names "render/render.hpp")
dart_generate_include_header_list(
  gui_osg_headers
  "dart/gui/osg/"
  "gui osg headers"
  ${header_names}
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/osg.hpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/osg.hpp
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/osg.hpp
  DESTINATION include/dart/gui/osg
  COMPONENT headers
)
