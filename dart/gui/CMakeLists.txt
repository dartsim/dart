# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")

# Set local target name
set(target_name ${PROJECT_NAME}-gui)
set(component_name gui)

# Add target
dart_add_library(${target_name} ${hdrs} ${srcs})
target_include_directories(
  ${target_name} SYSTEM
  PUBLIC
    ${OPENGL_INCLUDE_DIR}
    ${GLUT_INCLUDE_DIR}
)
target_link_libraries(
  ${target_name}
  PUBLIC
    dart-utils
    ${GLUT_LIBRARY}
    ${OPENGL_LIBRARIES}
    ${PROJECT_NAME}-external-lodepng
    ${PROJECT_NAME}-external-imgui
)

# Component
add_component(${PROJECT_NAME} ${component_name})
add_component_targets(${PROJECT_NAME} ${component_name} ${target_name})
add_component_dependencies(
  ${PROJECT_NAME}
  ${component_name}
  utils
  external-lodepng
  external-imgui
)

# Coverage test
dart_coveralls_add(${hdrs} ${srcs})

# Generate header for this namespace
dart_get_filename_components(header_names "gui headers" ${hdrs})
if(HAVE_OPENSCENEGRAPH)
  list(APPEND header_names "osg/osg.hpp")
endif(HAVE_OPENSCENEGRAPH)
dart_generate_include_header_list(
  gui_headers
  "dart/gui/"
  "gui headers"
  ${header_names}
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/gui.hpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/gui.hpp
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/gui.hpp
  DESTINATION include/dart/gui
  COMPONENT headers
)

# Add subdirectories
if(HAVE_OPENSCENEGRAPH)
  add_subdirectory(osg)
endif(HAVE_OPENSCENEGRAPH)
