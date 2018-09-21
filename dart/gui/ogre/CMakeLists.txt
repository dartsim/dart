# Dependency checks
if (NOT TARGET dart-gui)
  return()
endif()

find_package(OGRE 1.7 QUIET)
message(STATUS "[DEBUG] OGRE_FOUND: ${OGRE_FOUND}")
dart_check_optional_package(OGRE "dart-gui-ogre" "ogre" "1.9")

# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")

set(dart_gui_ogre_hdrs ${hdrs})
set(dart_gui_ogre_srcs ${srcs})

# Set local target name
set(target_name ${PROJECT_NAME}-gui-ogre)
set(component_name gui-ogre)

# Add target
dart_add_library(${target_name} ${hdrs} ${srcs} ${dart_gui_ogre_hdrs} ${dart_gui_ogre_srcs})
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

# Generate header for this namespace
dart_get_filename_components(header_names "gui ogre headers" ${hdrs})
dart_generate_include_header_list(
  gui_ogre_headers
  "dart/gui/ogre/"
  "gui ogre headers"
  ${header_names}
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/ogre.hpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/ogre.hpp
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/ogre.hpp
  DESTINATION include/dart/gui/ogre
  COMPONENT headers
)
