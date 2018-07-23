# Dependency checks
find_package(glfw3 QUIET)
dart_check_optional_package(glfw3 "dart-gui" "glfw3")

find_package(Threads QUIET)
dart_check_optional_package(Threads "dart-gui" "Threads")

# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")
file(GLOB detail_hdrs "detail/*.hpp")
file(GLOB detail_srcs "detail/*.cpp")

# Set local target name
set(target_name ${PROJECT_NAME}-gui-glfw)
set(component_name gui-glfw)

# Add target
dart_add_library(${target_name} ${hdrs} ${srcs} ${detail_hdrs} ${detail_srcs})
target_link_libraries(
  ${target_name}
  PUBLIC
    dart-gui
    glfw
    ${CMAKE_THREAD_LIBS_INIT}
)

# Component
add_component(${PROJECT_NAME} ${component_name})
add_component_targets(${PROJECT_NAME} ${component_name} ${target_name})
add_component_dependencies(${PROJECT_NAME} ${component_name} dart)

# Generate header for this namespace
dart_get_filename_components(header_names "gui_glfw headers" ${hdrs})
dart_generate_include_header_list(
  gui_glfw_headers
  "dart/gui/glfw/"
  "gui_glfw headers"
  ${header_names}
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/glfw.hpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/glfw.hpp
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/glfw.hpp
  DESTINATION include/dart/gui/glfw
  COMPONENT headers
)

dart_format_add(${hdrs} ${srcs} ${detail_hdrs} ${detail_srcs})
