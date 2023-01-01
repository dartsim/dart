# Set local target name
set(target_name ${PROJECT_NAME}-gui)
set(component_name gui)

# Dependency checks
dart_check_dependent_target(${target_name} dart-io)

dart_find_package(OpenGL)
dart_check_optional_package(OPENGL "dart-gui" "OpenGL")
dart_check_optional_package(OPENGL_GLU "dart-gui" "OpenGL GLU")

# Search all header and source files
file(GLOB hdrs "*.hpp" "*.h" "detail/*.hpp")
file(GLOB srcs "*.cpp" "*.c" "detail/*.cpp")

# Add target
dart_add_library(${target_name}
  ${hdrs} ${srcs} ${dart_gui_headers} ${dart_gui_sources}
)
target_link_libraries(${target_name}
  PUBLIC
    dart-io
    OpenGL::GL
    OpenGL::GLU
    ${PROJECT_NAME}-external-imgui
)
if(APPLE)
  # Suppress "deprecated-declarations" until we abandon glut.
  target_compile_options(${target_name}
    PUBLIC
      -Wno-deprecated-declarations
  )
endif()

# Component
add_component(${PROJECT_NAME} ${component_name})
add_component_targets(${PROJECT_NAME} ${component_name} ${target_name})
add_component_dependencies(
  ${PROJECT_NAME}
  ${component_name}
  io
  external-imgui
)
add_component_dependency_packages(${PROJECT_NAME} ${component_name} OpenGL)

# Add subdirectories
add_subdirectory(glut)
add_subdirectory(osg)

# Generate header for this namespace
dart_get_filename_components(header_names "gui headers" ${hdrs})

# Remove deprecated files from the list
list(REMOVE_ITEM header_names
  "GLFuncs.hpp"
  "GlutWindow.hpp"
  "GraphWindow.hpp"
  "LoadGlut.hpp"
  "MotionBlurSimWindow.hpp"
  "SimWindow.hpp"
  "SoftSimWindow.hpp"
  "Win2D.hpp"
  "Win3D.hpp"
)

dart_generate_include_header_file(
  "${CMAKE_CURRENT_BINARY_DIR}/gui.hpp"
  "dart/gui/"
  ${header_names}
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/gui.hpp
  DESTINATION include/dart/gui
  COMPONENT headers
)

dart_format_add(${hdrs} ${srcs} ${dart_gui_headers} ${dart_gui_sources})
