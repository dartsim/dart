# Dependency checks
if (NOT TARGET dart-utils)
  return()
endif()

find_package(OpenGL QUIET)
dart_check_optional_package(OPENGL "dart-gui" "OpenGL")

if(WIN32 AND NOT CYGWIN)
  set(GLUT_INCLUDE_DIR "${CMAKE_INSTALL_PREFIX}/include")
  set(GLUT_LIBRARIES glut32)
  set(HAVE_GLUT TRUE)
else()
  find_package(GLUT QUIET)
  dart_check_optional_package(GLUT "dart-gui" "freeglut3")
  if(GLUT_FOUND)
    set(GLUT_LIBRARIES ${GLUT_glut_LIBRARY})
    set(HAVE_GLUT TRUE)
  else()
    set(HAVE_GLUT FALSE)
  endif()
endif()

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
# Hide symbols by default
if(COMPILER_SUPPORTS_VISIBILITY)
  target_compile_options(${target_name} PRIVATE -fvisibility=hidden)
endif()

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

# Add subdirectories
add_subdirectory(osg)

# Generate export header
dart_add_export_file(${target_name} COMPONENT_PATH gui)

# Generate header for this namespace
dart_get_filename_components(header_names "gui headers" ${hdrs})
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
