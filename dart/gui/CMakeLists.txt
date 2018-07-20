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
    set(HAVE_GLUT TRUE)
  else()
    set(HAVE_GLUT FALSE)
  endif()
endif()

# Search all header and source files
file(GLOB hdrs "*.hpp" "*.h" "detail/*.hpp")
file(GLOB srcs "*.cpp" "*.c" "detail/*.cpp")

function(dart_add_gui_headers)
  dart_property_add(DART_GUI_HEADERS ${ARGN})
endfunction()

function(dart_add_gui_sources)
  dart_property_add(DART_GUI_SOURCES ${ARGN})
endfunction()

# Add required subdirectory
add_subdirectory(glut)
add_subdirectory(glfw)

get_property(dart_gui_headers GLOBAL PROPERTY DART_GUI_HEADERS)
get_property(dart_gui_sources GLOBAL PROPERTY DART_GUI_SOURCES)

# Set local target name
set(target_name ${PROJECT_NAME}-gui)
set(component_name gui)

# Add target
dart_add_library(${target_name}
  ${hdrs} ${srcs} ${dart_gui_headers} ${dart_gui_sources}
)
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
    ${GLUT_LIBRARIES}
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

# Add subdirectories
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

set(
  header_names
  ${header_names}
  glut/glut.hpp
)
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
