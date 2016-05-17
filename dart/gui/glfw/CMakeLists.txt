include_directories(SYSTEM ${glfw_INCLUDE_DIRS})

message(STATUS "glfw3_INCLUDE_DIRS: ${glfw3_INCLUDE_DIRS}")

# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")

set(dart_gui_glfw_hdrs ${hdrs})
set(dart_gui_glfw_srcs ${srcs})

# Library
dart_add_library(dart-gui-glfw ${dart_gui_glfw_srcs} ${dart_gui_glfw_hdrs})
target_link_libraries(dart-gui-glfw dart-gui glfw)

message(STATUS "glfw3_LIBRARIES: ${glfw3_LIBRARIES}")

# Generate header for this namespace
dart_get_filename_components(header_names "gui glfw headers" ${hdrs})
list(APPEND header_names "render/render.hpp")
dart_generate_include_header_list(
  gui_glfw_headers
  "dart/gui/glfw/"
  "gui glfw headers"
  ${header_names}
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/glfw.hpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/glfw.hpp
)

install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/glfw.hpp
  DESTINATION include/dart/gui/glfw
  COMPONENT headers
)
install(TARGETS dart-gui-glfw EXPORT DARTTargets DESTINATION lib)

# Coverage test files
set(dart_coveralls_files "${dart_coveralls_files};${hdrs};${srcs}"
    CACHE INTERNAL "files for coverage tests")
