# Search all header and source files
file(GLOB hdrs "*.hpp" "*.h")
file(GLOB srcs "*.cpp")

include_directories(SYSTEM ${OPENGL_INCLUDE_DIR})
include_directories(SYSTEM ${GLUT_INCLUDE_DIR})

# Add subdirectories
if(BUILD_DART_GUI_OSG)
  add_subdirectory(osg)
endif(BUILD_DART_GUI_OSG)

set(dart_gui_hdrs ${dart_gui_hdrs} ${hdrs})
set(dart_gui_srcs ${dart_gui_srcs} ${srcs})

# Library
dart_add_library(dart-gui ${srcs} ${hdrs})
target_link_libraries(dart-gui dart-utils ${GLUT_LIBRARY} ${OPENGL_LIBRARIES})

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
install(TARGETS dart-gui EXPORT DART-GUITargets DESTINATION lib)
install(EXPORT DART-GUITargets DESTINATION share/dart)

# Coverage test files
set(dart_coveralls_files "${dart_coveralls_files};${hdrs};${srcs}"
    CACHE INTERNAL "files for coverage tests")
