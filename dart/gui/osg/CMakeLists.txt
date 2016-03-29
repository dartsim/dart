include_directories(SYSTEM ${OPENSCENEGRAPH_INCLUDE_DIRS})

# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")

set(dart_gui_osg_hdrs ${hdrs})
set(dart_gui_osg_srcs ${srcs})

add_subdirectory(render)

# Library
dart_add_library(dart-gui-osg ${dart_gui_osg_srcs} ${dart_gui_osg_hdrs})
target_link_libraries(dart-gui-osg dart-gui ${OPENSCENEGRAPH_LIBRARIES})

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

add_subdirectory(examples EXCLUDE_FROM_ALL)

install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/osg.hpp
  DESTINATION include/dart/gui/osg
  COMPONENT headers
)
install(TARGETS dart-gui-osg EXPORT DARTTargets DESTINATION lib)

# Coverage test files
set(dart_coveralls_files "${dart_coveralls_files};${hdrs};${srcs}"
    CACHE INTERNAL "files for coverage tests")
