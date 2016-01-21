include_directories(SYSTEM ${OPENSCENEGRAPH_INCLUDE_DIRS})

# Search all header and source files
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.hpp")

set(kido_gui_osg_srcs ${srcs})
set(kido_gui_osg_hdrs ${hdrs})

add_subdirectory(render)

# Library
kido_add_library(kido-gui-osg ${kido_gui_osg_srcs} ${kido_gui_osg_hdrs})
target_link_libraries(kido-gui-osg kido kido-utils ${OPENSCENEGRAPH_LIBRARIES})

# Generate header for this namespace
kido_get_filename_components(header_names "gui osg headers" ${hdrs})
list(APPEND header_names "render/render.hpp")
kido_generate_include_header_list(
  gui_osg_headers
  "kido/gui/osg/"
  "gui osg headers"
  ${header_names}
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/osg.hpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/osg.hpp
)

if(${KIDO_BUILD_EXAMPLES})
  add_subdirectory(examples)
endif(${KIDO_BUILD_EXAMPLES})

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/osg.hpp
  DESTINATION include/kido/gui/osg
  COMPONENT headers
)
install(TARGETS kido-gui-osg EXPORT KIDOTargets DESTINATION lib)
