# Search all header and source files
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.hpp")

include_directories(SYSTEM ${GLUT_INCLUDE_DIR})

# Add subdirectories
if(HAVE_OPENSCENEGRAPH)
  add_subdirectory(osgKido)
endif(HAVE_OPENSCENEGRAPH)

# Library
kido_add_library(kido-gui ${srcs} ${hdrs})
target_link_libraries(kido-gui kido kido-utils ${GLUT_LIBRARY})

# Generate header for this namespace
kido_get_filename_components(header_names "gui headers" ${hdrs})
kido_generate_include_header_list(
  gui_headers
  "kido/gui/"
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
  DESTINATION include/kido/gui
  COMPONENT headers
)
install(TARGETS kido-gui EXPORT KIDOTargets DESTINATION lib)
