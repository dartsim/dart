# Search all header and source files
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.hpp")

set(kido_gui_hdrs ${hdrs} PARENT_SCOPE)
set(kido_gui_srcs ${srcs} PARENT_SCOPE)

# Library
#kido_add_library(kido_gui ${srcs} ${hdrs})
#target_link_libraries(
#  kido_gui
#  kido_collision
#  kido_simulation
#  ${KIDO_DEPENDENCIES}
#)

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
#install(TARGETS kido_gui EXPORT KIDOTargets DESTINATION lib)

