# Search all header and source files
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.hpp")

set(kido_utils_hdrs ${hdrs})
set(kido_utils_srcs ${srcs})

include_directories(SYSTEM ${urdfdom_INCLUDE_DIRS})
include_directories(SYSTEM ${TINYXML_INCLUDE_DIRS})
include_directories(SYSTEM ${TINYXML2_INCLUDE_DIRS})

# Add subdirectories
add_subdirectory(sdf)
add_subdirectory(urdf)

# Library
kido_add_library(kido-utils ${kido_utils_srcs} ${kido_utils_hdrs})
target_link_libraries(
  kido-utils
  kido
  ${urdfdom_LIBRARIES}
  ${TINYXML_LIBRARIES}
  ${TINYXML2_LIBRARIES}
)

# Generate header for this namespace
kido_get_filename_components(header_names "utils headers" ${hdrs})
list(APPEND header_names "sdf/sdf.hpp")
list(APPEND header_names "urdf/urdf.hpp")
kido_generate_include_header_list(
  utils_headers
  "kido/utils/"
  "utils headers"
  ${header_names}
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/utils.hpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/utils.hpp
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/utils.hpp
  DESTINATION include/kido/utils
  COMPONENT headers
)
install(TARGETS kido-utils EXPORT KIDOTargets DESTINATION lib)

