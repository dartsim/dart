# Search all header and source files
file(GLOB hdrs "*.h")
file(GLOB srcs "*.cpp")

set(dart_utils_hdrs ${hdrs})
set(dart_utils_srcs ${srcs})

include_directories(SYSTEM ${TINYXML_INCLUDE_DIRS})
include_directories(SYSTEM ${TINYXML2_INCLUDE_DIRS})

# Add subdirectories
add_subdirectory(sdf)
if(urdfdom_FOUND)
  add_subdirectory(urdf)
endif()

# Library
dart_add_library(dart-utils ${dart_utils_srcs} ${dart_utils_hdrs})
target_link_libraries(
  dart-utils
  dart
  ${TINYXML_LIBRARIES}
  ${TINYXML2_LIBRARIES}
)

# Generate header for this namespace
dart_get_filename_components(header_names "utils headers" ${hdrs})
if(urdfdom_FOUND)
  list(APPEND header_names "urdf/urdf.h")
endif()
set(
  header_names
  ${header_names}
  sdf/sdf.h
)
dart_generate_include_header_list(
  utils_headers
  "dart/utils/"
  "utils headers"
  ${header_names}
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/utils.h.in
  ${CMAKE_CURRENT_BINARY_DIR}/utils.h
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/utils.h
  DESTINATION include/dart/utils
  COMPONENT headers
)
install(TARGETS dart-utils EXPORT DARTTargets DESTINATION lib)
