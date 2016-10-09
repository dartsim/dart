# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")

function(dart_add_utils_headers)
  dart_property_add(DART_UTILS_HEADERS ${ARGN})
endfunction(dart_add_utils_headers)

function(dart_add_utils_sources)
  dart_property_add(DART_UTILS_SOURCES ${ARGN})
endfunction(dart_add_utils_sources)

# Add subdirectories
add_subdirectory(sdf)

get_property(dart_utils_headers GLOBAL PROPERTY DART_UTILS_HEADERS)
get_property(dart_utils_sources GLOBAL PROPERTY DART_UTILS_SOURCES)

# Set local target name
set(target_name ${PROJECT_NAME}-utils)
set(component_name utils)

# Add target
dart_add_library(${target_name} ${hdrs} ${srcs} ${dart_utils_headers} ${dart_utils_sources})
target_include_directories(
  ${target_name} SYSTEM
  PUBLIC
    ${TINYXML_INCLUDE_DIRS}
    ${TINYXML2_INCLUDE_DIRS}
)
target_link_libraries(
  ${target_name}
  dart-collision-bullet
  ${TINYXML_LIBRARIES}
  ${TINYXML2_LIBRARIES}
)

# Component
add_component(${PROJECT_NAME} ${component_name})
add_component_targets(${PROJECT_NAME} ${component_name} ${target_name})
add_component_dependencies(${PROJECT_NAME} ${component_name} collision-bullet)

# Coverage test
dart_coveralls_add(${hdrs} ${srcs} ${dart_utils_headers} ${dart_utils_sources})

# Generate header for this namespace
dart_get_filename_components(header_names "utils headers" ${hdrs})
if(urdfdom_FOUND)
  list(APPEND header_names "urdf/urdf.hpp")
endif()
set(
  header_names
  ${header_names}
  sdf/sdf.hpp
)
dart_generate_include_header_list(
  utils_headers
  "dart/utils/"
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
  DESTINATION include/dart/utils
  COMPONENT headers
)

# Add subdirectories (components)
if(urdfdom_FOUND)
  add_subdirectory(urdf)
endif()
