# Dependency checks
dart_find_package(urdfdom)

# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")

# Set local target name
set(target_name ${PROJECT_NAME}-utils-urdf)
set(component_name utils-urdf)

set(DART_UTILS_BACKWARDCOMPATIBILITY_HPP_IN
  ${CMAKE_CURRENT_SOURCE_DIR}/BackwardCompatibility.hpp.in
)
set(DART_UTILS_BACKWARDCOMPATIBILITY_HPP_OUT
  ${CMAKE_CURRENT_BINARY_DIR}/BackwardCompatibility.hpp
)
if(DART_VERBOSE)
  message(STATUS ${DART_UTILS_BACKWARDCOMPATIBILITY_HPP_OUT})
endif()
configure_file(
  ${DART_UTILS_BACKWARDCOMPATIBILITY_HPP_IN}
  ${DART_UTILS_BACKWARDCOMPATIBILITY_HPP_OUT}
  @ONLY
)
install(FILES ${DART_UTILS_BACKWARDCOMPATIBILITY_HPP_OUT}
  DESTINATION include/dart/utils/urdf
)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

# Add target
dart_add_library(${target_name} ${hdrs} ${srcs})
target_include_directories(
  ${target_name} SYSTEM
  PUBLIC ${urdfdom_INCLUDE_DIRS}
)
target_link_libraries(
  ${target_name}
  dart-utils
  ${urdfdom_LIBRARIES}
)

# Component
add_component(${PROJECT_NAME} ${component_name})
add_component_targets(${PROJECT_NAME} ${component_name} ${target_name})
add_component_dependencies(${PROJECT_NAME} ${component_name} utils)
add_component_dependency_packages(${PROJECT_NAME} ${component_name} urdfdom)

# Generate header for this namespace
dart_get_filename_components(header_names "utils_urdf headers" ${hdrs})
# TODO: remove below line once the files are completely removed.
list(REMOVE_ITEM header_names "URDFTypes.hpp")
dart_generate_include_header_file(
  "${CMAKE_CURRENT_BINARY_DIR}/urdf.hpp"
  "dart/utils/urdf/"
  ${header_names}
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/urdf.hpp
  DESTINATION include/dart/utils/urdf
  COMPONENT headers
)
