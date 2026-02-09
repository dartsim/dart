# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")

set(DART_BUILD_COLLISION_FCL TRUE CACHE BOOL "Build FCL collision backend." FORCE)

set(target_name ${PROJECT_NAME}-collision-fcl)

dart_add_library(${target_name} ${srcs})
target_include_directories(${target_name}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${target_name}
  PUBLIC
    dart
    fcl
)
if(TARGET ${PROJECT_NAME}-collision-experimental)
  target_link_libraries(${target_name} PUBLIC ${PROJECT_NAME}-collision-experimental)
  target_compile_definitions(${target_name} PRIVATE DART_FCL_HAS_EXPERIMENTAL=1)
endif()
target_compile_definitions(${target_name}
  PRIVATE
    DART_BUILDING_COLLISION_FCL
)

add_component(${PROJECT_NAME} collision-fcl)
add_component_targets(${PROJECT_NAME} collision-fcl ${target_name})
add_component_dependencies(${PROJECT_NAME} collision-fcl dart)
add_component_dependency_packages(${PROJECT_NAME} collision-fcl fcl)

# Generate header for this namespace
dart_get_filename_components(header_names "collision_fcl headers" ${hdrs})
dart_generate_component_headers(
  COMPONENT_NAME fcl
  TARGET_DIR "dart/collision/fcl/"
  OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}"
  HEADERS ${header_names}
  SOURCE_HEADERS ${hdrs}
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/All.hpp ${CMAKE_CURRENT_BINARY_DIR}/fcl.hpp ${DART_GENERATED_COMPAT_HEADERS}
  DESTINATION include/dart/collision/fcl
  COMPONENT headers
)
