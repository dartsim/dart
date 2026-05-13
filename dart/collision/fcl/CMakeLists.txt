# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")

if(NOT DART_BUILD_COLLISION_FCL)
  set(DART_HAVE_FCL FALSE CACHE BOOL "Check if fcl found." FORCE)
  return()
endif()

message(STATUS "Configuring dart/collision/fcl (DART_BUILD_COLLISION_FCL=${DART_BUILD_COLLISION_FCL})")

set(DART_HAVE_FCL TRUE CACHE BOOL "Check if fcl found." FORCE)

set(target_name ${PROJECT_NAME}-collision-reference-fcl)

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
if(TARGET ${PROJECT_NAME}-collision-native)
  target_link_libraries(${target_name} PUBLIC ${PROJECT_NAME}-collision-native)
  target_compile_definitions(${target_name} PRIVATE DART_FCL_HAS_DART=1)
endif()
target_compile_definitions(${target_name}
  PRIVATE
    DART_BUILDING_COLLISION_FCL
)

add_component_targets(${PROJECT_NAME} collision-reference-fcl ${target_name})
add_component_dependencies(${PROJECT_NAME} collision-reference-fcl dart)
add_component_dependency_packages(${PROJECT_NAME} collision-reference-fcl fcl)

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
