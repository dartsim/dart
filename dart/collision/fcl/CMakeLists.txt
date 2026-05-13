# Search all reference-engine header and source files. The top-level headers in
# this directory are native-backed compatibility facades; old FCL code lives
# under reference/ and is built only by the explicit reference target.
file(GLOB facade_hdrs "*.hpp")
file(GLOB hdrs "reference/*.hpp")
file(GLOB srcs "reference/*.cpp")
file(GLOB compat_hdrs "compat/*.hpp")
dart_format_add(${facade_hdrs} ${compat_hdrs})

if(NOT DART_BUILD_COLLISION_FCL)
  set(DART_HAVE_FCL FALSE CACHE BOOL "Check if fcl found." FORCE)
  install(
    FILES ${compat_hdrs}
    DESTINATION include/dart/collision/fcl
    COMPONENT headers
  )
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

# Generate reference-only headers for this namespace
set(header_names)
foreach(header IN LISTS hdrs)
  file(RELATIVE_PATH header_name "${CMAKE_CURRENT_SOURCE_DIR}" "${header}")
  list(APPEND header_names "${header_name}")
endforeach()
dart_generate_component_headers(
  COMPONENT_NAME fcl
  TARGET_DIR "dart/collision/fcl/"
  OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}"
  HEADERS ${header_names}
  SOURCE_HEADERS ${hdrs}
)

# Install native-backed compatibility headers on the public legacy path. The
# FCL implementation target remains explicit reference-only test/benchmark code.
install(
  FILES ${compat_hdrs}
  DESTINATION include/dart/collision/fcl
  COMPONENT headers
)
