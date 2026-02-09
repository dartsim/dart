# Dependency checks
if(NOT DART_BUILD_COLLISION_ODE)
  set(DART_HAVE_ODE FALSE CACHE BOOL "Check if ODE found." FORCE)
  return()
endif()

message(STATUS "Configuring dart/collision/ode (DART_BUILD_COLLISION_ODE=${DART_BUILD_COLLISION_ODE})")

# Skip find_package when ODE was already provided via FetchContent
# (ODE_FOUND and ODE::ODE target already set in dart_find_dependencies.cmake)
if(NOT ODE_FOUND OR NOT TARGET ODE::ODE)
  dart_find_package(ODE)
  if(NOT ODE_FOUND)
    message(
      FATAL_ERROR
      "DART_BUILD_COLLISION_ODE is ON but ODE (>= 0.13) was not found. "
      "Please install libode-dev or disable DART_BUILD_COLLISION_ODE."
    )
  endif()
endif()
set(DART_HAVE_ODE TRUE CACHE BOOL "Check if ODE found." FORCE)
set(_dart_ode_public_link ${ODE_LIBRARIES})
if(NOT _dart_ode_public_link)
  if(TARGET ODE::ODE)
    set(_dart_ode_public_link ODE::ODE)
  else()
    message(
      FATAL_ERROR
      "ODE was found but did not export ODE::ODE or populate ODE_LIBRARIES. "
      "Please install a complete ODE package."
    )
  endif()
endif()

# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")
file(GLOB detail_hdrs "detail/*.hpp")
file(GLOB detail_srcs "detail/*.cpp")

set(target_name ${PROJECT_NAME}-collision-ode)
dart_add_library(${target_name} ${srcs} ${detail_srcs})
target_include_directories(${target_name}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)
if(ODE_INCLUDE_DIRS)
  target_include_directories(${target_name} PUBLIC ${ODE_INCLUDE_DIRS})
endif()
target_link_libraries(${target_name}
  PUBLIC
    dart
    ${_dart_ode_public_link}
)
if(TARGET ${PROJECT_NAME}-collision-experimental)
  target_link_libraries(${target_name} PUBLIC ${PROJECT_NAME}-collision-experimental)
  target_compile_definitions(${target_name} PRIVATE DART_ODE_HAS_EXPERIMENTAL=1)
endif()
target_compile_definitions(${target_name}
  PRIVATE
    DART_BUILDING_COLLISION_ODE
)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(${target_name} PRIVATE -Wno-deprecated-enum-enum-conversion)
endif()

add_component_targets(${PROJECT_NAME} collision-ode ${target_name})
add_component_dependencies(${PROJECT_NAME} collision-ode dart)
add_component_dependency_packages(${PROJECT_NAME} collision-ode ODE)

# Generate header for this namespace
dart_get_filename_components(header_names "collision_ode headers" ${hdrs})
dart_generate_component_headers(
  COMPONENT_NAME ode
  TARGET_DIR "dart/collision/ode/"
  OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}"
  HEADERS ${header_names}
  SOURCE_HEADERS ${hdrs}
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/All.hpp ${CMAKE_CURRENT_BINARY_DIR}/ode.hpp
  DESTINATION include/dart/collision/ode
  COMPONENT headers
)

install(
  FILES ${detail_hdrs}
  DESTINATION include/dart/collision/ode/detail
  COMPONENT headers
)

dart_install_compat_headers(COMPAT_HEADERS ${DART_GENERATED_COMPAT_HEADERS} DESTINATION_PREFIX include/dart/collision/ode)

dart_format_add(${hdrs} ${srcs} ${detail_hdrs} ${detail_srcs})
