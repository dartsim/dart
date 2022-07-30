# Copyright (c) 2011-2022, The DART development contributors

# file(GLOB_RECURSE sources "*.h" "*.cpp")

set(target_name external-convhull_3d)

add_library(${target_name} INTERFACE)
add_library(convhull_3d::convhull_3d ALIAS ${target_name})

target_sources(
  ${target_name}
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/convhull_3d.h>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}${DART_VERSION_MAJOR}>
)

target_include_directories(
  ${target_name}
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}${DART_VERSION_MAJOR}>
)

# target_compile_options(${target_name} PRIVATE -w -fPIC)

# Install targets
install(
  TARGETS ${target_name}
  EXPORT ${target_name}-targets
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(
  EXPORT ${target_name}-targets
  FILE ${target_name}-targets.cmake
  DESTINATION
    ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}${DART_VERSION_MAJOR}
)
