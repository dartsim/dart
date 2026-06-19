add_subdirectory(convhull_3d)
if(NOT DART_USE_SYSTEM_IMGUI)
  add_subdirectory(imgui)
endif()
add_subdirectory(ikfast)

set(lodepng_target_name ${PROJECT_NAME}-external-lodepng)
set(lodepng_component_name external-lodepng)
set(lodepng_compat_header
  "${DART_BINARY_DIR}/dart/external/lodepng/lodepng.h"
)
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/lodepng_compat.h.in"
  "${lodepng_compat_header}"
  COPYONLY
)
dart_add_library(
  ${lodepng_target_name}
  "${lodepng_compat_header}"
  lodepng_compat.cpp
)
target_include_directories(${lodepng_target_name}
  PUBLIC
    $<BUILD_INTERFACE:${DART_BINARY_DIR}>
    $<BUILD_INTERFACE:${DART_BINARY_DIR}/dart/external/lodepng>
    $<INSTALL_INTERFACE:include>
    $<INSTALL_INTERFACE:include/dart/external/lodepng>
)
add_component(${PROJECT_NAME} ${lodepng_component_name})
add_component_targets(
  ${PROJECT_NAME}
  ${lodepng_component_name}
  ${lodepng_target_name}
)
install(
  FILES "${lodepng_compat_header}"
  DESTINATION include/dart/external/lodepng
  COMPONENT headers
)
dart_format_add(lodepng_compat.cpp)

add_subdirectory(odelcpsolver)
