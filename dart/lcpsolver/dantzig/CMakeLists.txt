file(GLOB hdrs "*.h")
file(GLOB srcs "*.cpp")
set(legacy_hdrs
  "${DART_SOURCE_DIR}/dart/external/odelcpsolver/common.h"
  "${DART_SOURCE_DIR}/dart/external/odelcpsolver/error.h"
  "${DART_SOURCE_DIR}/dart/external/odelcpsolver/lcp.h"
  "${DART_SOURCE_DIR}/dart/external/odelcpsolver/matrix.h"
  "${DART_SOURCE_DIR}/dart/external/odelcpsolver/misc.h"
  "${DART_SOURCE_DIR}/dart/external/odelcpsolver/odeconfig.h"
)

set(target_name ${PROJECT_NAME}-external-odelcpsolver)
set(component_name external-odelcpsolver)

dart_add_library(${target_name} ${hdrs} ${srcs})
target_include_directories(${target_name}
  PUBLIC
    $<BUILD_INTERFACE:${DART_SOURCE_DIR}>
    $<BUILD_INTERFACE:${DART_BINARY_DIR}>
)
target_compile_features(${target_name} PUBLIC cxx_std_17)

# Component
add_component(${PROJECT_NAME} ${component_name})
add_component_targets(${PROJECT_NAME} ${component_name} ${target_name})

install(
  FILES ${hdrs}
  DESTINATION include/dart/lcpsolver/dantzig
  COMPONENT headers
)

install(
  FILES ${legacy_hdrs}
  DESTINATION include/dart/external/odelcpsolver
  COMPONENT headers
)
