# Search all header and source files
file(GLOB hdrs "*.h")
file(GLOB srcs "*.cpp")

# Set local target name
set(target_name ${PROJECT_NAME}-external-odelcpsolver)
set(component_name external-odelcpsolver)

dart_add_library(${target_name} ${hdrs} ${srcs})
include_directories(
  ${target_name}
  PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}"
)

# Component
add_component(${PROJECT_NAME} ${component_name})
add_component_targets(${PROJECT_NAME} ${component_name} ${target_name})

# Install
install(
  FILES ${hdrs}
  DESTINATION include/dart/external/odelcpsolver
  COMPONENT headers
)
