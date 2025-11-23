# Search all header and source files
file(GLOB hdrs "*.h")
file(GLOB srcs "*.cpp")

# Set local target name
set(target_name ${PROJECT_NAME}-external-odelcpsolver)
set(component_name external-odelcpsolver)

dart_add_library(
  NAME ${target_name}
  HEADERS ${hdrs}
  SOURCES ${srcs}
  PUBLIC_INCLUDE_DIRS ${DART_SOURCE_DIR} ${DART_BINARY_DIR}
  COMPILE_FEATURES cxx_std_17
)

# Component
dart_add_component(
  NAME ${component_name}
  PACKAGE ${PROJECT_NAME}
  TARGETS ${target_name}
)

# Install
install(
  FILES ${hdrs}
  DESTINATION include/dart/external/odelcpsolver
  COMPONENT headers
)
