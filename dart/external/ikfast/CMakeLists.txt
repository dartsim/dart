# Search all header and source files
file(GLOB hdrs "*.h")

# Set local target name
set(target_name ${PROJECT_NAME}-external-ikfast)
set(component_name external-ikfast)

add_library(${target_name} STATIC ${hdrs})
target_compile_options(${target_name} PRIVATE -w)
set_target_properties(${target_name} PROPERTIES LINKER_LANGUAGE CXX)

# Component
add_component(${PROJECT_NAME} ${component_name})
add_component_targets(${PROJECT_NAME} ${component_name} ${target_name})

# Install
install(
  FILES ${hdrs}
  DESTINATION include/dart/external/ikfast
  COMPONENT headers
)
