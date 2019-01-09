# Search all header and source files
file(GLOB hdrs "*.h")
file(GLOB srcs "*.cpp")

# Set local target name
set(target_name ${PROJECT_NAME}-external-imgui)
set(component_name external-imgui)

dart_add_library(${target_name} ${hdrs} ${srcs})
target_include_directories(${target_name}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

if(CMAKE_COMPILER_IS_GNUCXX)
  target_compile_options(${target_name} PRIVATE -w)
endif()

# Component
add_component(${PROJECT_NAME} ${component_name})
add_component_targets(${PROJECT_NAME} ${component_name} ${target_name})

# Install
install(
  FILES ${hdrs}
  DESTINATION include/dart/external/imgui
  COMPONENT headers
)
