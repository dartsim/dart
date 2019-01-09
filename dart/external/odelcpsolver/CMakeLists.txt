# Search all header and source files
file(GLOB hdrs "*.h")
file(GLOB srcs "*.cpp")

# Set local target name
set(target_name ${PROJECT_NAME}-external-odelcpsolver)
set(component_name external-odelcpsolver)

dart_add_library(${target_name} ${hdrs} ${srcs})
target_include_directories(${target_name}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}>
)
if(CMAKE_VERSION VERSION_LESS 3.8.2)
  target_compile_options(${target_name} PUBLIC -std=c++11)
else()
  target_compile_features(${target_name} PUBLIC cxx_std_11)
endif()

# Component
add_component(${PROJECT_NAME} ${component_name})
add_component_targets(${PROJECT_NAME} ${component_name} ${target_name})

# Install
install(
  FILES ${hdrs}
  DESTINATION include/dart/external/odelcpsolver
  COMPONENT headers
)
