# Copyright (c) 2011, The DART development contributors

set(target_name ${PROJECT_NAME}-gui-experimental)
set(component_name gui-experimental)

file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")

dart_add_library(${target_name} ${hdrs} ${srcs})
target_include_directories(
  ${target_name}
  PUBLIC
    $<BUILD_INTERFACE:${DART_SOURCE_DIR}>
    $<BUILD_INTERFACE:${DART_BINARY_DIR}>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(${target_name} PUBLIC cxx_std_20)
target_compile_definitions(${target_name} PRIVATE DART_BUILDING_DART_GUI)
target_link_libraries(${target_name} PUBLIC dart)

add_component(${PROJECT_NAME} ${component_name})
add_component_targets(${PROJECT_NAME} ${component_name} ${target_name})
add_component_dependencies(${PROJECT_NAME} ${component_name} dart)

install(
  FILES ${hdrs}
  DESTINATION include/dart/gui/experimental
  COMPONENT headers
)

dart_format_add(${hdrs} ${srcs})
