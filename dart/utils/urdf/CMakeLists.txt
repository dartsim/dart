# Dependency checks
dart_find_package(urdfdom)
if(urdfdom_FOUND)
  if(DART_VERBOSE)
    message(STATUS "Looking for urdfdom - ${urdfdom_headers_VERSION} found")
  endif()
else()
  message(WARNING "Looking for urdfdom - NOT found, to use dart-utils-urdf, please install liburdfdom-dev")
  return()
endif()
if(MSVC)
  set(urdfdom_LIBRARIES optimized urdfdom_sensor      debug urdfdom_sensord
                        optimized urdfdom_model_state debug urdfdom_model_stated
                        optimized urdfdom_model       debug urdfdom_modeld
                        optimized urdfdom_world       debug urdfdom_worldd
                        optimized console_bridge      debug console_bridged)
endif()

# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")

# Set local target name
set(target_name ${PROJECT_NAME}-utils-urdf)

set(DART_UTILS_BACKWARDCOMPATIBILITY_HPP_IN
  ${CMAKE_CURRENT_SOURCE_DIR}/BackwardCompatibility.hpp.in
)
set(DART_UTILS_BACKWARDCOMPATIBILITY_HPP_OUT
  ${CMAKE_CURRENT_BINARY_DIR}/BackwardCompatibility.hpp
)
if(DART_VERBOSE)
  message(STATUS ${DART_UTILS_BACKWARDCOMPATIBILITY_HPP_OUT})
endif()
configure_file(
  ${DART_UTILS_BACKWARDCOMPATIBILITY_HPP_IN}
  ${DART_UTILS_BACKWARDCOMPATIBILITY_HPP_OUT}
  @ONLY
)
install(FILES ${DART_UTILS_BACKWARDCOMPATIBILITY_HPP_OUT}
  DESTINATION include/dart/utils/urdf
)

# Add target
dart_add_library(
  NAME ${target_name}
  HEADERS ${hdrs}
  SOURCES ${srcs}
  PUBLIC_LINK_LIBRARIES dart-utils urdfdom
)

# Component
dart_add_component(
  NAME utils-urdf
  PACKAGE ${PROJECT_NAME}
  TARGETS ${target_name}
  DEPENDENCIES utils
  DEPENDENCY_PACKAGES urdfdom
  COMPONENT_HEADERS ${hdrs}
  COMPONENT_HEADER_SOURCES ${hdrs}
  COMPONENT_TARGET_DIR "dart/utils/urdf/"
  COMPONENT_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}"
)
