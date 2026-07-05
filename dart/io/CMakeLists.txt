# Set local target/component names
set(target_name ${PROJECT_NAME}-io)
set(component_name io)
set(component_dependencies)

# Dependency checks
dart_check_dependent_target(${target_name} dart-utils)

# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")
if(NOT DART_ENABLE_SDFORMAT)
  list(REMOVE_ITEM hdrs "${CMAKE_CURRENT_SOURCE_DIR}/sdf_writer.hpp")
  list(REMOVE_ITEM srcs "${CMAKE_CURRENT_SOURCE_DIR}/sdf_writer.cpp")
endif()

# Add target
dart_add_library(${target_name} ${hdrs} ${srcs})
target_link_libraries(${target_name} PUBLIC dart-utils)

if(TARGET dart-utils-urdf)
  target_link_libraries(${target_name} PUBLIC dart-utils-urdf)
  target_compile_definitions(${target_name} PUBLIC DART_IO_HAS_URDF=1)
  list(APPEND component_dependencies utils-urdf)
else()
  target_compile_definitions(${target_name} PUBLIC DART_IO_HAS_URDF=0)
endif()

# Optional OpenUSD (pxr) scene loader, compiled into dart-io only when
# DART_BUILD_IO_USD is ON. The subdirectory wires the pxr dependency and sets
# DART_IO_HAS_USD=1; otherwise the front door in read.cpp emits a clean
# "USD support is not available" diagnostic.
if(DART_BUILD_IO_USD)
  add_subdirectory(usd)
else()
  target_compile_definitions(${target_name} PUBLIC DART_IO_HAS_USD=0)
endif()

# Component
add_component(${PROJECT_NAME} ${component_name})
add_component_targets(${PROJECT_NAME} ${component_name} ${target_name})
add_component_dependencies(
  ${PROJECT_NAME}
  ${component_name}
  utils
  ${component_dependencies}
)

# In a static build the USD loader's private pxr link is exported from the
# installed dart-io target, so register pxr as a component dependency package.
# The generated io component config then loads dart_find_pxr.cmake and resolves
# OpenUSD for downstream static links. Shared builds resolve pxr at libdart-io
# link time, so this is static-only (mirrors dart-simulation's EnTT/Taskflow/
# spdlog registration).
if(DART_BUILD_IO_USD AND NOT BUILD_SHARED_LIBS)
  add_component_dependency_packages(${PROJECT_NAME} ${component_name} pxr)
endif()

# Generate header for this namespace
dart_get_filename_components(header_names "io headers" ${hdrs})
dart_generate_component_headers(
  COMPONENT_NAME io
  TARGET_DIR "dart/io/"
  OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}"
  HEADERS ${header_names}
  SOURCE_HEADERS ${hdrs}
)

# Install (includes All.hpp, io.hpp for component compatibility,
# and generated PascalCase headers for case compatibility)
install(
  FILES
    ${hdrs}
    ${CMAKE_CURRENT_BINARY_DIR}/All.hpp
    ${CMAKE_CURRENT_BINARY_DIR}/io.hpp
  DESTINATION include/dart/io
  COMPONENT headers
)

# Install generated compatibility headers preserving subdirectory structure
dart_install_compat_headers(
  COMPAT_HEADERS ${DART_GENERATED_COMPAT_HEADERS}
  DESTINATION_PREFIX include/dart/io
)

dart_format_add(${hdrs} ${srcs})
