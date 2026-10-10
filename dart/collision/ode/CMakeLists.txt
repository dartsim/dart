# Dependency checks
dart_find_package(ODE)
dart_check_optional_package(ODE "dart-collision-ode" "ode" "0.16.2")

# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")
file(GLOB detail_hdrs "detail/*.hpp")
file(GLOB detail_srcs "detail/*.cpp")

# Set local target name
set(target_name ${PROJECT_NAME}-collision-ode)
set(component_name collision-ode)

# Add backend sources to core and retain the legacy target for rebuilt consumers.
dart_add_core_headers(${hdrs} ${detail_hdrs})
dart_add_core_sources(${srcs} ${detail_srcs})
set(ode_target ODE::ODE)
get_target_property(ode_alias ODE::ODE ALIASED_TARGET)
if(ode_alias)
  get_target_property(ode_alias_global ODE::ODE ALIAS_GLOBAL)
  if(NOT ode_alias_global)
    set(ode_target ${ode_alias})
  endif()
endif()
get_property(ode_imported_targets DIRECTORY PROPERTY IMPORTED_TARGETS)
if(ode_target IN_LIST ode_imported_targets)
  set_property(TARGET ${ode_target} PROPERTY IMPORTED_GLOBAL TRUE)
endif()
# Defer the installed name so CMake does not export a parent's alias target.
dart_property_add(DART_CORE_LINK_LIBRARIES
  $<BUILD_INTERFACE:${ode_target}> $<INSTALL_INTERFACE:$<1:ODE::ODE>>
)
dart_property_add(DART_CORE_DEPENDENCY_PACKAGES ODE)
add_library(${target_name} INTERFACE)
target_link_libraries(${target_name} INTERFACE dart)

# Component
add_component(${PROJECT_NAME} ${component_name})
add_component_targets(${PROJECT_NAME} ${component_name} ${target_name})
add_component_dependencies(${PROJECT_NAME} ${component_name} dart)
add_component_dependency_packages(${PROJECT_NAME} ${component_name} ODE)

# Generate header for this namespace
dart_get_filename_components(header_names "collision_ode headers" ${hdrs})
dart_generate_include_header_file(
  "${CMAKE_CURRENT_BINARY_DIR}/ode.hpp"
  "dart/collision/ode/"
  ${header_names}
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/ode.hpp
  DESTINATION include/dart/collision/ode
  COMPONENT headers
)

dart_format_add(${hdrs} ${srcs} ${detail_hdrs} ${detail_srcs})
