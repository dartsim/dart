# Dependency checks

# Bullet. Force MODULE mode to use the FindBullet.cmake file distributed with
# CMake. Otherwise, we may end up using the BulletConfig.cmake file distributed
# with Bullet, which uses relative paths and may break transitive dependencies.
dart_find_package(Bullet)
dart_check_optional_package(BULLET "dart" "Bullet" "3.05")
if(DART_USE_SYSTEM_BULLET)
  # Test whether Bullet was built with double precision. If so, we need to
  # define the BT_USE_DOUBLE_PRECISION pre-processor directive before including
  # any Bullet headers. This is a workaround for the fact that Bullet does not
  # add the definition to BULLET_DEFINITIONS or generate a #cmakedefine header.
  include(CheckCXXSourceCompiles)
  include(CMakePushCheckState)
  cmake_push_check_state(RESET)
  set(CMAKE_REQUIRED_FLAGS "-w")
  set(CMAKE_REQUIRED_DEFINITIONS "-DBT_USE_DOUBLE_PRECISION")
  set(CMAKE_REQUIRED_INCLUDES "${BULLET_INCLUDE_DIRS}")
  # The Bullet target links the libraries DARTFindBullet selected.
  set(CMAKE_REQUIRED_LIBRARIES Bullet)
  unset(BT_USE_DOUBLE_PRECISION CACHE)
  check_cxx_source_compiles(
    "
      #include <btBulletCollisionCommon.h>
      int main()
      {
        btVector3 v(0., 0., 1.);
        btStaticPlaneShape planeShape(v, 0.);
        return 0;
      }
    "
    BT_USE_DOUBLE_PRECISION
  )
  cmake_pop_check_state()
else()
  # FetchContent builds Bullet with USE_DOUBLE_PRECISION=ON, so keep DART in sync.
  set(BT_USE_DOUBLE_PRECISION TRUE)
endif()

if(DART_VERBOSE)
  if(BT_USE_DOUBLE_PRECISION)
    message(STATUS "Looking for Bullet - found (double precision)")
  else()
    message(STATUS "Looking for Bullet - found (single precision)")
  endif()
endif()

# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")
file(GLOB detail_hdrs "detail/*.hpp")
file(GLOB detail_srcs "detail/*.cpp")

# Set local target name
set(target_name ${PROJECT_NAME}-collision-bullet)
set(component_name collision-bullet)

# Add backend sources to core and retain the legacy target for rebuilt consumers.
dart_add_core_headers(${hdrs} ${detail_hdrs})
dart_add_core_sources(${srcs} ${detail_srcs})
set(bullet_target Bullet)
get_target_property(bullet_alias Bullet ALIASED_TARGET)
if(bullet_alias)
  get_target_property(bullet_alias_global Bullet ALIAS_GLOBAL)
  if(NOT bullet_alias_global)
    set(bullet_target ${bullet_alias})
  endif()
endif()
get_property(bullet_imported_targets DIRECTORY PROPERTY IMPORTED_TARGETS)
if(bullet_target IN_LIST bullet_imported_targets)
  set_property(TARGET ${bullet_target} PROPERTY IMPORTED_GLOBAL TRUE)
endif()
# Defer the installed name so CMake does not export a parent's alias target.
dart_property_add(DART_CORE_LINK_LIBRARIES
  $<BUILD_INTERFACE:${bullet_target}> $<INSTALL_INTERFACE:$<1:Bullet>>
)
dart_property_add(DART_CORE_DEPENDENCY_PACKAGES Bullet)
if(BT_USE_DOUBLE_PRECISION)
  dart_property_add(DART_CORE_COMPILE_DEFINITIONS BT_USE_DOUBLE_PRECISION)
endif()
add_library(${target_name} INTERFACE ${hdrs})
target_link_libraries(${target_name} INTERFACE dart)
add_dependencies(${target_name} dart)

# Component
add_component(${PROJECT_NAME} ${component_name})
add_component_targets(${PROJECT_NAME} ${component_name} ${target_name})
add_component_dependencies(${PROJECT_NAME} ${component_name} dart)
add_component_dependency_packages(${PROJECT_NAME} ${component_name} Bullet)

# Generate header for this namespace
dart_get_filename_components(header_names "collision_bullet headers" ${hdrs})
dart_generate_include_header_file(
  "${CMAKE_CURRENT_BINARY_DIR}/bullet.hpp"
  "dart/collision/bullet/"
  ${header_names}
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/bullet.hpp
  DESTINATION include/dart/collision/bullet
  COMPONENT headers
)

dart_format_add(${hdrs} ${srcs} ${detail_hdrs} ${detail_srcs})
