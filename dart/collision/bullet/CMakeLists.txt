# Dependency checks

# Bullet. Force MODULE mode to use the FindBullet.cmake file distributed with
# CMake. Otherwise, we may end up using the BulletConfig.cmake file distributed
# with Bullet, which uses relative paths and may break transitive dependencies.
dart_find_package(Bullet)
if(BULLET_FOUND)
  # Test whether Bullet was built with double precision. If so, we need to
  # define the BT_USE_DOUBLE_PRECISION pre-processor directive before including
  # any Bullet headers. This is a workaround for the fact that Bullet does not
  # add the definition to BULLET_DEFINITIONS or generate a #cmakedefine header.
  include(CheckCXXSourceCompiles)
  set(CMAKE_REQUIRED_FLAGS "-w")
  set(CMAKE_REQUIRED_DEFINITIONS "-DBT_USE_DOUBLE_PRECISION")
  set(CMAKE_REQUIRED_INCLUDES "${BULLET_INCLUDE_DIRS}")
  set(CMAKE_REQUIRED_LIBRARIES "${BULLET_LIBRARIES}")
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

  if(DART_VERBOSE)
    if(BT_USE_DOUBLE_PRECISION)
      message(STATUS "Looking for Bullet - found (double precision)")
    else()
      message(STATUS "Looking for Bullet - found (single precision)")
    endif()
  endif()

  set(HAVE_BULLET TRUE CACHE BOOL "Check if BULLET found." FORCE)
else()
  message(WARNING "Looking for Bullet - NOT found, to use dart-collision-bullet, please install libbullet-dev")
  set(HAVE_BULLET FALSE CACHE BOOL "Check if BULLET found." FORCE)
  return()
endif()

# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")
file(GLOB detail_hdrs "detail/*.hpp")
file(GLOB detail_srcs "detail/*.cpp")

# Set local target name
set(target_name ${PROJECT_NAME}-collision-bullet)
set(component_name collision-bullet)

# Add target
dart_add_library(${target_name} ${hdrs} ${srcs} ${detail_hdrs} ${detail_srcs})
target_link_libraries(${target_name} PUBLIC dart Bullet)

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
