# Dependency checks
if(NOT DART_BUILD_COLLISION_BULLET)
  set(DART_HAVE_BULLET FALSE CACHE BOOL "Check if BULLET found." FORCE)
  unset(BT_USE_DOUBLE_PRECISION CACHE)
  unset(BT_USE_DOUBLE_PRECISION)
  return()
endif()

message(STATUS "Configuring dart/collision/bullet (DART_BUILD_COLLISION_BULLET=${DART_BUILD_COLLISION_BULLET})")

# Skip find_package when Bullet was already provided via FetchContent
# (BULLET_FOUND and BULLET_LIBRARIES already set in dart_find_dependencies.cmake)
if(NOT BULLET_FOUND)
  # Bullet. Force MODULE mode to use the FindBullet.cmake file distributed with
  # CMake. Otherwise, we may end up using the BulletConfig.cmake file distributed
  # with Bullet, which uses relative paths and may break transitive dependencies.
  dart_find_package(Bullet)
  if(NOT BULLET_FOUND)
    message(
      FATAL_ERROR
      "DART_BUILD_COLLISION_BULLET is ON but Bullet was not found. "
      "Please install libbullet-dev (>= 3.25) or disable "
      "DART_BUILD_COLLISION_BULLET."
    )
  endif()
endif()

# Test whether Bullet was built with double precision. If so, we need to
# define the BT_USE_DOUBLE_PRECISION pre-processor directive before including
# any Bullet headers. This is a workaround for the fact that Bullet does not
# add the definition to BULLET_DEFINITIONS or generate a #cmakedefine header.
#
# Skip the probe if BT_USE_DOUBLE_PRECISION is already set (e.g., from
# FetchContent in dart_find_dependencies.cmake where we know the precision).
if(NOT DEFINED BT_USE_DOUBLE_PRECISION)
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
endif()

if(DART_VERBOSE)
  if(BT_USE_DOUBLE_PRECISION)
    message(STATUS "Looking for Bullet - found (double precision)")
  else()
    message(STATUS "Looking for Bullet - found (single precision)")
  endif()
endif()

set(DART_HAVE_BULLET TRUE CACHE BOOL "Check if BULLET found." FORCE)

# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")
file(GLOB detail_hdrs "detail/*.hpp")
file(GLOB detail_srcs "detail/*.cpp")

dart_add_core_headers(${hdrs} ${detail_hdrs})
dart_add_core_sources(${srcs} ${detail_srcs})
set(_dart_bullet_link_items ${BULLET_LIBRARIES})
if(TARGET Bullet::Bullet)
  list(APPEND _dart_bullet_link_items Bullet::Bullet)
endif()
if(NOT _dart_bullet_link_items)
  set(_bullet_candidate_libs ${BULLET_COLLISION_LIBRARY} ${BULLET_DYNAMICS_LIBRARY} ${BULLET_SOFTBODY_LIBRARY} ${BULLET_MATH_LIBRARY})
  foreach(_bullet_candidate IN LISTS _bullet_candidate_libs)
    if(_bullet_candidate)
      list(APPEND _dart_bullet_link_items ${_bullet_candidate})
    endif()
  endforeach()
endif()
list(REMOVE_DUPLICATES _dart_bullet_link_items)
if(NOT _dart_bullet_link_items)
  message(FATAL_ERROR "Bullet was found but did not report any linkable libraries.")
endif()
set(_dart_bullet_include_dirs ${BULLET_INCLUDE_DIRS})
if(_dart_bullet_include_dirs)
  set_property(GLOBAL APPEND PROPERTY DART_PUBLIC_SYSTEM_INCLUDE_DIRS ${_dart_bullet_include_dirs})
endif()
set_property(GLOBAL APPEND PROPERTY DART_EXTRA_LINK_LIBS ${_dart_bullet_link_items})
dart_pkgconfig_append_libraries(${_dart_bullet_link_items})

# Generate header for this namespace
dart_get_filename_components(header_names "collision_bullet headers" ${hdrs})
dart_generate_component_headers(
  COMPONENT_NAME bullet
  TARGET_DIR "dart/collision/bullet/"
  OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}"
  HEADERS ${header_names}
  SOURCE_HEADERS ${hdrs}
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/All.hpp ${CMAKE_CURRENT_BINARY_DIR}/bullet.hpp
  DESTINATION include/dart/collision/bullet
  COMPONENT headers
)

install(FILES ${detail_hdrs} DESTINATION include/dart/collision/bullet/detail COMPONENT headers)

dart_install_compat_headers(COMPAT_HEADERS ${DART_GENERATED_COMPAT_HEADERS} DESTINATION_PREFIX include/dart/collision/bullet)

dart_format_add(${hdrs} ${srcs} ${detail_hdrs} ${detail_srcs})
