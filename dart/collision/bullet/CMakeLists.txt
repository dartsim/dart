# Dependency checks
file(GLOB facade_hdrs "*.hpp")
file(GLOB compat_hdrs "compat/*.hpp")
dart_format_add(${facade_hdrs} ${compat_hdrs})

if(NOT _dart_build_collision_references)
  set(DART_HAVE_BULLET FALSE CACHE BOOL "Check if BULLET found." FORCE)
  unset(BT_USE_DOUBLE_PRECISION CACHE)
  unset(BT_USE_DOUBLE_PRECISION)
  install(
    FILES ${compat_hdrs}
    DESTINATION include/dart/collision/bullet
    COMPONENT headers
  )
  return()
endif()

message(STATUS "Configuring dart/collision/bullet reference component")

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
      "Collision reference tests or benchmarks are enabled but Bullet was not "
      "found. Please install libbullet-dev (>= 3.25) or disable the "
      "collision reference test/benchmark options."
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

# Search all reference-engine header and source files. The top-level headers in
# this directory are native-backed compatibility facades; old Bullet code lives
# under reference/ and is built only by the explicit reference target.
file(GLOB hdrs "reference/*.hpp")
file(GLOB srcs "reference/*.cpp")
file(GLOB detail_hdrs "reference/detail/*.hpp")
file(GLOB detail_srcs "reference/detail/*.cpp")

set(_dart_bullet_link_items ${BULLET_LIBRARIES})
if(TARGET Bullet::Bullet)
  list(APPEND _dart_bullet_link_items Bullet::Bullet)
endif()
if(NOT _dart_bullet_link_items)
  set(_bullet_candidate_libs
    ${BULLET_COLLISION_LIBRARY}
    ${BULLET_DYNAMICS_LIBRARY}
    ${BULLET_SOFTBODY_LIBRARY}
    ${BULLET_MATH_LIBRARY}
  )
  foreach(_bullet_candidate IN LISTS _bullet_candidate_libs)
    if(_bullet_candidate)
      list(APPEND _dart_bullet_link_items ${_bullet_candidate})
    endif()
  endforeach()
endif()
list(REMOVE_DUPLICATES _dart_bullet_link_items)
if(NOT _dart_bullet_link_items)
  message(
    FATAL_ERROR
    "Bullet was found but did not report any linkable libraries."
  )
endif()
set(_dart_bullet_include_dirs ${BULLET_INCLUDE_DIRS})

set(target_name ${PROJECT_NAME}-collision-reference-bullet)
dart_add_library(${target_name} ${srcs} ${detail_srcs})
target_include_directories(${target_name}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)
if(_dart_bullet_include_dirs)
  target_include_directories(${target_name} SYSTEM PUBLIC ${_dart_bullet_include_dirs})
endif()
target_link_libraries(${target_name}
  PUBLIC
    dart
    ${_dart_bullet_link_items}
)
if(TARGET ${PROJECT_NAME}-collision-native)
  target_link_libraries(${target_name} PUBLIC ${PROJECT_NAME}-collision-native)
  target_compile_definitions(${target_name} PRIVATE DART_BULLET_HAS_DART=1)
endif()
target_compile_definitions(${target_name}
  PRIVATE
    DART_BUILDING_COLLISION_BULLET
)
if(BT_USE_DOUBLE_PRECISION)
  target_compile_definitions(${target_name} PUBLIC BT_USE_DOUBLE_PRECISION)
endif()

add_component_targets(${PROJECT_NAME} collision-reference-bullet ${target_name})
add_component_dependencies(${PROJECT_NAME} collision-reference-bullet dart)
add_component_dependency_packages(${PROJECT_NAME} collision-reference-bullet Bullet)

# Generate reference-only headers for this namespace
set(header_names)
foreach(header IN LISTS hdrs)
  file(RELATIVE_PATH header_name "${CMAKE_CURRENT_SOURCE_DIR}" "${header}")
  list(APPEND header_names "${header_name}")
endforeach()
dart_generate_component_headers(
  COMPONENT_NAME bullet
  TARGET_DIR "dart/collision/bullet/"
  OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}"
  HEADERS ${header_names}
  SOURCE_HEADERS ${hdrs}
)

# Install native-backed compatibility headers on the public legacy path. The
# Bullet implementation target remains explicit reference-only test/benchmark
# code.
install(
  FILES ${compat_hdrs}
  DESTINATION include/dart/collision/bullet
  COMPONENT headers
)

dart_format_add(${hdrs} ${srcs} ${detail_hdrs} ${detail_srcs})
