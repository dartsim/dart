# Copyright (c) 2011-2025, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

# Bullet. Force MODULE mode to use the FindBullet.cmake file distributed with
# CMake. Otherwise, we may end up using the BulletConfig.cmake file distributed
# with Bullet, which uses relative paths and may break transitive dependencies.
include("${CMAKE_CURRENT_LIST_DIR}/DARTFindPackageVersion.cmake")

if(NOT DART_USE_SYSTEM_BULLET)
  if(TARGET BulletCollision AND TARGET LinearMath)
    if(DEFINED DART_BULLET_SOURCE_DIR)
      set(_bullet_include_dir "${DART_BULLET_SOURCE_DIR}/src")
    elseif(DEFINED BULLET_PHYSICS_SOURCE_DIR)
      set(_bullet_include_dir "${BULLET_PHYSICS_SOURCE_DIR}/src")
    endif()
    if(_bullet_include_dir)
      set(BULLET_INCLUDE_DIRS "${_bullet_include_dir}")
    endif()
    set(BULLET_LIBRARIES BulletCollision LinearMath)
    set(BULLET_FOUND TRUE)
    set(Bullet_FOUND TRUE)
    unset(_bullet_include_dir)
  endif()
endif()

set(_dart_bullet_system FALSE)
if(DART_USE_SYSTEM_BULLET OR NOT BULLET_FOUND)
  find_package(Bullet COMPONENTS BulletMath BulletCollision MODULE QUIET)
  set(_dart_bullet_system TRUE)
endif()

if(BULLET_FOUND OR Bullet_FOUND)
  unset(BULLET_VERSION)
  dart_read_header_version(_bullet_header_version
    LinearMath/btScalar.h BT_BULLET_VERSION ${BULLET_INCLUDE_DIRS}
  )
  if(_bullet_header_version MATCHES "^[0-9]+$")
    math(EXPR _bullet_major "${_bullet_header_version} / 100")
    math(EXPR _bullet_minor "${_bullet_header_version} % 100")
    set(BULLET_VERSION "${_bullet_major}.${_bullet_minor}")
  endif()
  # Upstream Bullet 3.06 and 3.07 still report 305 (Ubuntu 22.04 ships 3.06).
  dart_check_package_version(Bullet 3.05)
  unset(_bullet_header_version)
  unset(_bullet_major)
  unset(_bullet_minor)
endif()

if((BULLET_FOUND OR Bullet_FOUND) AND NOT TARGET Bullet)
  # Single-precision Bullet's fixed GJK tolerances drop or tilt contacts of
  # centimeter-scale shapes (gz-physics#1117), so prefer the double-precision
  # libraries that Ubuntu and conda-forge install beside the single-precision
  # ones. Float heightfields need Bullet 3.17 in double precision. Leave
  # FindBullet's variables alone: consumers may run it for their own targets.
  # An installed DART sets DART_BULLET_REQUIRE_FLOAT64 to the selection it was
  # built with, because a consumer must not load both Bullet precisions.
  if(DEFINED DART_BULLET_REQUIRE_FLOAT64)
    set(_dart_bullet_want_float64 "${DART_BULLET_REQUIRE_FLOAT64}")
  elseif(DART_BULLET_FLOAT64 AND BULLET_VERSION VERSION_GREATER_EQUAL 3.17)
    set(_dart_bullet_want_float64 TRUE)
  else()
    set(_dart_bullet_want_float64 FALSE)
  endif()
  set(_dart_bullet_libraries ${BULLET_LIBRARIES})
  set(_dart_bullet_float64 FALSE)
  if(
    _dart_bullet_system
    AND BULLET_COLLISION_LIBRARY
    AND _dart_bullet_want_float64
  )
    get_filename_component(
      _dart_bullet_dir
      "${BULLET_COLLISION_LIBRARY}"
      DIRECTORY
    )
    find_library(
      DART_BULLET_COLLISION_FLOAT64_LIBRARY
      BulletCollision-float64
      HINTS "${_dart_bullet_dir}"
      NO_DEFAULT_PATH
    )
    find_library(
      DART_BULLET_MATH_FLOAT64_LIBRARY
      LinearMath-float64
      HINTS "${_dart_bullet_dir}"
      NO_DEFAULT_PATH
    )
    mark_as_advanced(
      DART_BULLET_COLLISION_FLOAT64_LIBRARY
      DART_BULLET_MATH_FLOAT64_LIBRARY
    )
    if(
      DART_BULLET_COLLISION_FLOAT64_LIBRARY
      AND DART_BULLET_MATH_FLOAT64_LIBRARY
    )
      set(
        _dart_bullet_libraries
        "${DART_BULLET_COLLISION_FLOAT64_LIBRARY}"
        "${DART_BULLET_MATH_FLOAT64_LIBRARY}"
      )
      set(_dart_bullet_float64 TRUE)
    endif()
    unset(_dart_bullet_dir)
  endif()

  if(DART_BULLET_REQUIRE_FLOAT64 AND NOT _dart_bullet_float64)
    message(
      WARNING
      "DART was built with Bullet's -float64 libraries, which were not found "
      "beside '${BULLET_COLLISION_LIBRARY}'."
    )
    set(BULLET_FOUND FALSE)
    set(Bullet_FOUND FALSE)
  else()
    add_library(Bullet INTERFACE IMPORTED)
    target_include_directories(Bullet INTERFACE ${BULLET_INCLUDE_DIRS})
    target_link_libraries(Bullet INTERFACE ${_dart_bullet_libraries})
  endif()
  if(NOT DEFINED DART_BULLET_REQUIRE_FLOAT64)
    set(
      DART_BULLET_USES_FLOAT64
      ${_dart_bullet_float64}
      CACHE INTERNAL
      "Whether DART links Bullet's -float64 libraries"
    )
  endif()
  unset(_dart_bullet_libraries)
  unset(_dart_bullet_float64)
  unset(_dart_bullet_want_float64)
endif()
unset(_dart_bullet_system)
