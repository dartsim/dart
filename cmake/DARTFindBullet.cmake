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

if(DART_USE_SYSTEM_BULLET OR NOT BULLET_FOUND)
  find_package(Bullet COMPONENTS BulletMath BulletCollision MODULE QUIET)
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
  # Bundled Bullet supplies targets; only FindBullet's library files have
  # -float64 siblings.
  set(_dart_bullet_files TRUE)
  foreach(_dart_bullet_item IN LISTS BULLET_LIBRARIES)
    if(TARGET "${_dart_bullet_item}")
      set(_dart_bullet_files FALSE)
    endif()
  endforeach()
  set(_dart_bullet_libraries ${BULLET_LIBRARIES})
  set(_dart_bullet_float64 FALSE)
  if(
    _dart_bullet_want_float64
    AND _dart_bullet_files
    AND BULLET_COLLISION_LIBRARY
  )
    get_filename_component(
      _dart_bullet_dir
      "${BULLET_COLLISION_LIBRARY}"
      DIRECTORY
    )
    find_library(
      _dart_bullet_collision_float64
      BulletCollision-float64
      HINTS "${_dart_bullet_dir}"
      NO_DEFAULT_PATH
      NO_CACHE
    )
    find_library(
      _dart_bullet_math_float64
      LinearMath-float64
      HINTS "${_dart_bullet_dir}"
      NO_DEFAULT_PATH
      NO_CACHE
    )
    if(_dart_bullet_collision_float64 AND _dart_bullet_math_float64)
      set(
        _dart_bullet_libraries
        "${_dart_bullet_collision_float64}"
        "${_dart_bullet_math_float64}"
      )
      # Keep the dynamics and soft-body libraries that FindBullet also links.
      foreach(_dart_bullet_name BulletDynamics BulletSoftBody)
        find_library(
          _dart_bullet_extra_float64
          ${_dart_bullet_name}-float64
          HINTS "${_dart_bullet_dir}"
          NO_DEFAULT_PATH
          NO_CACHE
        )
        if(_dart_bullet_extra_float64)
          list(APPEND _dart_bullet_libraries "${_dart_bullet_extra_float64}")
        endif()
        unset(_dart_bullet_extra_float64)
      endforeach()
      set(_dart_bullet_float64 TRUE)
    endif()
    unset(_dart_bullet_dir)
    unset(_dart_bullet_name)
    unset(_dart_bullet_collision_float64)
    unset(_dart_bullet_math_float64)
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
  unset(_dart_bullet_want_float64)
  unset(_dart_bullet_files)
  unset(_dart_bullet_item)
  unset(_dart_bullet_libraries)
  unset(_dart_bullet_float64)
elseif(TARGET Bullet)
  # A parent project or an earlier include created the Bullet target. Record
  # its variant when building DART; an installed DART links it, so reject one
  # that links the other variant instead of loading both precisions.
  set(_dart_bullet_target Bullet)
  get_target_property(_dart_bullet_alias Bullet ALIASED_TARGET)
  if(_dart_bullet_alias)
    set(_dart_bullet_target "${_dart_bullet_alias}")
  endif()
  get_target_property(
    _dart_bullet_links
    "${_dart_bullet_target}"
    INTERFACE_LINK_LIBRARIES
  )
  # Classify by library files, including those of linked imported targets.
  set(_dart_bullet_files)
  foreach(_dart_bullet_item IN LISTS _dart_bullet_links)
    if(NOT TARGET "${_dart_bullet_item}")
      list(APPEND _dart_bullet_files "${_dart_bullet_item}")
      continue()
    endif()
    foreach(
      _dart_bullet_property
      IMPORTED_LOCATION
      IMPORTED_LOCATION_RELEASE
      IMPORTED_LOCATION_RELWITHDEBINFO
      IMPORTED_LOCATION_NOCONFIG
      IMPORTED_IMPLIB
      IMPORTED_IMPLIB_RELEASE
    )
      get_target_property(
        _dart_bullet_location
        "${_dart_bullet_item}"
        ${_dart_bullet_property}
      )
      if(_dart_bullet_location)
        list(APPEND _dart_bullet_files "${_dart_bullet_location}")
      endif()
    endforeach()
  endforeach()
  set(_dart_bullet_float64 FALSE)
  if("${_dart_bullet_files}" MATCHES "-float64")
    set(_dart_bullet_float64 TRUE)
  endif()
  if(NOT DEFINED DART_BULLET_REQUIRE_FLOAT64)
    set(
      DART_BULLET_USES_FLOAT64
      ${_dart_bullet_float64}
      CACHE INTERNAL
      "Whether DART links Bullet's -float64 libraries"
    )
  elseif(
    _dart_bullet_files
    AND
      (
        DART_BULLET_REQUIRE_FLOAT64
        AND NOT _dart_bullet_float64
        OR NOT DART_BULLET_REQUIRE_FLOAT64
        AND _dart_bullet_float64
      )
  )
    message(
      WARNING
      "The existing Bullet target links a different Bullet precision than "
      "DART was built with."
    )
    set(BULLET_FOUND FALSE)
    set(Bullet_FOUND FALSE)
  endif()
  unset(_dart_bullet_target)
  unset(_dart_bullet_alias)
  unset(_dart_bullet_links)
  unset(_dart_bullet_files)
  unset(_dart_bullet_item)
  unset(_dart_bullet_property)
  unset(_dart_bullet_location)
  unset(_dart_bullet_float64)
endif()
