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
  # CMake's FindBullet ignores COMPONENTS and always returns all four
  # libraries. DART's collision backend uses only Collision and LinearMath,
  # so narrow BULLET_LIBRARIES to those two after discovery.
  find_package(Bullet MODULE QUIET)
  if(BULLET_FOUND OR Bullet_FOUND)
    set(_bullet_narrowed_libraries "")
    foreach(_bullet_lib_var BULLET_COLLISION_LIBRARY BULLET_MATH_LIBRARY)
      if(${_bullet_lib_var}_DEBUG)
        list(
          APPEND _bullet_narrowed_libraries
          optimized
          ${${_bullet_lib_var}}
          debug
          ${${_bullet_lib_var}_DEBUG}
        )
      elseif(${_bullet_lib_var})
        list(APPEND _bullet_narrowed_libraries ${${_bullet_lib_var}})
      endif()
    endforeach()
    if(_bullet_narrowed_libraries)
      set(BULLET_LIBRARIES "${_bullet_narrowed_libraries}")
    endif()
    unset(_bullet_narrowed_libraries)
    unset(_bullet_lib_var)
  endif()
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
  add_library(Bullet INTERFACE IMPORTED)
  target_include_directories(Bullet INTERFACE ${BULLET_INCLUDE_DIRS})
  target_link_libraries(Bullet INTERFACE ${BULLET_LIBRARIES})
endif()
