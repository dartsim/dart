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
    if(NOT TARGET Bullet)
      add_library(Bullet INTERFACE IMPORTED)
      if(BULLET_INCLUDE_DIRS)
        target_include_directories(Bullet INTERFACE ${BULLET_INCLUDE_DIRS})
      endif()
      target_link_libraries(Bullet INTERFACE ${BULLET_LIBRARIES})
    endif()
    unset(_bullet_include_dir)
    return()
  endif()
endif()

find_package(Bullet COMPONENTS BulletMath BulletCollision MODULE QUIET)

if((BULLET_FOUND OR Bullet_FOUND) AND NOT TARGET Bullet)
  add_library(Bullet INTERFACE IMPORTED)
  target_include_directories(Bullet INTERFACE ${BULLET_INCLUDE_DIRS})
  target_link_libraries(Bullet INTERFACE ${BULLET_LIBRARIES})
endif()
