# Copyright (c) 2011-2021, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/master/LICENSE
#
# This file is provided under the "BSD-style" License

# Bullet. Force MODULE mode to use the FindBullet.cmake file distributed with
# CMake. Otherwise, we may end up using the BulletConfig.cmake file distributed
# with Bullet, which uses relative paths and may break transitive dependencies.
find_package(Bullet COMPONENTS BulletMath BulletCollision MODULE QUIET)

if((BULLET_FOUND OR Bullet_FOUND) AND NOT TARGET Bullet)
  if(WIN32 AND "optimized" IN_LIST BULLET_LIBRARIES
           AND     "debug" IN_LIST BULLET_LIBRARIES)
    cmake_parse_arguments(BULLET_INTERFACE_LIBRARIES "" "" "debug;optimized"
        ${BULLET_LIBRARIES})
    set(BULLET_INTERFACE_LIBRARIES
        $<$<CONFIG:Debug>:${BULLET_INTERFACE_LIBRARIES_debug}>
        $<$<NOT:$<CONFIG:Debug>>:${BULLET_INTERFACE_LIBRARIES_optimized}>)
  else()
    set(BULLET_INTERFACE_LIBRARIES ${BULLET_LIBRARIES})
  endif()
  add_library(Bullet INTERFACE IMPORTED)
  set_target_properties(Bullet PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${BULLET_INCLUDE_DIRS}"
    INTERFACE_LINK_LIBRARIES "${BULLET_INTERFACE_LIBRARIES}"
  )
endif()
