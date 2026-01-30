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

# On Windows/MSVC, we need to handle Debug/Release library configuration carefully
# to avoid runtime library mismatches (MDd vs MD)
if(MSVC)
  set(_DART_MULTI_CONFIG FALSE)
  if(CMAKE_CONFIGURATION_TYPES)
    set(_DART_MULTI_CONFIG TRUE)
  endif()
  # Store original CMAKE_CONFIGURATION_TYPES and CMAKE_BUILD_TYPE
  set(_BULLET_ORIG_CMAKE_FIND_LIBRARY_SUFFIXES ${CMAKE_FIND_LIBRARY_SUFFIXES})

  # For multi-config generators (like Visual Studio), set library suffixes appropriately
  # FindBullet.cmake looks for libraries with _Debug/_d suffix for debug builds
  if(NOT _DART_MULTI_CONFIG AND CMAKE_BUILD_TYPE MATCHES "Debug")
    # Prefer Debug versions first, then fall back to Release
    set(CMAKE_FIND_LIBRARY_SUFFIXES "_Debug.lib" "_d.lib" ".lib")
  else()
    # For Release, RelWithDebInfo, MinSizeRel
    set(CMAKE_FIND_LIBRARY_SUFFIXES ".lib" "_Release.lib")
  endif()
endif()

find_package(Bullet COMPONENTS BulletMath BulletCollision MODULE QUIET)

# On Windows/MSVC, validate that library configuration matches build configuration
if(MSVC AND BULLET_FOUND)
  # Restore original suffixes first
  if(DEFINED _BULLET_ORIG_CMAKE_FIND_LIBRARY_SUFFIXES)
    set(CMAKE_FIND_LIBRARY_SUFFIXES ${_BULLET_ORIG_CMAKE_FIND_LIBRARY_SUFFIXES})
    unset(_BULLET_ORIG_CMAKE_FIND_LIBRARY_SUFFIXES)
  endif()

  # Check if we have a configuration mismatch
  set(_BULLET_HAS_DEBUG_LIBS FALSE)
  set(_BULLET_HAS_RELEASE_LIBS FALSE)
  set(_BULLET_CONFIG_MISMATCH FALSE)

  foreach(_bullet_lib ${BULLET_LIBRARIES})
    if(_bullet_lib MATCHES ".*_d\\.lib$" OR _bullet_lib MATCHES ".*_Debug\\.lib$")
      set(_BULLET_HAS_DEBUG_LIBS TRUE)
    elseif(_bullet_lib MATCHES ".*\\.lib$")
      set(_BULLET_HAS_RELEASE_LIBS TRUE)
    endif()
  endforeach()

  # Check for mismatch and disable Bullet if found
  if(NOT _DART_MULTI_CONFIG AND CMAKE_BUILD_TYPE MATCHES "Debug" AND _BULLET_HAS_RELEASE_LIBS AND NOT _BULLET_HAS_DEBUG_LIBS)
    set(_BULLET_CONFIG_MISMATCH TRUE)
    message(
      WARNING
      "Bullet libraries appear to be Release builds, but CMAKE_BUILD_TYPE is Debug. "
      "This will cause LNK2038 runtime library mismatch errors on Windows/MSVC. "
      "Disabling Bullet support to avoid link errors."
      "\n"
      "To enable Bullet support:"
      "\n"
      "  1. Build Bullet in Debug mode (with -DCMAKE_BUILD_TYPE=Debug)\n"
      "  2. Or change DART build to Release mode: -DCMAKE_BUILD_TYPE=Release\n"
      "  3. Or use RelWithDebInfo: -DCMAKE_BUILD_TYPE=RelWithDebInfo"
    )
    set(BULLET_FOUND FALSE)
  elseif(NOT _DART_MULTI_CONFIG AND NOT CMAKE_BUILD_TYPE MATCHES "Debug" AND _BULLET_HAS_DEBUG_LIBS AND NOT _BULLET_HAS_RELEASE_LIBS)
    set(_BULLET_CONFIG_MISMATCH TRUE)
    message(
      WARNING
      "Bullet libraries appear to be Debug builds, but CMAKE_BUILD_TYPE is ${CMAKE_BUILD_TYPE}. "
      "This will cause runtime library mismatch errors on Windows/MSVC. "
      "Disabling Bullet support to avoid link errors."
      "\n"
      "To enable Bullet support:"
      "\n"
      "  1. Build Bullet in Release mode (with -DCMAKE_BUILD_TYPE=Release)\n"
      "  2. Or change DART build to Debug mode: -DCMAKE_BUILD_TYPE=Debug"
    )
    set(BULLET_FOUND FALSE)
  endif()

  unset(_BULLET_HAS_DEBUG_LIBS)
  unset(_BULLET_HAS_RELEASE_LIBS)
  unset(_BULLET_CONFIG_MISMATCH)
  unset(_DART_MULTI_CONFIG)
elseif(DEFINED _BULLET_ORIG_CMAKE_FIND_LIBRARY_SUFFIXES)
  # Restore original suffixes if not done above
  set(CMAKE_FIND_LIBRARY_SUFFIXES ${_BULLET_ORIG_CMAKE_FIND_LIBRARY_SUFFIXES})
  unset(_BULLET_ORIG_CMAKE_FIND_LIBRARY_SUFFIXES)
endif()

if((BULLET_FOUND OR Bullet_FOUND) AND NOT TARGET Bullet)
  add_library(Bullet INTERFACE IMPORTED)
  target_include_directories(Bullet INTERFACE ${BULLET_INCLUDE_DIRS})
  target_link_libraries(Bullet INTERFACE ${BULLET_LIBRARIES})
endif()

if(DEFINED _DART_MULTI_CONFIG)
  unset(_DART_MULTI_CONFIG)
endif()
