# If you add a dependency, please add the corresponding rosdep key as a
# dependency in package.xml.

#=======================
# Required Dependencies
#=======================
if(DART_VERBOSE)
  message(STATUS "")
  message(STATUS "[ Mandatory dependencies for DART core ]")
endif()

# Eigen
dart_find_package(Eigen3)
dart_check_required_package(EIGEN3 "eigen3")

# CCD
dart_find_package(ccd)
dart_check_required_package(ccd "libccd")

# FCL
dart_find_package(fcl)
dart_check_required_package(fcl "fcl")

# ASSIMP
dart_find_package(assimp)
dart_check_required_package(assimp "assimp")
if(ASSIMP_FOUND)
  # Check for missing symbols in ASSIMP (see #451)
  include(CheckCXXSourceCompiles)
  set(CMAKE_REQUIRED_DEFINITIONS "")
  if (NOT ASSIMP_VERSION VERSION_LESS 3.3.0 AND NOT MSVC)
    set(CMAKE_REQUIRED_FLAGS "-std=c++11 -w")
  else()
    set(CMAKE_REQUIRED_FLAGS "-w")
  endif()
  set(CMAKE_REQUIRED_INCLUDES ${ASSIMP_INCLUDE_DIRS})
  set(CMAKE_REQUIRED_LIBRARIES ${ASSIMP_LIBRARIES})

  check_cxx_source_compiles(
  "
  #include <assimp/scene.h>
  int main()
  {
    aiScene* scene = new aiScene;
    delete scene;
    return 1;
  }
  "
  ASSIMP_AISCENE_CTOR_DTOR_DEFINED)

  if(NOT ASSIMP_AISCENE_CTOR_DTOR_DEFINED)
    if(DART_VERBOSE)
      message(WARNING "The installed version of ASSIMP (${ASSIMP_VERSION}) is "
                      "missing symbols for the constructor and/or destructor of "
                      "aiScene. DART will use its own implementations of these "
                      "functions. We recommend using a version of ASSIMP that "
                      "does not have this issue, once one becomes available.")
    endif()
  endif(NOT ASSIMP_AISCENE_CTOR_DTOR_DEFINED)

  check_cxx_source_compiles(
  "
  #include <assimp/material.h>
  int main()
  {
    aiMaterial* material = new aiMaterial;
    delete material;
    return 1;
  }
  "
  ASSIMP_AIMATERIAL_CTOR_DTOR_DEFINED)

  if(NOT ASSIMP_AIMATERIAL_CTOR_DTOR_DEFINED)
    if(DART_VERBOSE)
      message(WARNING "The installed version of ASSIMP (${ASSIMP_VERSION}) is "
                      "missing symbols for the constructor and/or destructor of "
                      "aiMaterial. DART will use its own implementations of "
                      "these functions. We recommend using a version of ASSIMP "
                      "that does not have this issue, once one becomes available.")
    endif()
  endif(NOT ASSIMP_AIMATERIAL_CTOR_DTOR_DEFINED)

  unset(CMAKE_REQUIRED_FLAGS)
  unset(CMAKE_REQUIRED_INCLUDES)
  unset(CMAKE_REQUIRED_LIBRARIES)
endif()

# Boost
dart_find_package(Boost)

# octomap
dart_find_package(octomap)
if(MSVC)
  if(OCTOMAP_FOUND OR octomap_FOUND)
    if(NOT DEFINED octomap_VERSION)
      set(DART_HAVE_OCTOMAP FALSE CACHE BOOL "Check if octomap found." FORCE)
      message(WARNING "Looking for octomap - octomap_VERSION is not defined, "
          "please install octomap with version information"
      )
    elseif(octomap_VERSION VERSION_LESS_EQUAL 1.9.6)
      set(DART_HAVE_OCTOMAP FALSE CACHE BOOL "Check if octomap found." FORCE)
      # Supporting Octomap on Windows is disabled for the following issue:
      # https://github.com/OctoMap/octomap/pull/213
      message(WARNING "Octomap ${octomap_VERSION} is found, but Octomap "
          "is not supported on Windows until "
          "'https://github.com/OctoMap/octomap/pull/213' "
          "is resolved."
      )
    else()
      set(DART_HAVE_OCTOMAP TRUE CACHE BOOL "Check if octomap found." FORCE)
      if(DART_VERBOSE)
        message(STATUS "Looking for octomap - version ${octomap_VERSION} found")
      endif()
    endif()
  else()
    set(DART_HAVE_OCTOMAP FALSE CACHE BOOL "Check if octomap found." FORCE)
    message(WARNING "Looking for octomap - NOT found, to use VoxelGridShape, "
        "please install octomap"
    )
  endif()
else()
  if(OCTOMAP_FOUND OR octomap_FOUND)
    if(NOT DEFINED octomap_VERSION)
      set(DART_HAVE_OCTOMAP FALSE CACHE BOOL "Check if octomap found." FORCE)
      message(WARNING "Looking for octomap - octomap_VERSION is not defined, "
          "please install octomap with version information"
      )
    else()
      set(DART_HAVE_OCTOMAP TRUE CACHE BOOL "Check if octomap found." FORCE)
      if(DART_VERBOSE)
        message(STATUS "Looking for octomap - version ${octomap_VERSION} found")
      endif()
    endif()
  else()
    set(DART_HAVE_OCTOMAP FALSE CACHE BOOL "Check if octomap found." FORCE)
    message(WARNING "Looking for octomap - NOT found, to use VoxelGridShape, "
        "please install octomap"
    )
  endif()
endif()

#=======================
# Optional Dependencies
#=======================

dart_find_package(spdlog)

# find_package(Threads)
dart_find_package(NLOPT)
dart_find_package(IPOPT)
dart_find_package(pagmo)
dart_find_package(ODE)

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

  set(DART_HAVE_BULLET TRUE CACHE BOOL "Check if BULLET found." FORCE)
else()
  message(WARNING "Looking for Bullet - NOT found, to use dart-collision-bullet, please install libbullet-dev")
  set(DART_HAVE_BULLET FALSE CACHE BOOL "Check if BULLET found." FORCE)
  return()
endif()

# Dependency checks
dart_find_package(lz4)  # needed for flann
dart_find_package(flann)

dart_find_package(tinyxml2)

# Test whether boost algorithm and lexical_cast headers are available.
include(CheckCXXSourceCompiles)
set(CMAKE_REQUIRED_FLAGS "-w")
set(CMAKE_REQUIRED_INCLUDES "${Boost_INCLUDE_DIRS}")
check_cxx_source_compiles(
  "
  #include <boost/algorithm/string.hpp>
  #include <boost/lexical_cast.hpp>
  int main()
  {
    return 0;
  }
  "
  HAS_BOOST_ALGORITHM_LEXICAL_CAST
)
if(HAS_BOOST_ALGORITHM_LEXICAL_CAST)
  if(DART_VERBOSE)
    message(STATUS "Looking for Boost algorithm and lexical_cast headers - found")
  endif()
else()
  message(WARNING "Looking for Boost algorithm and lexical_cast headers - NOT found, to use dart-io, please install missing boost headers")
  return()
endif()

dart_find_package(urdfdom)
if(urdfdom_FOUND)
  if(DART_VERBOSE)
    message(STATUS "Looking for urdfdom - ${urdfdom_headers_VERSION} found")
  endif()
else()
  message(WARNING "Looking for urdfdom - NOT found, to use dart-io-urdf, please install liburdfdom-dev")
  return()
endif()
if(MSVC)
  set(urdfdom_LIBRARIES optimized urdfdom_sensor      debug urdfdom_sensord
                        optimized urdfdom_model_state debug urdfdom_model_stated
                        optimized urdfdom_model       debug urdfdom_modeld
                        optimized urdfdom_world       debug urdfdom_worldd
                        optimized console_bridge      debug console_bridged)
endif()

dart_find_package(OpenGL)
dart_find_package(GLUT)

# Minimum required OSG version
set(min_osg_version 3.2.3)
# OpenSceneGraph 3.6.5 or less are not compatible with macOS 10.15 (Catalina) or greater
# See:
#   - https://github.com/openscenegraph/OpenSceneGraph/issues/926
#   - https://github.com/dartsim/dart/issues/1439
if(APPLE)
  if(NOT ${CMAKE_SYSTEM_VERSION} VERSION_LESS 19)
    set(min_osg_version 3.7.0)
  endif()
endif()
dart_find_package(OpenSceneGraph)

#--------------------
# Misc. dependencies
#--------------------

# Doxygen
find_package(Doxygen QUIET)
dart_check_optional_package(DOXYGEN "generating API documentation" "doxygen")
