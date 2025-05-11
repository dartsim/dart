# If you add a dependency, please add the corresponding rosdep key as a
# dependency in package.xml.

#======================================
# Required dependencies for DART core
#======================================
if(DART_VERBOSE)
  message(STATUS "")
  message(STATUS "[ Required dependencies for DART core ]")
endif()

# fmt
dart_find_package(fmt)
dart_check_required_package(fmt "libfmt")

# Eigen
dart_find_package(Eigen3)
dart_check_required_package(EIGEN3 "eigen3")

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
  if (MSVC)
    set(CMAKE_REQUIRED_FLAGS "-w")
  else()
    set(CMAKE_REQUIRED_FLAGS "-std=c++11 -w")
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

# octomap
dart_find_package(octomap)
if(OCTOMAP_FOUND OR octomap_FOUND)
  if(NOT DEFINED octomap_VERSION)
    set(HAVE_OCTOMAP FALSE CACHE BOOL "Check if octomap found." FORCE)
    message(WARNING "Looking for octomap - octomap_VERSION is not defined, "
        "please install octomap with version information"
    )
  else()
    set(HAVE_OCTOMAP TRUE CACHE BOOL "Check if octomap found." FORCE)
    if(DART_VERBOSE)
      message(STATUS "Looking for octomap - version ${octomap_VERSION} found")
    endif()
  endif()
else()
  set(HAVE_OCTOMAP FALSE CACHE BOOL "Check if octomap found." FORCE)
  message(WARNING "Looking for octomap - NOT found, to use VoxelGridShape, "
      "please install octomap"
  )
endif()

#=======================
# Optional dependencies
#=======================

if(DART_BUILD_PROFILE)
  if(DART_USE_SYSTEM_TRACY)
    find_package(Tracy CONFIG REQUIRED)
  else()
    include(FetchContent)
    FetchContent_Declare(tracy
      GIT_REPOSITORY https://github.com/wolfpld/tracy.git
      GIT_TAG v0.11.1
      GIT_SHALLOW TRUE
      GIT_PROGRESS TRUE
    )
    FetchContent_MakeAvailable(tracy)
    if(MSVC)
      target_compile_options(TracyClient PRIVATE /W0)
    else()
      target_compile_options(TracyClient PRIVATE -w)
    endif()
  endif()
endif()

find_package(Python3 COMPONENTS Interpreter Development)

option(DART_SKIP_spdlog "If ON, do not use spdlog even if it is found." OFF)
mark_as_advanced(DART_SKIP_spdlog)
dart_find_package(spdlog)

#--------------------
# Misc. dependencies
#--------------------

# Doxygen
find_package(Doxygen QUIET)
dart_check_optional_package(DOXYGEN "generating API documentation" "doxygen")

