# If you add a dependency, please add the corresponding rosdep key as a
# dependency in package.xml.

#======================================
# Mandatory dependencies for DART core
#======================================
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
    set(CMAKE_REQUIRED_FLAGS "-std=c++11")
  else()
    set(CMAKE_REQUIRED_FLAGS "")
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
if (octomap_FOUND AND NOT MSVC)
  if (MSVC)
    # Supporting Octomap on Windows is disabled for the following issue:
    # https://github.com/OctoMap/octomap/pull/213
    message(WARNING "Octomap ${octomap_VERSION} is found, but Octomap "
        "is not supported on Windows until "
        "'https://github.com/OctoMap/octomap/pull/213' "
        "is resolved.")
    set(HAVE_OCTOMAP FALSE CACHE BOOL "Check if octomap found." FORCE)
  elseif (NOT octomap_VERSION VERSION_LESS 1.9.0)
    message(WARNING "Octomap ${octomap_VERSION} is found, but Octomap 1.9.0 or "
        "greater is not supported yet. Please see "
        "'https://github.com/dartsim/dart/issues/1078' for the details")
    set(HAVE_OCTOMAP FALSE CACHE BOOL "Check if octomap found." FORCE)
  else()
    set(HAVE_OCTOMAP TRUE CACHE BOOL "Check if octomap found." FORCE)
    if(DART_VERBOSE)
      message(STATUS "Looking for octomap - version ${octomap_VERSION} found")
    endif()
  endif()
else()
  set(HAVE_OCTOMAP FALSE CACHE BOOL "Check if octomap found." FORCE)
  message(STATUS "Looking for octomap - NOT found, to use VoxelGridShape, "
      "please install octomap")
endif()

#--------------------
# Misc. dependencies
#--------------------

# Perl modules
find_package(PerlModules COMPONENTS Regexp::Common Getopt::ArgvFile Getopt::Long Term::ANSIColor QUIET)
if(DART_VERBOSE)
  if("${PERLMODULES_FOUND}" STREQUAL "TRUE")
    message(STATUS "Looking for PerlModules - found")
  else()
    message(STATUS "Looking for PerlModules - NOT found, to colorize gcc messages, please install Regexp::Common Getopt::ArgvFile Getopt::Long Term::ANSIColor (http://www.cpan.org/modules/INSTALL.html)")
  endif()
endif()

# Doxygen
find_package(Doxygen QUIET)
dart_check_optional_package(DOXYGEN "generating API documentation" "doxygen")
