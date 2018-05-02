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
find_package(EIGEN3 3.0.5 REQUIRED)
dart_check_required_package(EIGEN3 "eigen3")

# CCD
find_package(CCD 1.4.0 REQUIRED)
dart_check_required_package(CCD "libccd")

# FCL
find_package(FCL 0.2.9 REQUIRED)
dart_check_required_package(FCL "fcl")

# ASSIMP
find_package(ASSIMP 3.0.0 REQUIRED)
dart_check_required_package(ASSIMP "assimp")
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
set(DART_MIN_BOOST_VERSION 1.46.0 CACHE INTERNAL "Boost min version requirement" FORCE)
if(MSVC)
  add_definitions(-DBOOST_ALL_NO_LIB)
endif()
add_definitions(-DBOOST_TEST_DYN_LINK)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
if(MSVC)
  set(BOOST_REQUIRED_COMPONENTS system filesystem)
else()
  set(BOOST_REQUIRED_COMPONENTS regex system filesystem)
endif()
if(DART_VERBOSE)
  find_package(Boost ${DART_MIN_BOOST_VERSION} REQUIRED COMPONENTS ${BOOST_REQUIRED_COMPONENTS})
else()
  find_package(Boost ${DART_MIN_BOOST_VERSION} QUIET REQUIRED COMPONENTS ${BOOST_REQUIRED_COMPONENTS})
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
