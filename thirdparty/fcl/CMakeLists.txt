cmake_minimum_required(VERSION 2.8)
project(fcl CXX C)

set(CMAKE_BUILD_TYPE Release)
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")

SET(FCL_VERSION "0") # they use SVN
SET(FCL_SOVERSION "80") # SVN revision number

SET(CPACK_DEBIAN_PACKAGE_DEPENDS "libboost-dev, libccd")
SET(CPACK_PACKAGE_VERSION_MAJOR ${FCL_VERSION})
SET(CPACK_PACKAGE_VERSION_MINOR ${FCL_SOVERSION})
SET(CPACK_PACKAGE_VERSION_PATCH "1") # maintained by Golems
SET(CPACK_PACKAGE_NAME "libfcl")
SET(CPACK_SYSTEM_NAME "amd64")

set(PKG_DESC "Fast Collision Library from WillowGarage")

include(FCLVersion)

# Whether to enable SSE
option(FCL_USE_SSE "Whether FCL should SSE instructions" ON)
set(FCL_HAVE_SSE 0)
if(FCL_USE_SSE)
  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    set(FCL_HAVE_SSE 0) #always disable, for now
    add_definitions(-march=native -std=c++0x)
  endif()
  # TODO: do something similar for other compilers
endif()

# Find Octomap (optional)
find_package(PkgConfig QUIET)
set(FCL_HAVE_OCTOMAP 0)
if(PKG_CONFIG_FOUND)
    pkg_check_modules(OCTOMAP QUIET octomap)
endif()
if(NOT OCTOMAP_FOUND)
    # if pkfconfig is not installed, then fall back on more fragile detection
    # of octomap
    find_path(OCTOMAP_INCLUDE_DIRS octomap.h
        PATH_SUFFIXES octomap)
    find_path(OCTOMAP_LIBRARY_DIRS
        ${CMAKE_SHARED_LIBRARY_PREFIX}octomap${CMAKE_SHARED_LIBRARY_SUFFIX})
    if(OCTOMAP_INCLUDE_DIRS AND OCTOMAP_LIBRARY_DIRS)
        set(OCTOMAP_LIBRARIES "octomap;octomath")
    endif()
endif()
if (OCTOMAP_INCLUDE_DIRS AND OCTOMAP_LIBRARY_DIRS)
  include_directories(${OCTOMAP_INCLUDE_DIRS})
  link_directories(${OCTOMAP_LIBRARY_DIRS})
  set(FCL_HAVE_OCTOMAP 1)
  message(STATUS "FCL uses Octomap")
else()
  message(STATUS "FCL does not use Octomap")
endif()

find_package(Boost COMPONENTS thread date_time filesystem system unit_test_framework REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

if(MSVC OR MSVC90 OR MSVC10)
    add_definitions(-DBOOST_ALL_NO_LIB)
endif()
add_definitions(-DBOOST_TEST_DYN_LINK)

include_directories("include")

if(PKG_CONFIG_FOUND)
    pkg_check_modules(CCD REQUIRED ccd)
endif()
# if(NOT CCD_FOUND)
#    # if pkfconfig is not installed, then fall back on more fragile detection
#    # of ccd
#    find_path(CCD_INCLUDE_DIRS ccd.h PATH_SUFFIXES ccd)
#    find_path(CCD_LIBRARY_DIRS ${CMAKE_SHARED_LIBRARY_PREFIX}ccd${CMAKE_SHARED_LIBRARY_SUFFIX})
#    if(CCD_INCLUDE_DIRS AND CCD_LIBRARY_DIRS)
        set(CCD_LIBRARIES "ccd")
#    else()
#        message(FATAL_ERROR "Libccd is required by FCL")
#    endif()
# endif()
# include_directories(${CCD_INCLUDE_DIRS})
# link_directories(${CCD_LIBRARY_DIRS})

add_subdirectory(include/fcl)
add_subdirectory(src)

set(pkg_conf_file "${CMAKE_CURRENT_SOURCE_DIR}/fcl.pc")
configure_file("${pkg_conf_file}.in" "${pkg_conf_file}" @ONLY)

install(DIRECTORY include/ DESTINATION include
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hxx"
  PATTERN ".DS_Store" EXCLUDE
  PATTERN ".svn" EXCLUDE
)

install(FILES "${pkg_conf_file}" DESTINATION lib/pkgconfig/ COMPONENT pkgconfig)

SET(CPACK_PACKAGE_VERSION "${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH}")
SET(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}-${CPACK_SYSTEM_NAME}")
SET(CPACK_DEBIAN_PACKAGE_MAINTAINER "Golems")

SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY ${PKG_DESC})

INCLUDE(CPack)

#enable_testing()
#add_subdirectory(test)
