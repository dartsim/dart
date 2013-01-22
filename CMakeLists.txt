cmake_minimum_required(VERSION 2.8)
set(CMAKE_LEGACY_CYGWIN_WIN32 0)
set(CMAKE_CXX_WARNING_LEVEL 4) 
# Disables a warning about a change in Cygwin Cmake

project (dart)
message(STATUS "Generate makefile/project file for DART")

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")

set(DART_MAJOR_VERSION "2")
set(DART_MINOR_VERSION "1")
set(DART_PATCH_VERSION "1")
set(PKG_DESC "Dynamic Animation and Robotics Toolkit.")
set(DART_VERSION "${DART_MAJOR_VERSION}.${DART_MINOR_VERSION}.${DART_PATCH_VERSION}")
set(PKG_EXTERNAL_DEPS "flann, ccd, fcl")

message(STATUS "CMAKE_BINARY_DIR = ${CMAKE_BINARY_DIR}")

option(DART_UI "Enable DART yui" ON)
option(SNOPT "Build for SNOPT optimizer" OFF)
option(PLANNING "Compile Planning Libraries" ON)
option(ROBOTICS "Compile Robotics API" ON)
if(UNIX)
  option(ENABLE_MSSE2 "Enable -msse2 flag" ON)
endif()

# Can't test for WIN32 - We might be building on Cygwin
if (WIN32 AND NOT CYGWIN)
  message(STATUS "Setup Visual Studio Specific Flags")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MDd /Zi /Gy /W1 /EHsc")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MD /Zi /GL /Gy /W1 /EHsc")
  set(CMAKE_DEBUG_POSTFIX "d")
  set(CMAKE_INSTALL_PREFIX "C:/Golems" CACHE PATH "Install prefix" FORCE)
  set(CMAKE_EXE_LINKER_FLAGS_RELEASE "/LTCG")
elseif (UNIX)  
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse2 -O3 -std=c++0x")
  set(CMAKE_CXX_FLAGS_PROFILE "-pg" CACHE STRING "Profiling flags") 
  set(CMAKE_INSTALL_PREFIX /usr )
endif()

set(THIRDPARTY_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/thirdparty CACHE PATH "Root directory for 'installing' thirdparty builds")

###############
# Generate the path header file:i
set(PATH_HEADER_DIR ${CMAKE_SOURCE_DIR}/src/utils/Paths.h)
message(STATUS "Generate the path header file to  ${PATH_HEADER_DIR}")
configure_file(${PATH_HEADER_DIR}.in ${PATH_HEADER_DIR} @ONLY)

###############
# User specific settings
include(cmake/Directories.cmake)
include(cmake/ThirdParties.cmake)

find_path( MATHPLOT_INCLUDE_DIR NAMES mathplot2.h )
message(STATUS "MATHPLOT = ${MATHPLOT_INCLUDE_DIR}")

###############
# Find important packages

find_package(PkgConfig QUIET)

# OpenGL
find_package(OpenGL REQUIRED)

# GLUT
if(WIN32 AND NOT CYGWIN)
    message(STATUS "Defaulting to provided GLUT libraries. Change GLUT_PREFIX_PATH to use a different version of GLUT")
    set(GLUT_PREFIX_PATH "C:/Golems/" CACHE PATH "Root directory of GLUT installation")
	include_directories("C:/Golems/include")
	include_directories("C:/Golems/lib")
elseif(UNIX)
find_package(GLUT REQUIRED)
    include_directories(${GLUT_INCLUDE_DIR})
endif()

# Boost and Assimp Boost Workaround
find_package(Boost REQUIRED system filesystem)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

if(MSVC OR MSVC90 OR MSVC10)
    add_definitions(-DBOOST_ALL_NO_LIB)
endif()
add_definitions(-DBOOST_TEST_DYN_LINK)
set(Boost_USE_MULTITHREADED      ON)
set(Boost_USE_STATIC_RUNTIME    OFF)

# FLANN
if(PKG_CONFIG_FOUND)
   pkg_check_modules(PC_FLANN flann)
   find_path(FLANN_INCLUDEDIR flann/flann.h 
   PATHS ${CMAKE_INSTALL_PREFIX}/include ${PC_FLANN_INCLUDEDIR} PATH_SUFFIXES flann)
   set(FLANN_LIBRARIES ${PC_FLANN_LIBRARIES})
endif()

# CCD
if(PKG_CONFIG_FOUND)
   pkg_check_modules(PC_CCD ccd)
   find_path(CCD_INCLUDEDIR ccd/ccd.h
   PATHS ${CMAKE_INSTALL_PREFIX}/include ${PC_CCD_INCLUDEDIR} PATH_SUFFIXES ccd)
   set(CCD_LIBRARIES ${PC_CCD_LIBRARIES})
endif()

# FCL
if(PKG_CONFIG_FOUND)
   pkg_check_modules(PC_FCL fcl)
   find_path(FCL_INCLUDEDIR fcl/collision.h 
   PATHS ${CMAKE_INSTALL_PREFIX}/include ${PC_FCL_INCLUDEDIR} PATH_SUFFIXES fcl)
   set(FCL_LIBRARIES ${PC_FCL_LIBRARIES})
endif()

#Eigen
find_path(EIGEN3_INCLUDEDIR Eigen/Core
    PATHS ${CMAKE_INSTALL_PREFIX}/include PATH_SUFFIXES eigen3 eigen)

include_directories(${FLANN_INCLUDEDIR} ${CCD_INCLUDEDIR} ${FCL_INCLUDEDIR} ${EIGEN3_INCLUDEDIR})

###############
# Thirdparty libraries
message(STATUS "Configuring thirdparty libraries")

add_subdirectory(thirdparty/gtest-1.6.0)
add_subdirectory(thirdparty/tinyxml2)
add_subdirectory(thirdparty/tinyxml)

if(SNOPT)
 set(snopt_folder ${CMAKE_SOURCE_DIR}/thirdparty/snopt)
 add_subdirectory(${snopt_folder})
endif()

###############
# DART Source
message(STATUS "Building DART source")
set(DART_LIBRARIES "")
set(DART_INCLUDEDIR "")
include_directories(src)
add_subdirectory(src)

###############
# Install Targets
# Generate the Dart CMake Config file
file(GLOB cmake_mods "cmake/Modules/*.cmake")
configure_file(${CMAKE_SOURCE_DIR}/cmake/DARTConfig.cmake.in ${CMAKE_SOURCE_DIR}/DARTConfig.cmake @ONLY)
configure_file(${CMAKE_SOURCE_DIR}/cmake/DARTConfig.cmake.in ${CMAKE_SOURCE_DIR}/FindDART.cmake @ONLY)
install(FILES ${CMAKE_SOURCE_DIR}/DARTConfig.cmake DESTINATION share/dart)
install(FILES ${CMAKE_SOURCE_DIR}/FindDART.cmake DESTINATION ${CMAKE_ROOT}/Modules)
install(FILES ${cmake_mods} DESTINATION share/dart/cmake)

set(PC_CONFIG ${CMAKE_SOURCE_DIR}/cmake/dart.pc)
configure_file(${PC_CONFIG}.in ${PC_CONFIG} @only)
install(FILES ${PC_CONFIG} DESTINATION lib/pkgconfig)

# Add an "uninstall" target
configure_file ("${PROJECT_SOURCE_DIR}/cmake/uninstall_target.cmake.in" "${PROJECT_BINARY_DIR}/uninstall_target.cmake" IMMEDIATE @ONLY)
add_custom_target (uninstall "${CMAKE_COMMAND}" -P "${PROJECT_BINARY_DIR}/uninstall_target.cmake")

###############
# Doxygen
find_package(Doxygen)
if(DOXYGEN_FOUND)
  set(DOXYGEN_INPUT "Doxyfile")
  set(DOXYGEN_OUTPUT "doc")

  add_custom_command(
    OUTPUT ${DOXYGEN_OUTPUT}
    COMMAND ${CMAKE_COMMAND} -E echo_append "Building API Documentation..."
    COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_INPUT}
    COMMAND ${CMAKE_COMMAND} -E echo "Done."
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    DEPENDS ${DOXYGEN_INPUT}
    )

  add_custom_target(docs DEPENDS ${DOXYGEN_OUTPUT})
  # add_custom_target(apidoc ALL DEPENDS ${DOXYGEN_OUTPUT})

  add_custom_target(docs_forced
    COMMAND ${CMAKE_COMMAND} -E echo_append "Building API Documentation..."
    COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_INPUT}
    COMMAND ${CMAKE_COMMAND} -E echo "Done."
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
endif()

###############
# Enable a test targets for running CTest executables
enable_testing()
add_test (unittests ${CMAKE_SOURCE_DIR}/bin/unittests)
add_test (test_core_collision ${CMAKE_SOURCE_DIR}/bin/test_core_collision)

###############
# Package Installer
set(CPACK_PACKAGE_NAME "dart")
set(CPACK_SYSTEM_NAME "amd64")
set(CPACK_PACKAGE_VERSION_MAJOR ${DART_MAJOR_VERSION})
set(CPACK_PACKAGE_VERSION_MINOR ${DART_MINOR_VERSION})
set(CPACK_PACKAGE_VERSION_PATCH ${DART_PATCH_VERSION})
set(CPACK_DEBIAN_PACKAGE_DEPENDS "freeglut3, freeglut3-dev, libboost-dev, flann (>=1.8), libccd (>=1.4.2), libfcl (>=0.2.7), libeigen3-dev, libxmu-dev, libxi-dev")

set(CPACK_PACKAGE_VERSION "${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH}")
set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}-${CPACK_SYSTEM_NAME}")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Karen Liu (www.cc.gatech.edu/~karenliu/)")

set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "${PKG_DESC}")
set(CPACK_PACKAGE_VENDOR "Computer Graphics Lab at GT")

include(CPack)
