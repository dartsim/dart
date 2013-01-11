###############################################################################
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)


###############################################################################
# These properties must be set before the project(...) line to work properly

# Disables a warning about a change in Cygwin Cmake
# Remove when CMake >= 2.8.4 is required
SET(CMAKE_LEGACY_CYGWIN_WIN32 0)


###############################################################################
# Project definition
project (dart)
message(STATUS "Generate makefile/project file for DART")

SET(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")
message(STATUS "DART CMake Modules: ${CMAKE_MODULE_PATH}")

SET(CPACK_PACKAGE_VERSION_MAJOR "2")
SET(CPACK_PACKAGE_VERSION_MINOR "1")
SET(CPACK_PACKAGE_VERSION_PATCH "0")
SET(CPACK_PACKAGE_NAME "dart")
SET(CPACK_SYSTEM_NAME "amd64")
SET(CPACK_DEBIAN_PACKAGE_DEPENDS "freeglut3, freeglut3-dev, libboost-dev, flann (>=1.8), libccd (>=1.4.2), lifcl (>=0.2.7)")

message(STATUS "CMAKE_BINARY_DIR = ${CMAKE_BINARY_DIR}")

option ( DART_UI "DART_UI" ON )
option ( SNOPT "SNOPT" OFF )
option ( PLANNING "PLANNING" ON )
option ( ROBOTICS "ROBOTICS" ON )
if( UNIX )
  option ( ENABLE_MSSE2 "Enable -msse2 flag" ON )
endif( UNIX )

###############################################################################
# SETTING: Global settings
INCLUDE_DIRECTORIES("src")

SET(BUILD_SHARED_LIBS OFF)
SET(CMAKE_CXX_WARNING_LEVEL 4)

# Can't test for WIN32 - We might be building on Cygwin
if (WIN32 AND NOT CYGWIN)
  message(STATUS "Setup Visual Studio Specific Flags")
  SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MDd /Zi /Gy /W1 /EHsc")
  SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MD /Zi /GL /Gy /W1 /EHsc")
  SET(CMAKE_DEBUG_POSTFIX "d")
  SET(CMAKE_INSTALL_PREFIX "C:/Golems" CACHE PATH "Install prefix" FORCE)
  SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "/LTCG")
#elseif (APPLE)
#elseif (UNIX AND NOT APPLE)
#elseif (UNIX)
#  if( ENABLE_MSSE2 )
#    ADD_DEFINITIONS("-msse2")
#  endif( ENABLE_MSSE2 )
#  SET(CMAKE_INSTALL_PREFIX /usr/local CACHE PATH "Install prefix")
#endif()
elseif (UNIX)  
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse2 -O3  -std=c++0x")
  SET(CMAKE_INSTALL_PREFIX /usr/local )
endif()

SET(THIRDPARTY_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/thirdparty CACHE PATH "Root directory for 'installing' thirdparty builds")

###############################################################################
# Generate the path header file
SET(PATH_HEADER_DIR ${CMAKE_SOURCE_DIR}/src/utils/Paths.h)
message(STATUS "Generate the path header file to  ${PATH_HEADER_DIR}")
CONFIGURE_FILE( ${PATH_HEADER_DIR}.in ${PATH_HEADER_DIR} @ONLY )

###############################################################################
# User specific settings
INCLUDE(Directories.cmake)
INCLUDE(ThirdParties.cmake)

FIND_PATH( MATHPLOT_INCLUDE_DIR NAMES mathplot2.h )
message(STATUS "INCLUDES = ${CMAKE_INCLUDE_PATH}")
message(STATUS "MATHPLOT = ${MATHPLOT_INCLUDE_DIR}")

###############################################################################
# Find important packages

# OpenGL
FIND_PACKAGE(OpenGL)
IF (OPENGL_FOUND)
  MESSAGE(STATUS "OPENGL FOUND")
ELSE (OPENGL_FOUND)
  MESSAGE(STATUS "OPENGL NOT FOUND")
ENDIF ()

# GLUT
if(WIN32 AND NOT CYGWIN)
    MESSAGE(STATUS "Defaulting to provided GLUT libraries. Change GLUT_PREFIX_PATH to use a different version of GLUT")
    SET(GLUT_PREFIX_PATH "C:/Golems/" CACHE PATH "Root directory of GLUT installation")
	INCLUDE_DIRECTORIES( "C:/Golems/include" )
	INCLUDE_DIRECTORIES( "C:/Golems/lib" )
endif(WIN32 AND NOT CYGWIN)
if(DEFINED GLUT_PREFIX_PATH)
    SET(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${GLUT_PREFIX_PATH})
endif(DEFINED GLUT_PREFIX_PATH)

FIND_PACKAGE(GLUT REQUIRED)
INCLUDE_DIRECTORIES( ${GLUT_INCLUDE_DIR} )


# Boost and Assimp Boost Workaround
set(Boost_USE_MULTITHREADED      ON)
set(Boost_USE_STATIC_RUNTIME    OFF)
FIND_PACKAGE(Boost 1.42.0 REQUIRED)
add_definitions( -DBOOST_ALL_DYN_LINK )
include_directories( ${Boost_INCLUDE_DIRS} )
link_directories( ${Boost_LIBRARY_DIRS} )

# FLANN
if(WIN32 AND NOT CYGWIN)
    SET(FLANN_PREFIX_PATH "C:/Program Files/flann" CACHE PATH "Root directory of FLANN installation")
endif(WIN32 AND NOT CYGWIN)
if(DEFINED FLANN_PREFIX_PATH)
    SET(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${FLANN_PREFIX_PATH})
endif(DEFINED FLANN_PREFIX_PATH)

#Eigen
FIND_PACKAGE(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
message (STATUS "Eigen directory is ${EIGEN3_INCLUDE_DIR}")

###############################################################################
# Thirdparty libraries
message(STATUS "Configuring thirdparty libraries")

ADD_SUBDIRECTORY (thirdparty/gtest-1.6.0)

ADD_SUBDIRECTORY (thirdparty/tinyxml2)

ADD_SUBDIRECTORY (thirdparty/tinyxml)

if ( SNOPT )
 SET( snopt_folder ${CMAKE_SOURCE_DIR}/thirdparty/snopt )
 ADD_SUBDIRECTORY ( ${snopt_folder} )
endif ( SNOPT )

###############################################
# DART Source
message(STATUS "Building DART source")
ADD_SUBDIRECTORY(src)

###############################################################################
# Install Targets
# Generate the Dart CMake Config file
SET(DART_CONFIG_FILE DARTConfig.cmake)
SET(DART_CONFIG ${CMAKE_SOURCE_DIR}/${DART_CONFIG_FILE})
FILE(GLOB cmake_mods "cmake/Modules/*.cmake")
message(STATUS "Generating ${DART_CONFIG_FILE}")
CONFIGURE_FILE( ${DART_CONFIG}.in ${DART_CONFIG} @ONLY )
INSTALL(FILES ${DART_CONFIG} DESTINATION share/dart)
INSTALL(FILES ${cmake_mos} DESTINATION share/dart/cmake)

###############################################################################
# Doxygen
FIND_PACKAGE(Doxygen)
IF (DOXYGEN_FOUND)

  SET(DOXYGEN_INPUT "Doxyfile")
  SET(DOXYGEN_OUTPUT "doc")

  ADD_CUSTOM_COMMAND(
    OUTPUT ${DOXYGEN_OUTPUT}
    COMMAND ${CMAKE_COMMAND} -E echo_append "Building API Documentation..."
    COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_INPUT}
    COMMAND ${CMAKE_COMMAND} -E echo "Done."
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    DEPENDS ${DOXYGEN_INPUT}
    )

  ADD_CUSTOM_TARGET(docs DEPENDS ${DOXYGEN_OUTPUT})
  # ADD_CUSTOM_TARGET(apidoc ALL DEPENDS ${DOXYGEN_OUTPUT})

  ADD_CUSTOM_TARGET(docs_forced
    COMMAND ${CMAKE_COMMAND} -E echo_append "Building API Documentation..."
    COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_INPUT}
    COMMAND ${CMAKE_COMMAND} -E echo "Done."
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

ENDIF (DOXYGEN_FOUND)

###############################################################################
# Enable a test targets for running CTest executables
enable_testing()
add_test (unittests ${CMAKE_SOURCE_DIR}/bin/unittests)
add_test (test_core_collision ${CMAKE_SOURCE_DIR}/bin/test_core_collision)

###############################################################################
# Package Installer

SET(CPACK_PACKAGE_VERSION "${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH}")
SET(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}-${CPACK_SYSTEM_NAME}")
SET(CPACK_DEBIAN_PACKAGE_MAINTAINER "Karen Liu (www.cc.gatech.edu/~karenliu/)")

SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Dynamic Animation and Robotics Toolkit.")
SET(CPACK_PACKAGE_VENDOR "Computer Graphics Lab at GT")

INCLUDE(CPack)
