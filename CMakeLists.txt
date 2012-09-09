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

add_definitions(-g)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

SET(CPACK_PACKAGE_VERSION_MAJOR "0")
SET(CPACK_PACKAGE_VERSION_MINOR "1")
SET(CPACK_PACKAGE_VERSION_PATCH "2")
SET(CPACK_PACKAGE_NAME "dart")
SET(CPACK_SYSTEM_NAME "i386")
SET(CPACK_DEBIAN_PACKAGE_DEPENDS "glutg3 (>= 3.7), glutg3-dev (>= 3.7), cmake (>=2.8), cmake-curses-gui (>=2.8), libflann1.7-dev (>=1.7), libboost-dev")

message(STATUS "CMAKE_BINARY_DIR = ${CMAKE_BINARY_DIR}")

option ( DART_UI "DART_UI" ON )
option ( SNOPT "SNOPT" OFF )
option ( PLANNING "PLANNING" OFF )
option ( ROBOTICS "ROBOTICS" OFF )
if( UNIX )
  option ( ENABLE_MSSE2 "Enable -msse2 flag" ON )
endif( UNIX )

###############################################################################
# SETTING: Global settings
INCLUDE_DIRECTORIES("src")

SET(BUILD_SHARED_LIBS OFF)
SET(CMAKE_CXX_WARNING_LEVEL 4)
SET(EIGEN_DIR ${CMAKE_SOURCE_DIR}/thirdparty/eigen-3.0.5)

# Can't test for WIN32 - We might be building on Cygwin
if (MSVC)
  message(STATUS "Setup Visual Studio Specific Flags")
  SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MDd /Zi /GL /Gy /W1 /EHsc")
  SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MD /Zi /GL /Gy /W1 /EHsc")
  SET(CMAKE_DEBUG_POSTFIX "d")
  set(CMAKE_EXE_LINKER_FLAGS "/LTCG")# "${CMAKE_EXE_LINKER_FLAGS} /LTCG")
  set(CMAKE_MODULE_LINKER_FLAGS "/LTCG") #"${CMAKE_MODULE_LINKER_FLAGS} /LTCG")
  SET(CMAKE_INSTALL_PREFIX "C:/Golems" CACHE PATH "Install prefix")
#elseif (APPLE)
#elseif (UNIX AND NOT APPLE)
elseif (UNIX)
  if( ENABLE_MSSE2 )
    ADD_DEFINITIONS("-msse2")
  endif( ENABLE_MSSE2 )
  SET(CMAKE_INSTALL_PREFIX /usr/local CACHE PATH "Install prefix")
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
    SET(GLUT_PREFIX_PATH "${CMAKE_SOURCE_DIR}/thirdparty/glut-win" CACHE PATH "Root directory of GLUT installation")
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


# FLANN
if(WIN32 AND NOT CYGWIN)
    SET(FLANN_PREFIX_PATH "C:/Program Files/flann" CACHE PATH "Root directory of FLANN installation")
endif(WIN32 AND NOT CYGWIN)
if(DEFINED FLANN_PREFIX_PATH)
    SET(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${FLANN_PREFIX_PATH})
endif(DEFINED FLANN_PREFIX_PATH)


###############################################################################
# Thirdparty libraries
message(STATUS "Configuring thirdparty libraries")

ADD_SUBDIRECTORY (thirdparty/gtest-1.6.0)

ADD_SUBDIRECTORY (thirdparty/gflags-2.0)
MESSAGE(STATUS "gflags include dir: ${GFLAGS_INCLUDE_PATH}")
include_directories( ${GFLAGS_INCLUDE_PATH} )

ADD_SUBDIRECTORY (thirdparty/glog-0.3.2)
MESSAGE(STATUS "glog include dir: ${GLOG_INCLUDE_PATH}")
include_directories( ${GLOG_INCLUDE_PATH} )


ADD_SUBDIRECTORY (thirdparty/tinyxml2)

ADD_SUBDIRECTORY (thirdparty/libccd)
SET(CCD_FOUND TRUE)
SET(CCD_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/libccd/src")
SET(CCD_LIBRARY_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/libccd/lib")
SET(CCD_LIBRARIES ccd)

include_directories(thirdparty/fcl/include)
ADD_SUBDIRECTORY (thirdparty/fcl)

if (PLANNING OR ROBOTICS)
    if(TARGET zlib)
        SET_TARGET_PROPERTIES( zlib PROPERTIES FOLDER thirdParty/assimp )
    endif(TARGET zlib)
endif (PLANNING OR ROBOTICS)

if (SNOPT)
    ADD_SUBDIRECTORY (thirdparty/snopt)
endif (SNOPT)

###############################################
# DART Source
message(STATUS "Building DART source")
ADD_SUBDIRECTORY(src)

###############################################################################
# Install Targets
# Generate the Dart CMake Config file
SET(DART_CONFIG_FILE DARTConfig.cmake)
SET(DART_CONFIG ${CMAKE_SOURCE_DIR}/${DART_CONFIG_FILE})
message(STATUS "Generating ${DART_CONFIG_FILE}")
CONFIGURE_FILE( ${DART_CONFIG}.in ${DART_CONFIG} @ONLY )
INSTALL(FILES ${DART_CONFIG} DESTINATION share/dart)

# Install Eigen
INSTALL(DIRECTORY ${EIGEN_DIR}/Eigen DESTINATION include/eigen-3.0.5)
INSTALL(DIRECTORY ${EIGEN_DIR}/unsupported DESTINATION include/eigen-3.0.5)
INSTALL(FILES ${EIGEN_DIR}/signature_of_eigen3_matrix_library DESTINATION include/eigen-3.0.5)

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
