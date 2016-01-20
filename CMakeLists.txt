#===============================================================================
# CMake settings
#===============================================================================
if(MSVC)
  cmake_minimum_required(VERSION 3.1.3)
else()
  cmake_minimum_required(VERSION 2.8.6)
endif()

# Use MACOSX_RPATH by default on OS X. This was added in CMake 2.8.12 and
# became default in CMake 3.0. Explicitly setting this policy is necessary to
# suppress a warning in CMake 3.0 and above.
if(POLICY CMP0042)
  cmake_policy(SET CMP0042 NEW)
endif()

# Simplify variable reference and escape sequence evaluation. This was added in
# CMake 3.1. Explicitly setting this policy is necessary to suppress a warning
# in CMake 3.1 and above.
if(POLICY CMP0053)
  cmake_policy(SET CMP0053 NEW)
endif()

# Disables a warning about a change in Cygwin Cmake
set(CMAKE_LEGACY_CYGWIN_WIN32 0)
set(CMAKE_CXX_WARNING_LEVEL 4)

set(CMAKE_DEBUG_POSTFIX "d")
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

include(${CMAKE_MODULE_PATH}/KIDOMacros.cmake)

# System install paths on Windows
if(WIN32)
  set(CMAKE_INSTALL_PREFIX "C:/Golems" CACHE PATH "Install prefix" FORCE)
endif()

#===============================================================================
# Project settings
#===============================================================================
#
# If you change the version, please update the <version> tag in package.xml.

project(kido)

set(KIDO_MAJOR_VERSION "0")
set(KIDO_MINOR_VERSION "1")
set(KIDO_PATCH_VERSION "0")
set(KIDO_VERSION "${KIDO_MAJOR_VERSION}.${KIDO_MINOR_VERSION}.${KIDO_PATCH_VERSION}")
set(KIDO_PKG_DESC "Kinematics, Dynamics and Optimization Library for Animation and Robotics")
set(KIDO_PKG_EXTERNAL_DEPS "flann, ccd, fcl")

#===============================================================================
# Print intro
#===============================================================================
message(STATUS "")
message(STATUS "============================================")
message(STATUS "                KIDO ${KIDO_VERSION}")
message(STATUS "============================================")

#===============================================================================
# Build options
#===============================================================================
option(ENABLE_OPENMP "Build with OpenMP parallaization enabled" ON)
if(MSVC)
  set(KIDO_RUNTIME_LIBRARY "/MD" CACHE STRING "BaseName chosen by the user at CMake configure time")
  set_property(CACHE KIDO_RUNTIME_LIBRARY PROPERTY STRINGS /MD /MT)
  option(KIDO_MSVC_DEFAULT_OPTIONS "Build KIDO with default Visual Studio options" OFF)
else()
  option(BUILD_SHARED_LIBS "Build shared libraries" ON)
endif()
option(KIDO_BUILD_OSGKIDO "Build osgKido library" ON) # TODO(JS): Rename the option name to more relevant name to osg GUI and add more options for other extensions (#590).
option(KIDO_BUILD_EXAMPLES "Build examples" ON)
option(KIDO_BUILD_TUTORIALS "Build tutorials" ON)
option(KIDO_BUILD_UNITTESTS "Build unit tests" ON)

#===============================================================================
# Build type settings
#===============================================================================
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build, options are: Debug | Release | RelWithDebInfo | MinSizeRel" FORCE)
endif()
string(TOUPPER ${CMAKE_BUILD_TYPE} CMAKE_BUILD_TYPE_UPPERCASE)

set(BUILD_TYPE_DEBUG FALSE)
set(BUILD_TYPE_RELEASE FALSE)
set(BUILD_TYPE_RELWITHDEBINFO FALSE)
set(BUILD_TYPE_MINSIZEREL FALSE)

if("${CMAKE_BUILD_TYPE_UPPERCASE}" STREQUAL "DEBUG")
  set(BUILD_TYPE_DEBUG TRUE)
elseif("${CMAKE_BUILD_TYPE_UPPERCASE}" STREQUAL "RELEASE")
  set(BUILD_TYPE_RELEASE TRUE)
elseif("${CMAKE_BUILD_TYPE_UPPERCASE}" STREQUAL "RELWITHDEBINFO")
  set(BUILD_TYPE_RELWITHDEBINFO TRUE)
elseif("${CMAKE_BUILD_TYPE_UPPERCASE}" STREQUAL "MINSIZEREL")
  set(BUILD_TYPE_MINSIZEREL TRUE)
else()
  build_error("CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} unknown. Valid options are: Debug | Release | RelWithDebInfo | MinSizeRel")
endif()

#===============================================================================
# Find dependencies
#===============================================================================
#
# If you add a dependency, please add the corresponding rosdep key as a
# dependency in package.xml.

#------------------------
# Mandatory dependencies
#------------------------
message(STATUS "")
message(STATUS "[ Mandatory dependencies ]")

# Eigen
find_package(EIGEN3 3.0.5 QUIET)
if(EIGEN3_FOUND)
  message(STATUS "Looking for EIGEN3 - ${EIGEN3_VERSION} found")
else()
  if(PC_EIGEN3_VERSION)
    message(SEND_ERROR "Looking for EIGEN3 - ${EIGEN3_VERSION} found, ${PROJECT_NAME} requires 3.0.5 or greater.")
  else()
    message(SEND_ERROR "Looking for EIGEN3 - NOT found, please install libeigen3-dev")
  endif()
endif()

# CCD
find_package(CCD 1.4.0 QUIET)
if(CCD_FOUND)
  message(STATUS "Looking for CCD - ${CCD_VERSION} found")
else()
  if(CCD_VERSION)
    message(SEND_ERROR "Looking for CCD - ${CCD_VERSION} found, ${PROJECT_NAME} requires 1.4.0 or greater.")
  else()
    message(SEND_ERROR "Looking for CCD - NOT found, please install libccd-dev")
  endif()
endif()

# FCL
# TODO: We need to increate the minimum required fcl version to 0.4.0 for KIDO
# 7.0 once fcl 0.4.0 is released.
find_package(FCL 0.2.9 QUIET)
if(FCL_FOUND)
  message(STATUS "Looking for FCL - ${FCL_VERSION} found")
else()
  if(FCL_VERSION)
    message(SEND_ERROR "Looking for FCL - ${FCL_VERSION} found, ${PROJECT_NAME} requires 0.2.9 or greater.")
  else()
    message(SEND_ERROR "Looking for FCL - NOT found, please install libfcl-dev")
  endif()
endif()

# ASSIMP
find_package(ASSIMP 3.0.0 QUIET)
if(ASSIMP_FOUND)
  message(STATUS "Looking for ASSIMP - ${ASSIMP_VERSION} found")

  # Check for missing symbols in ASSIMP (see #451)
  include(CheckCXXSourceCompiles)
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
    message(WARNING "The installed version of ASSIMP (${ASSIMP_VERSION}) is "
                    "missing symbols for the constructor and/or destructor of "
                    "aiScene. KIDO will use its own implementations of these "
                    "functions. We recommend using a version of ASSIMP that "
                    "does not have this issue, once one becomes available.")
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
    message(WARNING "The installed version of ASSIMP (${ASSIMP_VERSION}) is "
                    "missing symbols for the constructor and/or destructor of "
                    "aiMaterial. KIDO will use its own implementations of "
                    "these functions. We recommend using a version of ASSIMP "
                    "that does not have this issue, once one becomes available.")
  endif(NOT ASSIMP_AIMATERIAL_CTOR_DTOR_DEFINED)

  unset(CMAKE_REQUIRED_INCLUDES)
  unset(CMAKE_REQUIRED_LIBRARIES)

else()
  if(ASSIMP_VERSION)
    message(SEND_ERROR "Looking for ASSIMP - ${ASSIMP_VERSION} found, ${PROJECT_NAME} requires 3.0.0 or greater.")
  else()
    message(SEND_ERROR "Looking for ASSIMP - NOT found, please install libassimp-dev")
  endif()
endif()

# OpenGL
find_package(OpenGL QUIET)
if(OPENGL_FOUND)
  message(STATUS "Looking for OpenGL - found")
else()
  message(SEND_ERROR "Looking for OpenGL - NOT found, please install OpenGL")
endif()

# Boost
set(KIDO_MIN_BOOST_VERSION 1.46.0 CACHE INTERNAL "Boost min version requirement" FORCE)
if(MSVC OR MSVC90 OR MSVC10)
  add_definitions(-DBOOST_ALL_NO_LIB)
endif()
add_definitions(-DBOOST_TEST_DYN_LINK)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost ${KIDO_MIN_BOOST_VERSION} COMPONENTS regex system QUIET)
if(Boost_FOUND)
  message(STATUS "Looking for Boost - ${Boost_MAJOR_VERSION}.${Boost_MINOR_VERSION}.${Boost_SUBMINOR_VERSION} found")
else()
  message(SEND_ERROR "Please install system boost version ${KIDO_MIN_BOOST_VERSION} or higher.")
endif()

#-----------------------
# Optional dependencies
#-----------------------
message(STATUS "")
message(STATUS "[ Optional dependencies ]")

# GLUT
if(WIN32 AND NOT CYGWIN)
  set(GLUT_INCLUDE_DIR "@CMAKE_INSTALL_PREFIX@/include")
  set(GLUT_LIBRARIES glut32)
else()
  find_package(GLUT QUIET)
  if(GLUT_FOUND)
    message(STATUS "Looking for GLUT - found")
    set(GLUT_LIBRARIES ${GLUT_glut_LIBRARY})
  else()
    message(STATUS "Looking for GLUT - NOT found")
    message(STATUS "To enable component kido-gui, please install freeglut3-dev")
  endif()
endif()

# OpenSceneGraph
find_package(OpenSceneGraph 3.0 QUIET
  COMPONENTS osg osgViewer osgManipulator osgGA osgDB)
if(OPENSCENEGRAPH_FOUND)
  message(STATUS "Looking for OpenSceneGraph -- ${OPENSCENEGRAPH_VERSION} found")
  set(HAVE_OPENSCENEGRAPH TRUE)
else(OPENSCENEGRAPH_FOUND)
  # kido-gui-osg requires both OSG and OpenThreads. This section attempts to
  # identify which of those are missing from the building machine and offer
  # advice to the user for getting osgKido to build.
  find_package(OpenThreads QUIET)
  if(OPENTHREADS_FOUND)
    set(warning_msg "Could NOT find OpenSceneGraph")
  else(OPENTHREADS_FOUND)
    if(OSG_LIBRARY)
      set(warning_msg "Could NOT find OpenThreads")
    else(OSG_LIBRARY)
      set(warning_msg "Could NOT find OpenSceneGraph nor OpenThreads")
    endif(OSG_LIBRARY)
  endif(OPENTHREADS_FOUND)
  message(WARNING "${warning_msg} -- we will skip kido-gui-osg\n"
          "If you believe you do have both OSG and OpenThreads installed, try setting OSG_DIR")
  set(HAVE_OPENSCENEGRAPH FALSE)
endif(OPENSCENEGRAPH_FOUND)

# FLANN
find_package(FLANN 1.8.4 QUIET)
if(FLANN_FOUND)
  message(STATUS "Looking for FLANN - ${FLANN_VERSION} found")
else()
  if(FLANN_VERSION)
    message(STATUS "Looking for FLANN - ${FLANN_VERSION} (< 1.8.4) found")
    message(STATUS "To enable component kido-planning, please install libflann-dev 1.8.4 or greater")
  else()
    message(STATUS "Looking for FLANN - NOT found")
    message(STATUS "To enable component kido-planning, please install libflann-dev")
  endif()
endif()

# TINYXML
find_package(TINYXML 2.6.2 QUIET)
if(TINYXML_FOUND)
  message(STATUS "Looking for TINYXML - ${TINYXML_VERSION} found")
else()
  if(TINYXML_VERSION)
    message(STATUS "Looking for TINYXML - ${TINYXML_VERSION} (< 2.6.2) found")
    message(STATUS "To enable component kido-utils, please install libtinyxml-dev 2.6.2 or greater")
  else()
    message(STATUS "Looking for TINYXML - NOT found")
    message(STATUS "To enable component kido-utils, please install libtinyxml-dev")
  endif()
endif()

# TINYXML2
find_package(TINYXML2 QUIET)
if(TINYXML2_FOUND)
  message(STATUS "Looking for TINYXML2 - ${TINYXML2_VERSION} found")
else()
  if(TINYXML2_VERSION)
    message(STATUS "Looking for TINYXML2 - ${TINYXML2_VERSION} (< 1.0.1) found")
    message(STATUS "To enable component kido-utils, please install libtinyxml2-dev 1.0.1 or greater")
  else()
    message(STATUS "Looking for TINYXML2 - NOT found")
    message(STATUS "To enable component kido-utils, please install libtinyxml2-dev")
  endif()
endif()

# urdfdom
find_package(urdfdom QUIET)
if(urdfdom_FOUND)
  message(STATUS "Looking for urdfdom - found")
else()
  message(STATUS "Looking for urdfdom - NOT found, please install liburdfdom-dev")
  message(STATUS "To enable component kido-utils, please install liburdfdom-dev")
endif()
if(MSVC)
  set(urdfdom_LIBRARIES optimized urdfdom_sensor      debug urdfdom_sensord
                        optimized urdfdom_model_state debug urdfdom_model_stated
                        optimized urdfdom_model       debug urdfdom_modeld
                        optimized urdfdom_world       debug urdfdom_worldd
                        optimized console_bridge      debug console_bridged)
endif()

# OpenMP
if(ENABLE_OPENMP)
  find_package(OpenMP QUIET)
  if(OPENMP_FOUND)
    message(STATUS "Looking for OpenMP - found")
  else()
    message(STATUS "Looking for OpenMP - NOT found")
  endif()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# NLOPT
find_package(NLOPT 2.4.1 QUIET)
if(NLOPT_FOUND)
  message(STATUS "Looking for NLOPT - ${NLOPT_VERSION} found")
  set(HAVE_NLOPT TRUE)
else()
  if(NLOPT_VERSION)
    message(STATUS "Looking for NLOPT - ${NLOPT_VERSION} (< 2.4.1) found")
    message(STATUS "To enable component kido-optimizer-nlopt, please install libnlopt-dev 2.4.1 or greater")
  else()
    message(STATUS "Looking for NLOPT - NOT found")
    message(STATUS "To enable component kido-optimizer-nlopt, please install libnlopt-dev")
  endif()
  set(HAVE_NLOPT FALSE)
endif()

# IPOPT
find_package(IPOPT 3.11.4 QUIET)
if(IPOPT_FOUND)
  message(STATUS "Looking for IPOPT - ${IPOPT_VERSION} found")
  set(HAVE_IPOPT TRUE)
else()
  if(IPOPT_VERSION)
    message(STATUS "Looking for IPOPT - ${IPOPT_VERSION} (< 3.11.4) found")
    message(STATUS "To enable component kido-optimizer-ipopt, please install libipopt-dev 3.11.4 or greater")
  else()
    message(STATUS "Looking for IPOPT - NOT found")
    message(STATUS "To enable component kido-optimizer-ipopt, please install libipopt-dev")
  endif()
  set(HAVE_IPOPT FALSE)
endif()

# Shark
find_package(SHARK QUIET)
if(SHARK_FOUND)
  message(STATUS "Looking for SHARK - ${SHARK_VERSION} found")
  set(HAVE_SHARK TRUE)
else()
  message(STATUS "Looking for SHARK - NOT found, please install SHARK (http://image.diku.dk/shark)")
  set(HAVE_SHARK FALSE)
endif()

# BulletCollision
if(UNIX)
  pkg_check_modules(BULLET bullet>=2.82 QUIET)
  if(NOT BULLET_FOUND)
     pkg_check_modules(BULLET bullet2.82>=2.82 QUIET)
  endif()
else()
  find_package(Bullet COMPONENTS BulletMath BulletCollision QUIET)
endif()
if(BULLET_FOUND)
  message(STATUS "Looking for BulletCollision - ${BULLET_VERSION} found")
  set(HAVE_BULLET_COLLISION TRUE)
  add_definitions(-DHAVE_BULLET_COLLISION)
else()
  message(STATUS "Looking for BulletCollision - NOT found")
  message(STATUS "To enable bullet collision detection, please install libbullet-dev")
  set(HAVE_BULLET_COLLISION FALSE)
endif()

# Perl modules
find_package(PerlModules COMPONENTS Regexp::Common Getopt::ArgvFile Getopt::Long Term::ANSIColor QUIET)
if("${PERLMODULES_FOUND}" STREQUAL "TRUE")
  message(STATUS "Looking for PerlModules - found")
else()
  message(STATUS "Looking for PerlModules - NOT found, please install Regexp::Common Getopt::ArgvFile Getopt::Long Term::ANSIColor (http://www.cpan.org/modules/INSTALL.html)")
endif()

# Doxygen
find_package(Doxygen QUIET)
if(DOXYGEN_FOUND)
  message(STATUS "Looking for Doxygen - found")
else()
  message(STATUS "Looking for Doxygen - NOT found, please install doxygen")
endif()

#===============================================================================
# Include directories
#===============================================================================
include_directories(BEFORE ${PROJECT_SOURCE_DIR})
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})
include_directories(SYSTEM ${CCD_INCLUDE_DIRS})
include_directories(SYSTEM ${FCL_INCLUDE_DIRS})
include_directories(SYSTEM ${ASSIMP_INCLUDE_DIRS})
include_directories(SYSTEM ${Boost_INCLUDE_DIRS})
include_directories(SYSTEM ${OPENGL_INCLUDE_DIR})
if(HAVE_BULLET_COLLISION)
  include_directories(SYSTEM ${BULLET_INCLUDE_DIRS})
  add_definitions(${BULLET_CFLAGS})
endif()
include_directories("${CMAKE_BINARY_DIR}")

#===============================================================================
# Link directories
#===============================================================================
link_directories("${CMAKE_BINARY_DIR}/lib")
link_directories("${CMAKE_INSTALL_PREFIX}/lib")
link_directories(${Boost_LIBRARY_DIRS})
if(HAVE_BULLET_COLLISION)
  link_directories(${BULLET_LIBRARY_DIRS})
endif()

#===============================================================================
# Link directories
#===============================================================================
set(KIDO_DEPENDENCIES ${CCD_LIBRARIES}
                      ${FCL_LIBRARIES}
                      ${ASSIMP_LIBRARIES}
                      ${Boost_LIBRARIES}
                      ${OPENGL_LIBRARIES})
if(HAVE_BULLET_COLLISION)
  set(KIDO_DEPENDENCIES ${KIDO_DEPENDENCIES} ${BULLET_LIBRARIES})
endif()

#===============================================================================
# Check for non-case-sensitive filesystems
#===============================================================================
execute_process(COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/tools/case_sensitive_filesystem
                RESULT_VARIABLE FILESYSTEM_CASE_SENSITIVE_RETURN)
if(${FILESYSTEM_CASE_SENSITIVE_RETURN} EQUAL 0)
  set(FILESYSTEM_CASE_SENSITIVE TRUE)
else()
  set(FILESYSTEM_CASE_SENSITIVE FALSE)
endif()

#===============================================================================
# Compiler flags
#===============================================================================
if(MSVC)
  # Visual Studio enables c++11 support by default
  if(NOT MSVC12 AND NOT MSVC14)
    message(FATAL_ERROR "${PROJECT_NAME} requires VS 2013 or greater.")
  endif()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP4")
  set(CMAKE_EXE_LINKER_FLAGS_RELEASE "/LTCG /INCREMENTAL:NO")
  if(NOT KIDO_MSVC_DEFAULT_OPTIONS)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} ${KIDO_RUNTIME_LIBRARY}d /Zi /Gy /W1 /EHsc")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${KIDO_RUNTIME_LIBRARY} /Zi /GL /Gy /W1 /EHsc /arch:SSE2")
  endif(NOT KIDO_MSVC_DEFAULT_OPTIONS)
elseif(CMAKE_COMPILER_IS_GNUCXX)
  set(CMAKE_CXX_FLAGS "-Wall -msse2 -fPIC")
  execute_process(
    COMMAND ${CMAKE_CXX_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
  set(CXX_COMPILER_VERSION ${GCC_VERSION})
  if(GCC_VERSION VERSION_LESS 4.8)
    message(FATAL_ERROR "The installed g++ version is ${GCC_VERSION}. ${PROJECT_NAME} requires g++ 4.8 or greater.")
  else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
  endif()
  set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
  set(CMAKE_CXX_FLAGS_DEBUG "-g -fno-omit-frame-pointer -fno-inline-functions -fno-inline-functions-called-once -fno-optimize-sibling-calls")
  set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELEASE} ${CMAKE_CXX_FLAGS_DEBUG}")
  set(CMAKE_CXX_FLAGS_PROFILE "${CMAKE_CXX_FLAGS_DEBUG} -pg")
elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  set(CMAKE_CXX_FLAGS "-msse2 -fPIC")
  execute_process(
    COMMAND ${CMAKE_CXX_COMPILER} -dumpversion OUTPUT_VARIABLE CLANG_VERSION)
  set(CXX_COMPILER_VERSION ${CLANG_VERSION})
  if(CLANG_VERSION VERSION_LESS 3.3)
    message(FATAL_ERROR "The installed Clang version is ${CLANG_VERSION}. ${PROJECT_NAME} requires clang 3.3 or greater.")
  else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
  endif()
  if("${CMAKE_SYSTEM_NAME}" MATCHES "Darwin")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
  endif()
  set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
  set(CMAKE_CXX_FLAGS_DEBUG "-g -fno-omit-frame-pointer -fno-inline-functions -fno-optimize-sibling-calls")
  set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELEASE} ${CMAKE_CXX_FLAGS_DEBUG}")
  set(CMAKE_CXX_FLAGS_PROFILE "${CMAKE_CXX_FLAGS_DEBUG} -pg")
else()
  message(SEND_ERROR "Compiler[${CMAKE_CXX_COMPILER_ID}] not supported.")
endif()

#===============================================================================
# Print build summary
#===============================================================================
message(STATUS "")
message(STATUS "[ Build summary ]")
message(STATUS "CMAKE_GENERATOR  : ${CMAKE_GENERATOR}")
message(STATUS "Compiler ID      : ${CMAKE_CXX_COMPILER_ID}")
message(STATUS "Compiler version : ${CXX_COMPILER_VERSION}")
message(STATUS "Build type       : ${CMAKE_BUILD_TYPE}")
message(STATUS "BUILD_SHARED_LIBS: ${BUILD_SHARED_LIBS}")
message(STATUS "ENABLE_OPENMP    : ${ENABLE_OPENMP}")
message(STATUS "Build osgKido    : ${KIDO_BUILD_OSGKIDO}")
message(STATUS "Build examples   : ${KIDO_BUILD_EXAMPLES}")
message(STATUS "Build tutorials  : ${KIDO_BUILD_TUTORIALS}")
message(STATUS "Build unit tests : ${KIDO_BUILD_UNITTESTS}")
message(STATUS "Install path     : ${CMAKE_INSTALL_PREFIX}")
message(STATUS "CXX_FLAGS        : ${CMAKE_CXX_FLAGS}")
if(${CMAKE_BUILD_TYPE_UPPERCASE} STREQUAL "RELEASE")
  message(STATUS "CXX_FLAGS_RELEASE: ${CMAKE_CXX_FLAGS_RELEASE}")
elseif(${CMAKE_BUILD_TYPE_UPPERCASE} STREQUAL "DEBUG")
  message(STATUS "CXX_FLAGS_DEBUG  : ${CMAKE_CXX_FLAGS_DEBUG}")
elseif(${CMAKE_BUILD_TYPE_UPPERCASE} STREQUAL "RELWITHDEBINFO")
  message(STATUS "CXX_FLAGS_RELWITHDEBINFO: ${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")
elseif(${CMAKE_BUILD_TYPE_UPPERCASE} STREQUAL "PROFILE")
  message(STATUS "CXX_FLAGS_PROFILE: ${CMAKE_CXX_FLAGS_PROFILE}")
endif()
message(STATUS "CMAKE_SOURCE_DIR : ${CMAKE_SOURCE_DIR}")
message(STATUS "CMAKE_BINARY_DIR : ${CMAKE_BINARY_DIR}")

#===============================================================================
# Configure files
#===============================================================================
message(STATUS "")
message(STATUS "[ Configured files ]")

# Generate the kido config file
# Ref: http://www.vtk.org/Wiki/CMake:How_To_Write_Platform_Checks
set(KIDO_CONFIG_IN ${CMAKE_SOURCE_DIR}/kido/config.hpp.in)
set(KIDO_CONFIG_OUT ${CMAKE_BINARY_DIR}/kido/config.hpp)
message(STATUS ${KIDO_CONFIG_OUT})
configure_file(${KIDO_CONFIG_IN} ${KIDO_CONFIG_OUT} @ONLY)
install(FILES ${KIDO_CONFIG_OUT} DESTINATION include/kido)

# Generate the KIDO CMake Config and version files
include(WriteBasicConfigVersionFile)
set(KIDO_CONFIG_IN ${CMAKE_SOURCE_DIR}/cmake/KIDOConfig.cmake.in)
set(KIDO_CONFIG_OUT ${CMAKE_BINARY_DIR}/cmake/KIDOConfig.cmake)
message(STATUS ${KIDO_CONFIG_OUT})
message(STATUS ${CMAKE_BINARY_DIR}/cmake/KIDOConfigVersion.cmake)
configure_file(${KIDO_CONFIG_IN} ${KIDO_CONFIG_OUT} @ONLY)
write_basic_config_version_file(${CMAKE_BINARY_DIR}/cmake/KIDOConfigVersion.cmake VERSION ${KIDO_VERSION} COMPATIBILITY SameMajorVersion)
install(FILES ${KIDO_CONFIG_OUT} ${CMAKE_BINARY_DIR}/cmake/KIDOConfigVersion.cmake DESTINATION share/kido)

# Generate the KIDO pkg-config
set(PC_CONFIG_IN ${CMAKE_SOURCE_DIR}/cmake/kido.pc.in)
set(PC_CONFIG_OUT ${CMAKE_BINARY_DIR}/cmake/kido.pc)
message(STATUS ${PC_CONFIG_OUT})
configure_file(${PC_CONFIG_IN} ${PC_CONFIG_OUT} @ONLY)
install(FILES ${PC_CONFIG_OUT} DESTINATION lib/pkgconfig)

# Install a Catkin 'package.xml' file. This is required by REP-136.
install(FILES package.xml DESTINATION share/${PROJECT_NAME})

#===============================================================================
# Add sub-directories
#===============================================================================
add_subdirectory(kido)

# Unit tests
if(KIDO_BUILD_UNITTESTS)
  enable_testing()
  add_subdirectory(unittests)
endif(KIDO_BUILD_UNITTESTS)

# Examples
if(KIDO_BUILD_EXAMPLES)
  add_subdirectory(apps)
endif(KIDO_BUILD_EXAMPLES)

if(KIDO_BUILD_TUTORIALS)
  add_subdirectory(tutorials)
endif()

#===============================================================================
# Export targets
#===============================================================================
install(EXPORT KIDOTargets DESTINATION share/kido)

# Add an "uninstall" target
# Ref: http://www.cmake.org/Wiki/CMake_FAQ#Can_I_do_.22make_uninstall.22_with_CMake.3F
configure_file("${PROJECT_SOURCE_DIR}/cmake/uninstall_target.cmake.in" "${PROJECT_BINARY_DIR}/uninstall_target.cmake" IMMEDIATE @ONLY)
add_custom_target(uninstall "${CMAKE_COMMAND}" -P "${PROJECT_BINARY_DIR}/uninstall_target.cmake")

#===============================================================================
# API Document using Doxygen
# References:
#   http://mementocodex.wordpress.com/2013/01/19/how-to-generate-code-documentation-with-doxygen-and-cmake-a-slightly-improved-approach/
#   http://www.cmake.org/pipermail/cmake/2007-February/012796.html
#===============================================================================
if(DOXYGEN_FOUND)

  set(DOXYGEN_DOXYFILE_IN  ${PROJECT_SOURCE_DIR}/doxygen/Doxyfile.in    )
  set(DOXYGEN_DOXYFILE     ${PROJECT_BINARY_DIR}/doxygen/Doxyfile       )
  set(DOXYGEN_HTML_INDEX   ${PROJECT_SOURCE_DIR}/doxygen/html/index.html)
  set(DOXYGEN_OUTPUT_ROOT  ${PROJECT_SOURCE_DIR}/doxygen/html           ) # Pasted into Doxyfile.in
  set(DOXYGEN_INPUT_ROOT   ${PROJECT_SOURCE_DIR}/kido                   ) # Pasted into Doxyfile.in
  set(DOXYGEN_EXTRA_INPUTS ${PROJECT_SOURCE_DIR}/doxygen/mainpage.dox   ) # Pasted into Doxyfile.in

  configure_file(${DOXYGEN_DOXYFILE_IN} ${DOXYGEN_DOXYFILE} @ONLY)
  file(COPY "${PROJECT_SOURCE_DIR}/doxygen/KIDO logo.png" DESTINATION ${DOXYGEN_OUTPUT_ROOT})
  add_custom_command(OUTPUT ${DOXYGEN_HTML_INDEX}
                     COMMAND ${CMAKE_COMMAND} -E echo_append "Building API Documentation..."
                     COMMAND ${DOXYGEN_EXECUTABLE} -u ${DOXYGEN_DOXYFILE}
                     COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_DOXYFILE}
                     COMMAND ${CMAKE_COMMAND} -E echo "Done."
                     WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/doxygen
                     DEPENDS ${DOXYGEN_DOXYFILE}
  )
  # add_custom_target(docs ALL DEPENDS ${DOXYGEN_HTML_INDEX})
  add_custom_target(docs DEPENDS ${DOXYGEN_HTML_INDEX})
  add_custom_target(docs_forced
                    COMMAND ${CMAKE_COMMAND} -E echo_append "Building API Documentation..."
                    COMMAND ${DOXYGEN_EXECUTABLE} -u ${DOXYGEN_DOXYFILE}
                    COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_DOXYFILE}
                    COMMAND ${CMAKE_COMMAND} -E echo "Done."
                    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/doxygen
  )

endif()

#===============================================================================
# Coloring build outputs using gccfilter if appliciable
# Ref: http://stackoverflow.com/questions/14399984/make-cmake-use-gccfilter
#===============================================================================
if("${PERLMODULES_FOUND}" STREQUAL "TRUE")
  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_IS_GNUCPP)
    option(COLOR_GCC "Use GCCFilter to color compiler output messages" ON)
    set(COLOR_GCC_OPTIONS "-c -r -w" CACHE STRING "Arguments that are passed to gccfilter when output coloring is switchend on. Defaults to -c -r -w.")
    if(COLOR_GCC)
      set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${PROJECT_SOURCE_DIR}/tools/gccfilter ${COLOR_GCC_OPTIONS}")
    endif()
  endif()
endif()

#===============================================================================
# END
#===============================================================================
message(STATUS "")
