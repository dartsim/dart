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

include(${CMAKE_MODULE_PATH}/DARTMacros.cmake)

# System install paths on Windows
if(WIN32)
  set(CMAKE_INSTALL_PREFIX "C:/Golems" CACHE PATH "Install prefix" FORCE)
endif()

#===============================================================================
# Project settings
#===============================================================================
#
# If you change the version, please update the <version> tag in package.xml.

project(dart)

set(DART_MAJOR_VERSION "6")
set(DART_MINOR_VERSION "0")
set(DART_PATCH_VERSION "0")
set(DART_VERSION "${DART_MAJOR_VERSION}.${DART_MINOR_VERSION}.${DART_PATCH_VERSION}")
set(DART_PKG_DESC "Dynamic Animation and Robotics Toolkit.")
set(DART_PKG_EXTERNAL_DEPS "flann, ccd, fcl")

#===============================================================================
# Print intro
#===============================================================================
message(STATUS "")
message(STATUS "============================================")
message(STATUS "                DART ${DART_VERSION}")
message(STATUS "============================================")

#===============================================================================
# Build options
#===============================================================================
option(ENABLE_OPENMP "Build with OpenMP parallaization enabled" ON)
option(BUILD_CORE_ONLY "Build only the core of DART" OFF)
if(MSVC)
  set(DART_RUNTIME_LIBRARY "/MD" CACHE STRING "BaseName chosen by the user at CMake configure time")
  set_property(CACHE DART_RUNTIME_LIBRARY PROPERTY STRINGS /MD /MT)
  option(DART_MSVC_DEFAULT_OPTIONS "Build DART with default Visual Studio options" OFF)
else()
  option(BUILD_SHARED_LIBS "Build shared libraries" ON)
endif()
option(DART_BUILD_GUI_OSG "Build osgDart library" ON)
option(DART_COVERALLS "Turn on coveralls support" OFF)
option(DART_COVERALLS_UPLOAD "Upload the generated coveralls json" ON)

if(DART_COVERALLS)
  include(Coveralls)
  coveralls_turn_on_coverage()
endif()

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
  message(SEND_ERROR "Looking for EIGEN3 - NOT found, please install libeigen3-dev (>= 3.0.5)")
endif()

# CCD
find_package(CCD 1.4.0 QUIET)
if(CCD_FOUND)
  message(STATUS "Looking for CCD - ${CCD_VERSION} found")
else()
  message(SEND_ERROR "Looking for CCD - NOT found, please install libccd-dev (>= 1.4.0)")
endif()

# FCL
# TODO: We need to increate the minimum required fcl version to 0.4.0 for DART
# 5.0 once fcl 0.4.0 is released.
find_package(FCL 0.2.9 QUIET)
if(FCL_FOUND)
  message(STATUS "Looking for FCL - ${FCL_VERSION} found")
  if(FCL_VERSION)
    if("${FCL_VERSION}" VERSION_LESS "0.4.0")
      message(WARNING
        "Found FCL version ${FCL_VERSION}, which is older than 0.4.0. FCL"
        " versions predating 0.4.0 return incorrect contacts for"
        " primitive-primitive collsions. You should use FCLMeshCollisionDetector"
        " instead of FCLCollisionDetector if you need accurate contacts."
      )
      set(DART_USE_FCLMESHCOLLISIONDETECTOR TRUE)
    else()
      set(DART_USE_FCLMESHCOLLISIONDETECTOR FALSE)
    endif()
  endif()
else()
  message(SEND_ERROR "Looking for FCL - NOT found, please install libfcl-dev (>= 0.2.9)")
endif()

# ASSIMP
find_package(ASSIMP 3.0.0 QUIET)
if(ASSIMP_FOUND)
  message(STATUS "Looking for ASSIMP - ${ASSIMP_VERSION} found")

  # Check for missing symbols in ASSIMP (see #451)
  include(CheckCXXSourceCompiles)
  set(CMAKE_REQUIRED_DEFINITIONS "")
  set(CMAKE_REQUIRED_FLAGS "")
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
                    "aiScene. DART will use its own implementations of these "
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
                    "aiMaterial. DART will use its own implementations of "
                    "these functions. We recommend using a version of ASSIMP "
                    "that does not have this issue, once one becomes available.")
  endif(NOT ASSIMP_AIMATERIAL_CTOR_DTOR_DEFINED)

  unset(CMAKE_REQUIRED_INCLUDES)
  unset(CMAKE_REQUIRED_LIBRARIES)

else()
  message(SEND_ERROR "Looking for ASSIMP - NOT found, please install libassimp-dev (>= 3.0.0)")
endif()

# OpenGL
find_package(OpenGL QUIET)
if(OPENGL_FOUND)
  message(STATUS "Looking for OpenGL - found")
else()
  message(SEND_ERROR "Looking for OpenGL - NOT found, please install OpenGL")
endif()

# Boost
set(DART_MIN_BOOST_VERSION 1.46.0 CACHE INTERNAL "Boost min version requirement" FORCE)
if(MSVC)
  add_definitions(-DBOOST_ALL_NO_LIB)
endif()
add_definitions(-DBOOST_TEST_DYN_LINK)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost ${DART_MIN_BOOST_VERSION} COMPONENTS regex system QUIET)
if(Boost_FOUND)
  message(STATUS "Looking for Boost - ${Boost_MAJOR_VERSION}.${Boost_MINOR_VERSION}.${Boost_SUBMINOR_VERSION} found")
else()
  message(SEND_ERROR "Please install system boost version ${DART_MIN_BOOST_VERSION} or higher.")
endif()

if(NOT BUILD_CORE_ONLY)

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
      message(SEND_ERROR "Looking for GLUT - NOT found, Please install freeglut3-dev")
    endif()
  endif()

  # FLANN
  find_package(FLANN 1.8.4 QUIET)
  if(FLANN_FOUND)
    message(STATUS "Looking for FLANN - ${FLANN_VERSION} found")
  else()
    message(SEND_ERROR "Looking for FLANN - NOT found, please install libflann-dev (>= 1.8.4)")
  endif()

  # TINYXML
  find_package(TINYXML 2.6.2 QUIET)
  if(TINYXML_FOUND)
    message(STATUS "Looking for TINYXML - ${TINYXML_VERSION} found")
  else()
    message(SEND_ERROR "Looking for TINYXML - NOT found, please install libtinyxml-dev (>= 2.6.2)")
  endif()

  # TINYXML2
  find_package(TINYXML2 QUIET)
  if(TINYXML2_FOUND)
    message(STATUS "Looking for TINYXML2 - ${TINYXML2_VERSION} found")
  else()
    message(SEND_ERROR "Looking for TINYXML2 - NOT found, please install libtinyxml2-dev (>= 1.0.1)")
  endif()

  # urdfdom
  find_package(urdfdom QUIET)
  if(urdfdom_FOUND)
    message(STATUS "Looking for urdfdom - found")
  else()
    message(SEND_ERROR "Looking for urdfdom - NOT found, please install liburdfdom-dev")
  endif()
  if(MSVC)
    set(urdfdom_LIBRARIES optimized urdfdom_sensor      debug urdfdom_sensord
                          optimized urdfdom_model_state debug urdfdom_model_stated
                          optimized urdfdom_model       debug urdfdom_modeld
                          optimized urdfdom_world       debug urdfdom_worldd
                          optimized console_bridge      debug console_bridged)
  endif()

endif()

#-----------------------
# Optional dependencies
#-----------------------
message(STATUS "")
message(STATUS "[ Optional dependencies ]")

# OpenSceneGraph
if(DART_BUILD_GUI_OSG)

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

else()

  message(STATUS "Skipping OpenSceneGraph (DART_BUILD_GUI_OSG == ${DART_BUILD_GUI_OSG})")
  set(HAVE_OPENSCENEGRAPH FALSE)

endif(DART_BUILD_GUI_OSG)

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
  message(STATUS "Looking for NLOPT - NOT found, please install libnlopt-dev (>= 2.4.1)")
  set(HAVE_NLOPT FALSE)
endif()

# IPOPT
find_package(IPOPT 3.11.4 QUIET)
if(IPOPT_FOUND)
  message(STATUS "Looking for IPOPT - ${IPOPT_VERSION} found")
  set(HAVE_IPOPT TRUE)
else()
  message(STATUS "Looking for IPOPT - NOT found, please install coinor-libipopt-dev (>= 3.11.4)")
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

# Bullet. Force MODULE mode to use the FindBullet.cmake file distributed with
# CMake. Otherwise, we may end up using the BulletConfig.cmake file distributed
# with Bullet, which uses relative paths and may break transitive dependencies.
find_package(Bullet COMPONENTS BulletMath BulletCollision MODULE QUIET)

if(BULLET_FOUND)
  # Test whether Bullet was built with double precision. If so, we need to
  # define the BT_USE_DOUBLE_PRECISION pre-processor directive before including
  # any Bullet headers. This is a workaround for the fact that Bullet does not
  # add the definition to BULLET_DEFINITIONS or generate a #cmakedefine header.
  include(CheckCXXSourceCompiles)
  set(CMAKE_REQUIRED_FLAGS "")
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

  if(BT_USE_DOUBLE_PRECISION)
    message(STATUS "Looking for Bullet - found (double precision)")
  else()
    message(STATUS "Looking for Bullet - found (single precision)")
  endif()

  set(HAVE_BULLET_COLLISION TRUE)
else()
  message(STATUS "Looking for Bullet - NOT found, please install libbullet-dev")
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
# DART dependency variable settings
#===============================================================================
set(DART_CORE_DEPENDENCIES ${CCD_LIBRARIES}
                           ${FCL_LIBRARIES}
                           ${ASSIMP_LIBRARIES}
                           ${Boost_LIBRARIES}
                           ${OPENGL_LIBRARIES}
                           ${GLUT_LIBRARY}
)

if(HAVE_BULLET_COLLISION)
  set(DART_CORE_DEPENDENCIES ${DART_CORE_DEPENDENCIES} ${BULLET_LIBRARIES})
endif()

if(NOT BUILD_CORE_ONLY)
  set(DART_DEPENDENCIES ${urdfdom_LIBRARIES}
                        ${TINYXML_LIBRARIES}
                        ${TINYXML2_LIBRARIES}
  )

  if(HAVE_BULLET_COLLISION)
    set(DART_DEPENDENCIES ${DART_DEPENDENCIES} ${OPENSCENEGRAPH_LIBRARIES})
  endif()

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
include_directories(SYSTEM ${GLUT_INCLUDE_DIR})
if(HAVE_BULLET_COLLISION)
  include_directories(SYSTEM ${BULLET_INCLUDE_DIRS})
endif()

if(NOT BUILD_CORE_ONLY)
  include_directories(SYSTEM ${FLANN_INCLUDE_DIRS})
  include_directories(SYSTEM ${urdfdom_INCLUDE_DIRS})
  include_directories(SYSTEM ${TINYXML_INCLUDE_DIRS})
  include_directories(SYSTEM ${TINYXML2_INCLUDE_DIRS})
  if(HAVE_NLOPT)
    include_directories(SYSTEM ${NLOPT_INCLUDE_DIRS})
  endif()
  if(HAVE_IPOPT)
    include_directories(SYSTEM ${IPOPT_INCLUDE_DIRS})
  endif()
  if(HAVE_SHARK)
    include_directories(SYSTEM ${SHARK_INCLUDE_DIRS})
  endif()
endif()

include_directories("${CMAKE_BINARY_DIR}")

#===============================================================================
# Link directories
#===============================================================================
link_directories("${CMAKE_BINARY_DIR}/lib")
link_directories("${CMAKE_INSTALL_PREFIX}/lib")
link_directories(${Boost_LIBRARY_DIRS})

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
  if(MSVC_VERSION VERSION_LESS 1900)
    message(FATAL_ERROR "${PROJECT_NAME} requires VS 2015 or greater.")
  endif()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP4")
  set(CMAKE_EXE_LINKER_FLAGS_RELEASE "/LTCG /INCREMENTAL:NO")
  if(NOT DART_MSVC_DEFAULT_OPTIONS)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} ${DART_RUNTIME_LIBRARY}d /Zi /Gy /W1 /EHsc")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${DART_RUNTIME_LIBRARY} /Zi /GL /Gy /W1 /EHsc /arch:SSE2")
  endif(NOT DART_MSVC_DEFAULT_OPTIONS)
elseif(CMAKE_COMPILER_IS_GNUCXX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -msse2 -fPIC")
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
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse2 -fPIC")
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
message(STATUS "Build core only  : ${BUILD_CORE_ONLY}")
message(STATUS "Build gui::osg   : ${DART_BUILD_GUI_OSG}")
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

# Generate the dart config file
# Ref: http://www.vtk.org/Wiki/CMake:How_To_Write_Platform_Checks
set(DART_CONFIG_IN ${CMAKE_SOURCE_DIR}/dart/config.h.in)
set(DART_CONFIG_OUT ${CMAKE_BINARY_DIR}/dart/config.h)
message(STATUS ${DART_CONFIG_OUT})
configure_file(${DART_CONFIG_IN} ${DART_CONFIG_OUT} @ONLY)
install(FILES ${DART_CONFIG_OUT} DESTINATION include/dart)

# Generate the DART CMake Config and version files
include(WriteBasicConfigVersionFile)
set(DART_CORE_CONFIG_IN ${CMAKE_SOURCE_DIR}/cmake/DARTCoreConfig.cmake.in)
set(DART_CORE_CONFIG_OUT ${CMAKE_BINARY_DIR}/cmake/DARTCoreConfig.cmake)
message(STATUS ${DART_CORE_CONFIG_OUT})
message(STATUS ${CMAKE_BINARY_DIR}/cmake/DARTConfigVersion.cmake)
configure_file(${DART_CORE_CONFIG_IN} ${DART_CORE_CONFIG_OUT} @ONLY)
write_basic_config_version_file(cmake/DARTCoreConfigVersion.cmake VERSION ${DART_VERSION} COMPATIBILITY SameMajorVersion)
install(FILES ${DART_CORE_CONFIG_OUT} ${CMAKE_BINARY_DIR}/cmake/DARTCoreConfigVersion.cmake DESTINATION share/dartcore)
if(NOT BUILD_CORE_ONLY)
  set(DART_CONFIG_IN ${CMAKE_SOURCE_DIR}/cmake/DARTConfig.cmake.in)
  set(DART_CONFIG_OUT ${CMAKE_BINARY_DIR}/cmake/DARTConfig.cmake)
  message(STATUS ${DART_CONFIG_OUT})
  message(STATUS ${CMAKE_BINARY_DIR}/cmake/DARTConfigVersion.cmake)
  configure_file(${DART_CONFIG_IN} ${DART_CONFIG_OUT} @ONLY)
  write_basic_config_version_file(cmake/DARTConfigVersion.cmake VERSION ${DART_VERSION} COMPATIBILITY SameMajorVersion)
  install(FILES ${DART_CONFIG_OUT} ${CMAKE_BINARY_DIR}/cmake/DARTConfigVersion.cmake DESTINATION share/dart)
endif()

# Generate the DART pkg-config
set(PC_CONFIG_IN ${CMAKE_SOURCE_DIR}/cmake/dart.pc.in)
set(PC_CONFIG_OUT ${CMAKE_BINARY_DIR}/cmake/dart.pc)
message(STATUS ${PC_CONFIG_OUT})
configure_file(${PC_CONFIG_IN} ${PC_CONFIG_OUT} @ONLY)
install(FILES ${PC_CONFIG_OUT} DESTINATION lib/pkgconfig)

# Install a Catkin 'package.xml' file. This is required by REP-136.
install(FILES package.xml DESTINATION share/${PROJECT_NAME})

#===============================================================================
# Helper Functions
#===============================================================================

set_property(GLOBAL PROPERTY DART_EXAMPLES)
set_property(GLOBAL PROPERTY DART_TUTORIALS)
set_property(GLOBAL PROPERTY DART_UNITTESTS)

function(dart_add_example target_name)
  set_property(GLOBAL APPEND PROPERTY DART_EXAMPLES "${target_name}")
endfunction(dart_add_example)

function(dart_add_tutorial target_name)
  set_property(GLOBAL APPEND PROPERTY DART_TUTORIALS "${target_name}")
endfunction(dart_add_tutorial)

function(dart_add_unittest target_name)
  set_property(GLOBAL APPEND PROPERTY DART_UNITTESTS "${target_name}")
endfunction(dart_add_unittest)

#===============================================================================
# Add sub-directories
#===============================================================================

add_subdirectory(dart)

if(NOT BUILD_CORE_ONLY)

  add_subdirectory(apps EXCLUDE_FROM_ALL)
  add_subdirectory(tutorials EXCLUDE_FROM_ALL)

  # Add a "tutorials" target to build tutorials.
  get_property(tutorials GLOBAL PROPERTY DART_TUTORIALS)
  add_custom_target(tutorials DEPENDS ${tutorials})

  message(STATUS "")
  message(STATUS "[ Tutorials ]")
  foreach(tutorial ${tutorials})
    message(STATUS "Adding tutorial: ${tutorial}")
  endforeach(tutorial ${tutorials})

  # Add an "examples" target to build examples.
  get_property(examples GLOBAL PROPERTY DART_EXAMPLES)
  add_custom_target(examples DEPENDS ${examples})

  message(STATUS "")
  message(STATUS "[ Examples ]")

  foreach(example ${examples})
    message(STATUS "Adding example: ${example}")
  endforeach(example ${examples})
endif(NOT BUILD_CORE_ONLY)

# Add a "tests" target to build unit tests.
enable_testing()
add_subdirectory(unittests EXCLUDE_FROM_ALL)

get_property(unittests GLOBAL PROPERTY DART_UNITTESTS)
add_custom_target(tests DEPENDS ${unittests})

message(STATUS "")
message(STATUS "[ Unit Tests ]")
foreach(unittest ${unittests})
  message(STATUS "Adding test: ${unittest}")
endforeach(unittest ${unittests})

#===============================================================================
# Export targets
#===============================================================================
install(EXPORT DARTCoreTargets DESTINATION share/dartcore)
if(NOT BUILD_CORE_ONLY)
  install(EXPORT DARTTargets DESTINATION share/dart)
endif()

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
  set(DOXYGEN_INPUT_ROOT   ${PROJECT_SOURCE_DIR}/dart                   ) # Pasted into Doxyfile.in
  set(DOXYGEN_EXTRA_INPUTS ${PROJECT_SOURCE_DIR}/doxygen/mainpage.dox   ) # Pasted into Doxyfile.in

  configure_file(${DOXYGEN_DOXYFILE_IN} ${DOXYGEN_DOXYFILE} @ONLY)
  file(COPY "${PROJECT_SOURCE_DIR}/doxygen/DART logo.png" DESTINATION ${DOXYGEN_OUTPUT_ROOT})
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
