cmake_minimum_required(VERSION 2.8)
set(CMAKE_LEGACY_CYGWIN_WIN32 0)
set(CMAKE_CXX_WARNING_LEVEL 4)
# Disables a warning about a change in Cygwin Cmake

project(dart)
message(STATUS "Generate makefile/project file for DART")

set(DART_MAJOR_VERSION "3")
set(DART_MINOR_VERSION "0")
set(DART_PATCH_VERSION "0")
set(PKG_DESC "Dynamic Animation and Robotics Toolkit.")
set(DART_VERSION "${DART_MAJOR_VERSION}.${DART_MINOR_VERSION}.${DART_PATCH_VERSION}")
set(PKG_EXTERNAL_DEPS "flann, ccd, fcl")

message(STATUS "CMAKE_SOURCE_DIR = ${CMAKE_SOURCE_DIR}")
message(STATUS "CMAKE_BINARY_DIR = ${CMAKE_BINARY_DIR}")

option(BUILD_SNOPT "Build with SNOPT optimizer" OFF)
option(BUILD_CORE_ONLY "Build only the core of DART" OFF)
if(NOT MSVC)
    OPTION(BUILD_SHARED_LIBS "Build shared libraries" ON)
endif()

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo Profile."
      FORCE)
ENDIF(NOT CMAKE_BUILD_TYPE)

# GCCFilter, if appliciable
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_IS_GNUCPP)
   option(COLOR_GCC "Use GCCFilter to color compiler output messages" OFF)
   set(COLOR_GCC_OPTIONS "-c -r -w" CACHE STRING "Arguments that are passed to gccfilter when output coloring is switchend on. Defaults to -c -r -w.")
   if(COLOR_GCC)
      set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "gccfilter ${COLOR_GCC_OPTIONS}")
   endif()
endif()

if(MSVC)
  message(STATUS "Setup Visual Studio Specific Flags")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MDd /Zi /Gy /W1 /EHsc")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MD /Zi /GL /Gy /W1 /EHsc /arch:SSE2 /openmp")
  set(CMAKE_EXE_LINKER_FLAGS_RELEASE "/LTCG")
elseif(CMAKE_COMPILER_IS_GNUCXX OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  set(CMAKE_CXX_FLAGS "-msse2 -fPIC")
  set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
  set(CMAKE_CXX_FLAGS_DEBUG "-g -fno-omit-frame-pointer -fno-inline-functions -fno-inline-functions-called-once -fno-optimize-sibling-calls")
  set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELEASE} ${CMAKE_CXX_FLAGS_DEBUG}")
  set(CMAKE_CXX_FLAGS_PROFILE "${CMAKE_CXX_FLAGS_DEBUG} -pg")
else()
  message(SEND_ERROR "Compiler not supported.")
endif()

set(CMAKE_DEBUG_POSTFIX "d")

# System Install paths
if(WIN32 AND CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX "C:/Golems" CACHE PATH "Install prefix" FORCE)
endif()

###############
# Generate the path header file
set(PATH_HEADER_IN ${CMAKE_SOURCE_DIR}/dart/utils/Paths.h.in)
set(PATH_HEADER_OUT ${CMAKE_BINARY_DIR}/dart/utils/Paths.h)
message(STATUS "Generate the path header file to ${PATH_HEADER_OUT}")
configure_file(${PATH_HEADER_IN} ${PATH_HEADER_OUT} @ONLY)
install(FILES ${PATH_HEADER_OUT} DESTINATION include/dart/utils)

###############
# Find dependencies
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

find_package(Eigen REQUIRED)
find_package(FCL REQUIRED)
find_package(Assimp REQUIRED)
find_package(OpenGL REQUIRED)

# GLUT
if(WIN32 AND NOT CYGWIN)
    set(GLUT_INCLUDE_DIR "@CMAKE_INSTALL_PREFIX@/include")
    set(GLUT_LIBRARIES glut32)
else()
    find_package(GLUT REQUIRED)
    set(GLUT_LIBRARIES ${GLUT_glut_LIBRARY})
endif()

# Boost
find_package(Boost REQUIRED system)
if(MSVC OR MSVC90 OR MSVC10)
    add_definitions(-DBOOST_ALL_NO_LIB)
endif()
add_definitions(-DBOOST_TEST_DYN_LINK)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)

if(NOT BUILD_CORE_ONLY)
  find_package(FLANN REQUIRED)
  find_package(TinyXML REQUIRED)
  find_package(TinyXML2 REQUIRED)
  
  # urdfdom
  find_package(urdfdom REQUIRED)
  if(MSVC)
      set(urdfdom_LIBRARIES optimized urdfdom_sensor      debug urdfdom_sensord
                            optimized urdfdom_model_state debug urdfdom_model_stated
                            optimized urdfdom_model       debug urdfdom_modeld
                            optimized urdfdom_world       debug urdfdom_worldd
                            optimized console_bridge      debug console_bridged)
  endif()
endif()

include_directories(BEFORE ${PROJECT_SOURCE_DIR})
include_directories(${Eigen_INCLUDE_DIRS})
include_directories(${FCL_INCLUDE_DIRS})
include_directories(${Assimp_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${OPENGL_INCLUDE_DIR})
include_directories(${GLUT_INCLUDE_DIR})

if(NOT BUILD_CORE_ONLY)
  include_directories(${FLANN_INCLUDE_DIRS})
  include_directories(${urdfdom_INCLUDE_DIRS})
  include_directories(${TinyXML_INCLUDE_DIRS})
  include_directories(${TinyXML2_INCLUDE_DIRS})
endif()

include_directories("${CMAKE_BINARY_DIR}")

link_directories("${CMAKE_BINARY_DIR}/lib")
link_directories("${CMAKE_INSTALL_PREFIX}/lib")
link_directories(${Boost_LIBRARY_DIRS})

set(DART_CORE_DEPENDENCIES ${FCL_LIBRARIES} ${Assimp_LIBRARIES} ${Boost_LIBRARIES} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})
if(NOT BUILD_CORE_ONLY)
  set(DART_DEPENDENCIES ${urdfdom_LIBRARIES} ${TinyXML_LIBRARIES} ${TinyXML2_LIBRARIES})
endif()

###############
# Generate the Dart CMake Config file
configure_file(cmake/DARTCoreConfig.cmake.in "${CMAKE_BINARY_DIR}/cmake/DARTCoreConfig.cmake" @ONLY)
install(FILES ${CMAKE_BINARY_DIR}/cmake/DARTCoreConfig.cmake DESTINATION share/dartcore)
if(NOT BUILD_CORE_ONLY)
  configure_file(cmake/DARTConfig.cmake.in "${CMAKE_BINARY_DIR}/cmake/DARTConfig.cmake" @ONLY)
  install(FILES ${CMAKE_BINARY_DIR}/cmake/DARTConfig.cmake DESTINATION share/dart)
endif()

set(PC_CONFIG_IN ${CMAKE_SOURCE_DIR}/cmake/dart.pc.in)
set(PC_CONFIG_OUT ${CMAKE_BINARY_DIR}/cmake/dart.pc)
configure_file(${PC_CONFIG_IN} ${PC_CONFIG_OUT} @only)
install(FILES ${PC_CONFIG_OUT} DESTINATION lib/pkgconfig)

###############
add_subdirectory(dart)
if(NOT BUILD_CORE_ONLY)
  enable_testing()
  add_subdirectory(unittests)
  add_subdirectory(apps)
endif()

install(EXPORT DARTCoreTargets DESTINATION share/dartcore)
if(NOT BUILD_CORE_ONLY)
  install(EXPORT DARTTargets DESTINATION share/dart)
endif()

###############
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
# Package Installer
set(CPACK_PACKAGE_NAME "dart")
set(CPACK_SYSTEM_NAME "i386")
if( CMAKE_SIZEOF_VOID_P EQUAL 8 )
    set(CPACK_SYSTEM_NAME "amd64")
endif()
set(CPACK_PACKAGE_VERSION_MAJOR ${DART_MAJOR_VERSION})
set(CPACK_PACKAGE_VERSION_MINOR ${DART_MINOR_VERSION})
set(CPACK_PACKAGE_VERSION_PATCH ${DART_PATCH_VERSION})
set(CPACK_DEBIAN_PACKAGE_DEPENDS "freeglut3, freeglut3-dev, libboost1.46-all-dev, libccd (>=1.4.2), fcl (>=0.2.7), libeigen3-dev, libxmu-dev, libxi-dev, libgtest-dev, libtinyxml-dev, libtinyxml2-dev, libassimp3, libassimp-dev (>=3.0~dfsg-1), urdfdom")

set(CPACK_PACKAGE_VERSION "${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH}")
set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}_${CPACK_PACKAGE_VERSION}_${CPACK_SYSTEM_NAME}")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Karen Liu (www.cc.gatech.edu/~karenliu/)")

set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "${PKG_DESC}")
set(CPACK_PACKAGE_VENDOR "Computer Graphics Lab at GT")
set(CPACK_PACKAGING_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX})

include(CPack)
