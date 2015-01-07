cmake_minimum_required(VERSION 2.8)
set(CMAKE_LEGACY_CYGWIN_WIN32 0)
set(CMAKE_CXX_WARNING_LEVEL 4)
# Disables a warning about a change in Cygwin Cmake

project(dart)
message(STATUS "Generate makefile/project file for DART")

set(DART_MAJOR_VERSION "4")
set(DART_MINOR_VERSION "2")
set(DART_PATCH_VERSION "1")
set(PKG_DESC "Dynamic Animation and Robotics Toolkit.")
set(DART_VERSION "${DART_MAJOR_VERSION}.${DART_MINOR_VERSION}.${DART_PATCH_VERSION}")
set(PKG_EXTERNAL_DEPS "flann, ccd, fcl")

message(STATUS "CMAKE_SOURCE_DIR = ${CMAKE_SOURCE_DIR}")
message(STATUS "CMAKE_BINARY_DIR = ${CMAKE_BINARY_DIR}")

option(ENABLE_OPENMP "Build with OpenMP parallaization enabled" ON)
option(BUILD_CORE_ONLY "Build only the core of DART" OFF)
if(MSVC)
  set(RUNTIME_LIBRARY "/MD" CACHE STRING "BaseName chosen by the user at CMake configure time")
  set_property(CACHE RUNTIME_LIBRARY PROPERTY STRINGS /MD /MT)
else()
  option(BUILD_SHARED_LIBS "Build shared libraries" ON)
endif()

###############
message(STATUS "Checking dart build type")
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo Profile."
      FORCE)
endif(NOT CMAKE_BUILD_TYPE)
string(TOUPPER ${CMAKE_BUILD_TYPE} CMAKE_BUILD_TYPE_UPPERCASE)

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
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} ${RUNTIME_LIBRARY}d /Zi /Gy /W1 /EHsc")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${RUNTIME_LIBRARY} /Zi /GL /Gy /W1 /EHsc /arch:SSE2")
  set(CMAKE_EXE_LINKER_FLAGS_RELEASE "/LTCG")
elseif(CMAKE_COMPILER_IS_GNUCXX)
  set(CMAKE_CXX_FLAGS "-Wall -msse2 -fPIC")
  set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
  set(CMAKE_CXX_FLAGS_DEBUG "-g -fno-omit-frame-pointer -fno-inline-functions -fno-inline-functions-called-once -fno-optimize-sibling-calls")
  set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELEASE} ${CMAKE_CXX_FLAGS_DEBUG}")
  set(CMAKE_CXX_FLAGS_PROFILE "${CMAKE_CXX_FLAGS_DEBUG} -pg")
elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  set(CMAKE_CXX_FLAGS "-msse2 -fPIC")
  set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
  set(CMAKE_CXX_FLAGS_DEBUG "-g -fno-omit-frame-pointer -fno-inline-functions -fno-optimize-sibling-calls")
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
# Find dependencies
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

find_package(Eigen REQUIRED)
find_package(FCL REQUIRED)
find_package(Assimp REQUIRED)
find_package(OpenGL REQUIRED)

# NLOPT
find_package(NLOPT)
if(NLOPT_FOUND)
  message(STATUS "Looking for NLOPT - found")
  set(HAVE_NLOPT TRUE)
else(NLOPT_FOUND)
  message(WARNING "NLOPT not found")
  set(HAVE_NLOPT FALSE)
endif()

# IPOPT
find_package(IPOPT)
if(IPOPT_FOUND)
  message(STATUS "Looking for IPOPT - found")
  set(HAVE_IPOPT TRUE)
else(IPOPT_FOUND)
  message(WARNING "IPOPT not found")
  set(HAVE_IPOPT FALSE)
endif()

# Shark
find_package(Shark)
if(SHARK_FOUND)
  message(STATUS "Looking for SHARK - found")
  message(STATUS "SHARK_INCLUDE_DIRS: ${SHARK_INCLUDE_DIRS}")
  message(STATUS "SHARK_LIBRARIES: ${SHARK_LIBRARIES}")
  set(HAVE_SHARK TRUE)
else(SHARK_FOUND)
  message(WARNING "SHARK not found")
  set(HAVE_SHARK FALSE)
endif()

# BulletCollision
if(UNIX)
  pkg_check_modules(BULLET bullet>=2.82)
  if (NOT BULLET_FOUND)
     pkg_check_modules(BULLET bullet2.82>=2.82)
  endif()
else()
  find_package(Bullet COMPONENTS BulletMath BulletCollision)
endif()
if(BULLET_FOUND)
  message(STATUS "Looking for BulletCollision - found")
  set(HAVE_BULLET_COLLISION TRUE)
else(BULLET_FOUND)
  message(WARNING "BulletCollision not found, for bullet collision detector option, please install libbullet-dev.")
  set(HAVE_BULLET_COLLISION FALSE)
endif()

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

# OpenMP
if(ENABLE_OPENMP)
  find_package(OpenMP)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

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
if(HAVE_BULLET_COLLISION)
  include_directories(${BULLET_INCLUDE_DIRS})
  add_definitions(${BULLET_CFLAGS})
endif()

if(NOT BUILD_CORE_ONLY)
  include_directories(${FLANN_INCLUDE_DIRS})
  include_directories(${urdfdom_INCLUDE_DIRS})
  include_directories(${TinyXML_INCLUDE_DIRS})
  include_directories(${TinyXML2_INCLUDE_DIRS})
  if(HAVE_NLOPT)
    include_directories(${NLOPT_INCLUDE_DIRS})
  endif()
  if(HAVE_IPOPT)
    include_directories(${IPOPT_INCLUDE_DIRS})
  endif()
endif()

include_directories("${CMAKE_BINARY_DIR}")

link_directories("${CMAKE_BINARY_DIR}/lib")
link_directories("${CMAKE_INSTALL_PREFIX}/lib")
link_directories(${Boost_LIBRARY_DIRS})
if(HAVE_BULLET_COLLISION)
  link_directories("${BULLET_LIBRARY_DIRS}")
endif()

set(DART_CORE_DEPENDENCIES ${FCL_LIBRARIES}
                           ${Assimp_LIBRARIES}
                           ${Boost_LIBRARIES}
                           ${OPENGL_LIBRARIES}
                           ${GLUT_LIBRARY})

if(HAVE_BULLET_COLLISION)
  set(DART_CORE_DEPENDENCIES ${DART_CORE_DEPENDENCIES} ${BULLET_LIBRARIES})
endif()

if(NOT BUILD_CORE_ONLY)
  set(DART_DEPENDENCIES ${urdfdom_LIBRARIES}
                        ${TinyXML_LIBRARIES}
                        ${TinyXML2_LIBRARIES})
  if(HAVE_NLOPT)
    set(DART_DEPENDENCIES ${DART_DEPENDENCIES} ${NLOPT_LIBRARIES})
  endif()
  if(HAVE_IPOPT)
    set(DART_DEPENDENCIES ${DART_DEPENDENCIES} ${IPOPT_LIBRARIES})
  endif()
endif()

###############
# Generate the dart config file (reference: http://www.vtk.org/Wiki/CMake:How_To_Write_Platform_Checks)
set(DART_CONFIG_IN cmake/config.h.in)
set(DART_CONFIG_OUT ${CMAKE_BINARY_DIR}/dart/config.h)
message(STATUS "Generate the config file to ${DART_CONFIG_OUT}")
configure_file(${DART_CONFIG_IN} ${DART_CONFIG_OUT} @ONLY)
install(FILES ${DART_CONFIG_OUT} DESTINATION include/dart)

###############
# Generate the path header file
set(PATH_HEADER_IN ${CMAKE_SOURCE_DIR}/dart/utils/Paths.h.in)
set(PATH_HEADER_OUT ${CMAKE_BINARY_DIR}/dart/utils/Paths.h)
message(STATUS "Generate the path header file to ${PATH_HEADER_OUT}")
configure_file(${PATH_HEADER_IN} ${PATH_HEADER_OUT} @ONLY)
install(FILES ${PATH_HEADER_OUT} DESTINATION include/dart/utils)

###############
# Generate the Dart CMake Config and version files
include(WriteBasicConfigVersionFile)
configure_file(cmake/DARTCoreConfig.cmake.in "${CMAKE_BINARY_DIR}/cmake/DARTCoreConfig.cmake" @ONLY)
write_basic_config_version_file(cmake/DARTCoreConfigVersion.cmake
  VERSION ${DART_VERSION}
  COMPATIBILITY SameMajorVersion
)
install(FILES
        ${CMAKE_BINARY_DIR}/cmake/DARTCoreConfig.cmake
        ${CMAKE_BINARY_DIR}/cmake/DARTCoreConfigVersion.cmake
        DESTINATION share/dartcore)
if(NOT BUILD_CORE_ONLY)
  configure_file(cmake/DARTConfig.cmake.in "${CMAKE_BINARY_DIR}/cmake/DARTConfig.cmake" @ONLY)
  write_basic_config_version_file(cmake/DARTConfigVersion.cmake
    VERSION ${DART_VERSION}
    COMPATIBILITY SameMajorVersion
  )
  install(FILES
          ${CMAKE_BINARY_DIR}/cmake/DARTConfig.cmake
          ${CMAKE_BINARY_DIR}/cmake/DARTConfigVersion.cmake
          DESTINATION share/dart)
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
set(CPACK_DEBIAN_PACKAGE_DEPENDS
    "freeglut3, freeglut3-dev, libboost1.46-all-dev, libccd (>=1.4.2),
     fcl (>=0.2.7), libeigen3-dev, libxmu-dev, libxi-dev, libgtest-dev,
     libtinyxml-dev, libtinyxml2-dev, libassimp3, libassimp-dev (>=3.0~dfsg-1),
     urdfdom")
if(HAVE_BULLET_COLLISION)
  set(CPACK_DEBIAN_PACKAGE_DEPENDS ${CPACK_DEBIAN_PACKAGE_DEPENDS} "libbullet-dev (>=2.8.0)")
endif()
if(HAVE_NLOPT)
  set(CPACK_DEBIAN_PACKAGE_DEPENDS ${CPACK_DEBIAN_PACKAGE_DEPENDS} "libnlopt-dev")
endif()
if(HAVE_IPOPT)
  set(CPACK_DEBIAN_PACKAGE_DEPENDS ${CPACK_DEBIAN_PACKAGE_DEPENDS} "coinor-libipopt-dev")
endif()

set(CPACK_PACKAGE_VERSION "${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH}")
set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}_${CPACK_PACKAGE_VERSION}_${CPACK_SYSTEM_NAME}")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Karen Liu (www.cc.gatech.edu/~karenliu/)")

set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "${PKG_DESC}")
set(CPACK_PACKAGE_VENDOR "Computer Graphics Lab at GT")
set(CPACK_PACKAGING_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX})

include(CPack)
