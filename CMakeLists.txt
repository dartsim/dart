#===============================================================================
# CMake settings
#===============================================================================
cmake_minimum_required(VERSION 2.8.6 FATAL_ERROR)

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
# Poject settings
#===============================================================================
project(dart)

set(DART_MAJOR_VERSION "4")
set(DART_MINOR_VERSION "3")
set(DART_PATCH_VERSION "7")
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
  set(DART_RUNTIME_LIBRARY "/MT" CACHE STRING "BaseName chosen by the user at CMake configure time")
  set_property(CACHE DART_RUNTIME_LIBRARY PROPERTY STRINGS /MD /MT)
  option(DART_MSVC_DEFAULT_OPTIONS "Build DART with default Visual Studio options" OFF)
else()
  option(BUILD_SHARED_LIBS "Build shared libraries" ON)
endif()
option(DART_BUILD_EXAMPLES "Build examples" ON)
option(DART_BUILD_UNITTESTS "Build unit tests" ON)

#===============================================================================
# Build type settings
#===============================================================================
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build, options are: None Debug Release RelWithDebInfo Profile." FORCE)
endif()
string(TOUPPER ${CMAKE_BUILD_TYPE} CMAKE_BUILD_TYPE_UPPERCASE)

#===============================================================================
# Find dependencies
#===============================================================================

#------------------------
# Mandatory dependencies
#------------------------
message(STATUS "")
message(STATUS "[ Mandatory dependencies ]")

# Eigen
find_package(Eigen QUIET)
if(EIGEN_FOUND)
  message(STATUS "Looking for Eigen - found")
else()
  message(SEND_ERROR "Looking for Eigen - NOT found, please install libeigen3-dev")
endif()

# FCL
find_package(FCL QUIET)
if(FCL_FOUND)
  message(STATUS "Looking for FCL - found")
else()
  message(SEND_ERROR "Looking for FCL - NOT found, please install libfcl-dev")
endif()

# Assimp
find_package(Assimp QUIET)
if(ASSIMP_FOUND)
  message(STATUS "Looking for Assimp - found")
else()
  message(SEND_ERROR "Looking for Assimp - NOT found, please install libassimp-dev")
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
if(MSVC OR MSVC90 OR MSVC10)
  add_definitions(-DBOOST_ALL_NO_LIB)
endif()
add_definitions(-DBOOST_TEST_DYN_LINK)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost ${DART_MIN_BOOST_VERSION} COMPONENTS system QUIET)
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
  find_package(FLANN QUIET)
  if(FLANN_FOUND)
    message(STATUS "Looking for FLANN - found")
  else()
    message(SEND_ERROR "Looking for FLANN - NOT found, please install libflann-dev")
  endif()

  # TinyXML
  find_package(TinyXML QUIET)
  if(TINYXML_FOUND)
    message(STATUS "Looking for TinyXML - found")
  else()
    message(SEND_ERROR "Looking for TinyXML - NOT found, please install libtinyxml-dev")
  endif()

  # TinyXML2
  find_package(TinyXML2 QUIET)
  if(TINYXML2_FOUND)
    message(STATUS "Looking for TinyXML2 - found")
  else()
    message(SEND_ERROR "Looking for TinyXML2 - NOT found, please install libtinyxml2-dev")
  endif()

  # urdfdom
  find_package(urdfdom QUIET)
  if(urdfdom_FOUND)
    if(${urdfdom_headers_VERSION})
      message(STATUS "Looking for urdfdom - ${urdfdom_headers_VERSION} found")
    else()
      message(STATUS "Looking for urdfdom - found")
    endif()
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
find_package(NLOPT QUIET)
if(NLOPT_FOUND)
  message(STATUS "Looking for NLOPT - found")
  set(HAVE_NLOPT TRUE)
else()
  message(STATUS "Looking for NLOPT - NOT found, please install libnlopt-dev")
  set(HAVE_NLOPT FALSE)
endif()

# IPOPT
find_package(IPOPT QUIET)
if(IPOPT_FOUND)
  message(STATUS "Looking for IPOPT - found")
  set(HAVE_IPOPT TRUE)
else()
  message(STATUS "Looking for IPOPT - NOT found, please install coinor-libipopt-dev")
  set(HAVE_IPOPT FALSE)
endif()

# Shark
find_package(SHARK QUIET)
if(SHARK_FOUND)
  message(STATUS "Looking for SHARK - found")
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
  message(STATUS "Looking for BulletCollision - found")
  set(HAVE_BULLET_COLLISION TRUE)
else()
  message(STATUS "Looking for BulletCollision - NOT found, please install libbullet-dev")
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
set(DART_CORE_DEPENDENCIES ${FCL_LIBRARIES}
                           ${Assimp_LIBRARIES}
                           ${Boost_LIBRARIES}
                           ${OPENGL_LIBRARIES}
                           ${GLUT_LIBRARY}
)

if(HAVE_BULLET_COLLISION)
  set(DART_CORE_DEPENDENCIES ${DART_CORE_DEPENDENCIES} ${BULLET_LIBRARIES})
endif()

if(NOT BUILD_CORE_ONLY)
  set(DART_DEPENDENCIES ${urdfdom_LIBRARIES}
                        ${TinyXML_LIBRARIES}
                        ${TinyXML2_LIBRARIES}
  )
  if(HAVE_NLOPT)
    set(DART_DEPENDENCIES ${DART_DEPENDENCIES} ${NLOPT_LIBRARIES})
  endif()
  if(HAVE_IPOPT)
    set(DART_DEPENDENCIES ${DART_DEPENDENCIES} ${IPOPT_LIBRARIES})
  endif()
  if(HAVE_SHARK)
    set(DART_DEPENDENCIES ${DART_DEPENDENCIES} ${SHARK_LIBRARIES})
  endif()
  message(STATUS "FLANN_LIBRARIES: ${FLANN_LIBRARIES}")
  set(DART_DEPENDENCIES ${DART_DEPENDENCIES} ${FLANN_LIBRARIES})
endif()

#===============================================================================
# Include directories
#===============================================================================
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
  if(HAVE_SHARK)
    include_directories(${SHARK_INCLUDE_DIRS})
  endif()
endif()

include_directories("${CMAKE_BINARY_DIR}")

#===============================================================================
# Link directories
#===============================================================================
link_directories("${CMAKE_BINARY_DIR}/lib")
link_directories("${CMAKE_INSTALL_PREFIX}/lib")
link_directories(${Boost_LIBRARY_DIRS})
if(HAVE_BULLET_COLLISION)
  link_directories("${BULLET_LIBRARY_DIRS}")
endif()

#===============================================================================
# Check for non-case-sensitive filesystems
#===============================================================================
execute_process(COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/tools/case_sensitive_filesystem
                RESULT_VARIABLE FILESYSTEM_CASE_SENSITIVE_RETURN)
if (${FILESYSTEM_CASE_SENSITIVE_RETURN} EQUAL 0)
  set(FILESYSTEM_CASE_SENSITIVE TRUE)
else()
  set(FILESYSTEM_CASE_SENSITIVE FALSE)
endif()

#===============================================================================
# Compiler flags
#===============================================================================
if(MSVC)
  # Visual Studio enables c++11 support by default
  if(NOT MSVC12)
    message(FATAL_ERROR "${PROJECT_NAME} requires VS 2013 or greater.")
  endif()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP4")
  set(CMAKE_EXE_LINKER_FLAGS_RELEASE "/LTCG /INCREMENTAL:NO")
  if(NOT DART_MSVC_DEFAULT_OPTIONS)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} ${DART_RUNTIME_LIBRARY}d /Zi /Gy /W1 /EHsc")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${DART_RUNTIME_LIBRARY} /Zi /GL /Gy /W1 /EHsc /arch:SSE2")
  endif(NOT DART_MSVC_DEFAULT_OPTIONS)
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
message(STATUS "Build core only  : ${BUILD_CORE_ONLY}")
message(STATUS "Build examples   : ${DART_BUILD_EXAMPLES}")
message(STATUS "Build unit tests : ${DART_BUILD_UNITTESTS}")
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
configure_file(${PC_CONFIG_IN} ${PC_CONFIG_OUT} @only)
install(FILES ${PC_CONFIG_OUT} DESTINATION lib/pkgconfig)

#===============================================================================
# Add sub-directories
#===============================================================================
add_subdirectory(dart)

if(NOT BUILD_CORE_ONLY)

  # Unit tests
  if(DART_BUILD_UNITTESTS)
    enable_testing()
    add_subdirectory(unittests)
  endif()

  # Examples
  if(DART_BUILD_EXAMPLES)
    add_subdirectory(apps)
  endif()

endif()

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
# Package Installer
#===============================================================================
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
     fcl (>=0.2.7), libeigen3-dev, libxmu-dev, libxi-dev,
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

set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "${DART_PKG_DESC}")
set(CPACK_PACKAGE_VENDOR "Computer Graphics Lab at GT")
set(CPACK_PACKAGING_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX})

include(CPack)

#===============================================================================
# END
#===============================================================================
message(STATUS "")
