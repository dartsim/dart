cmake_minimum_required(VERSION 2.8)
set(CMAKE_LEGACY_CYGWIN_WIN32 0)
set(CMAKE_CXX_WARNING_LEVEL 4) 
# Disables a warning about a change in Cygwin Cmake

project(dart_all)
message(STATUS "Generate makefile/project file for DART")

set(DART_MAJOR_VERSION "2")
set(DART_MINOR_VERSION "1")
set(DART_PATCH_VERSION "1")
set(PKG_DESC "Dynamic Animation and Robotics Toolkit.")
set(DART_VERSION "${DART_MAJOR_VERSION}.${DART_MINOR_VERSION}.${DART_PATCH_VERSION}")
set(PKG_EXTERNAL_DEPS "flann, ccd, fcl")

message(STATUS "CMAKE_SOURCE_DIR = ${CMAKE_SOURCE_DIR}")
message(STATUS "CMAKE_BINARY_DIR = ${CMAKE_BINARY_DIR}")

option(BUILD_SNOPT "Build with SNOPT optimizer" OFF)
option(BUILD_ROBOTICS "Compile with Robotics API" ON)

if(MSVC)
  message(STATUS "Setup Visual Studio Specific Flags")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MDd /Zi /Gy /W1 /EHsc")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MD /Zi /GL /Gy /W1 /EHsc")
  set(CMAKE_DEBUG_POSTFIX "d")
  set(CMAKE_EXE_LINKER_FLAGS_RELEASE "/LTCG")
  set(CMAKE_INSTALL_PREFIX "C:/Golems" CACHE PATH "Install prefix" FORCE)
elseif(UNIX)  
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++0x")
  set(CMAKE_CXX_FLAGS_PROFILE "-pg" CACHE STRING "Profiling flags") 
  option(ENABLE_MSSE2 "Enable -msse2 flag" ON)
  set(CMAKE_INSTALL_PREFIX "/usr" CACHE PATH "Install prefix" FORCE)
endif()

###############
# Generate the path header file
set(PATH_HEADER_DIR ${CMAKE_SOURCE_DIR}/src/utils/Paths.h)
message(STATUS "Generate the path header file to  ${PATH_HEADER_DIR}")
configure_file(${PATH_HEADER_DIR}.in ${PATH_HEADER_DIR} @ONLY)

###############
# Install Targets
# Generate the Dart CMake Config file
file(GLOB cmake_mods "cmake/Modules/*.cmake")
configure_file(${CMAKE_SOURCE_DIR}/cmake/DARTConfig.cmake.in ${CMAKE_SOURCE_DIR}/cmake/FindDART.cmake @ONLY)
configure_file(${CMAKE_SOURCE_DIR}/cmake/DARTExtConfig.cmake.in ${CMAKE_SOURCE_DIR}/cmake/FindDARTExt.cmake @ONLY)
install(FILES ${CMAKE_SOURCE_DIR}/cmake/FindDART.cmake DESTINATION share/dart)
install(FILES ${CMAKE_SOURCE_DIR}/cmake/FindDARTExt.cmake DESTINATION share/dart)
if(UNIX)
    install(FILES ${CMAKE_SOURCE_DIR}/cmake/FindDART.cmake DESTINATION ${CMAKE_ROOT}/Modules)
    install(FILES ${CMAKE_SOURCE_DIR}/cmake/FindDARTExt.cmake DESTINATION ${CMAKE_ROOT}/Modules)
endif()

set(PC_CONFIG ${CMAKE_SOURCE_DIR}/cmake/dart.pc)
configure_file(${PC_CONFIG}.in ${PC_CONFIG} @only)
install(FILES ${PC_CONFIG} DESTINATION lib/pkgconfig)

###############
# Find important packages
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(DARTExt REQUIRED)

find_path( MATHPLOT_INCLUDE_DIR NAMES mathplot2.h )
message(STATUS "MATHPLOT = ${MATHPLOT_INCLUDE_DIR}")

message(STATUS "Building DART source")
# Since you are developing DART
include_directories(BEFORE src)
include_directories(${DARTExt_INCLUDEDIR})
link_directories(${DARTExt_LIBRARY_DIRS})
link_directories(${CMAKE_SOURCE_DIR}/lib)
add_subdirectory(src)

message(STATUS "DARTExt_LIBRARIES is ${DARTExt_LIBRARIES}")

add_subdirectory(unittests)
add_subdirectory(apps)

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
set(CPACK_DEBIAN_PACKAGE_DEPENDS "freeglut3, freeglut3-dev, libboost-dev, flann (>=1.8), libccd (>=1.4.2), libfcl (>=0.2.7), libeigen3-dev, libxmu-dev, libxi-dev, libgtest-dev, libtinyxml-dev, libassimp3")

set(CPACK_PACKAGE_VERSION "${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH}")
set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}-${CPACK_SYSTEM_NAME}")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Karen Liu (www.cc.gatech.edu/~karenliu/)")

set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "${PKG_DESC}")
set(CPACK_PACKAGE_VENDOR "Computer Graphics Lab at GT")

include(CPack)
