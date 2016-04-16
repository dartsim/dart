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
set(DART_PKG_EXTERNAL_DEPS "eigen, ccd, fcl, assimp, boost")

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
include(DARTFindDependencies)

#===============================================================================
# Include directories
#===============================================================================
include_directories(BEFORE ${PROJECT_SOURCE_DIR})
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})
include_directories(SYSTEM ${CCD_INCLUDE_DIRS})
include_directories(SYSTEM ${FCL_INCLUDE_DIRS})
include_directories(SYSTEM ${ASSIMP_INCLUDE_DIRS})
include_directories(SYSTEM ${Boost_INCLUDE_DIRS})
if(HAVE_BULLET_COLLISION)
  include_directories(SYSTEM ${BULLET_INCLUDE_DIRS})
endif()
include_directories("${CMAKE_BINARY_DIR}")

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
set(DART_CONFIG_IN ${CMAKE_SOURCE_DIR}/cmake/DARTConfig.cmake.in)
set(DART_CONFIG_OUT ${CMAKE_BINARY_DIR}/cmake/DARTConfig.cmake)
message(STATUS ${DART_CONFIG_OUT})
message(STATUS ${CMAKE_BINARY_DIR}/cmake/DARTConfigVersion.cmake)
configure_file(${DART_CONFIG_IN} ${DART_CONFIG_OUT} @ONLY)
write_basic_config_version_file(cmake/DARTConfigVersion.cmake VERSION ${DART_VERSION} COMPATIBILITY SameMajorVersion)
install(FILES ${DART_CONFIG_OUT} ${CMAKE_BINARY_DIR}/cmake/DARTConfigVersion.cmake DESTINATION share/dart)

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
install(EXPORT DARTTargets DESTINATION share/dart)

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
