#===============================================================================
# CMake settings
#===============================================================================
cmake_minimum_required(VERSION 3.22.1)

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

#===============================================================================
# Project settings
#===============================================================================

project(dart)

string(TOUPPER ${PROJECT_NAME} PROJECT_NAME_UPPERCASE)

# Extract version numbers from package.xml
file(READ package.xml PACKAGE_XML)
set(version_regex "<version>([0-9]+)\\.([0-9]+)\\.([0-9]+)(-[a-z]*[0-9]+)?</version>")
string(REGEX MATCH "${version_regex}" DIRTY_VERSION_STRING ${PACKAGE_XML})
string(REGEX REPLACE "^${version_regex}$" "\\1" DART_MAJOR_VERSION "${DIRTY_VERSION_STRING}")
string(REGEX REPLACE "^${version_regex}$" "\\2" DART_MINOR_VERSION "${DIRTY_VERSION_STRING}")
string(REGEX REPLACE "^${version_regex}$" "\\3" DART_PATCH_VERSION "${DIRTY_VERSION_STRING}")
string(FIND ${DIRTY_VERSION_STRING} \\4 FOUND_DART_PRERELEASE_VERSION)
if(FOUND_DART_PRERELEASE_VERSION)
  string(REGEX REPLACE "^${version_regex}$" "\\4" DART_PRERELEASE_VERSION "${DIRTY_VERSION_STRING}")
  set(DART_VERSION "${DART_MAJOR_VERSION}.${DART_MINOR_VERSION}.${DART_PATCH_VERSION}${DART_PRERELEASE_VERSION}")
else()
  set(DART_VERSION "${DART_MAJOR_VERSION}.${DART_MINOR_VERSION}.${DART_PATCH_VERSION}")
endif()

set(DART_PKG_DESC "Dynamic Animation and Robotics Toolkit.")
set(DART_PKG_EXTERNAL_DEPS "assimp, ccd, eigen3, fcl, octomap")
# Note: Check validity by running: pkg-config --list-all | grep <pkg_name>

# Use DART_SOURCE/BINARY_DIR instead of CMAKE_SOURCE/BINARY_DIR to support the
# case that DART is built as a sub-project in another project.
set(DART_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(DART_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR})

include(GNUInstallDirs)

# Variables used in Components.cmake
set(INCLUDE_INSTALL_DIR ${CMAKE_INSTALL_INCLUDEDIR})
set(LIBRARY_INSTALL_DIR ${CMAKE_INSTALL_LIBDIR})
set(CONFIG_INSTALL_DIR "${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}${DART_MAJOR_VERSION}/cmake")

# Set relative location to install additional documentation (sample data,
# examples, and tutorials)
set(DART_ADDITIONAL_DOCUMENTATION_INSTALL_PATH
  "${CMAKE_INSTALL_DATAROOTDIR}/doc/${PROJECT_NAME}${DART_MAJOR_VERSION}"
)

set(CMAKE_DEBUG_POSTFIX "d")
set(CMAKE_MODULE_PATH "${DART_SOURCE_DIR}/cmake")

include(DARTMacros)

# CMake component helper. It uses the following variables:
# - LIBRARY_INSTALL_DIR
# - CONFIG_INSTALL_DIR
include(Components)
initialize_component_helpers(${PROJECT_NAME}${DART_MAJOR_VERSION})

#===============================================================================
# Build options
#===============================================================================
dart_option(DART_VERBOSE "Whether print detailed information in CMake process" OFF)
dart_option(BUILD_SHARED_LIBS "Build shared libraries" ON)
# Note: DART_ENABLE_SIMD should only be enabled when building DART and its
# dependent projects on the same machine. Enabling this option adds the
# -march=native compile flag to the dart target, which enables all instruction
# subsets supported by the local machine. Building on different machines with
# differing architectures may lead to runtime errors, particularly memory
# alignment errors.
dart_option(DART_ENABLE_SIMD
  "Build DART with all SIMD instructions on the current local machine" ON)
dart_option(DART_ENABLE_GPU "Build DART with GPU support when OpenCL is available" ON)
dart_option(DART_BUILD_DARTPY "Build dartpy" ON)
dart_option(DART_CODECOV "Turn on codecov support" OFF)
dart_option(DART_FAST_DEBUG "Add -O1 option for DEBUG mode build" OFF)
# GCC and Clang add ANSI-formatted colors when they detect the output medium is a
# terminal. However, this doesn't work in some cases such as when the makefile is
# invoked by Ninja. DART_FORCE_COLORED_OUTPUT can be used in this case to enforce
# to always generate ANSI-formatted colors regardless of the output medium types.
# See: https://medium.com/@alasher/colored-c-compiler-output-with-ninja-clang-gcc-10bfe7f2b949
dart_option(DART_FORCE_COLORED_OUTPUT
  "Always produce ANSI-colored output (GNU/Clang only)." OFF)
dart_option(DART_DOWNLOAD_DEPENDENT_PACKAGES "Whether to download dependent package" ON)

dart_option(DART_PROFILE_BACKEND_REMOTERY "Enable Remotery profiling backend" OFF)

dart_option(DART_BUILD_TEMPLATE_CODE_FOR_DOUBLE "Build instantiation of template code for double" ON)
dart_option(DART_BUILD_TEMPLATE_CODE_FOR_FLOAT  "Build instantiation of template code for float" OFF)

dart_option(DART_BUILD_COMP_common       "Build component 'common'"       ON)
dart_option(DART_BUILD_COMP_math         "Build component 'math'"         ON)
dart_option(DART_BUILD_COMP_optimization "Build component 'optimization'" ON)
dart_option(DART_BUILD_COMP_collision    "Build component 'collision'"    ON)
dart_option(DART_BUILD_COMP_dynamics     "Build component 'dynamics'"     ON)
dart_option(DART_BUILD_COMP_simulation   "Build component 'simulation'"   ON)
dart_option(DART_BUILD_COMP_io           "Build component 'io'"           ON)
dart_option(DART_BUILD_COMP_gui          "Build component 'gui'"          ON)
dart_option(DART_BUILD_COMP_planning     "Build component 'planning'"     ON)

dart_option(DART_DBUILD_TESTING "Enable building tests and benchmark" ON)
dart_option(DART_IN_CI "Indicate building DART as part of CI"        OFF)
dart_option(DART_BUILD_WHEELS "Indicate building dartpy for wheels"  OFF)

#===============================================================================
# Print intro
#===============================================================================
message(STATUS "")
message(STATUS "============================================")
message(STATUS "                DART ${DART_VERSION}")
message(STATUS "============================================")
message(STATUS "")

# Print build tool information
message(STATUS "[ Build Tools ]")
message(STATUS "- CMake    : ${CMAKE_VERSION} (${CMAKE_COMMAND})")
message(STATUS "- C++      : ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "- Toolchain: ${CMAKE_TOOLCHAIN_FILE}")
message(STATUS "- Generator: ${CMAKE_GENERATOR} (${CMAKE_SYSTEM_PROCESSOR})")
if(CMAKE_PREFIX_PATH)
  message(STATUS "  - prefix: ${CMAKE_PREFIX_PATH}")
endif()
message(STATUS "")

# Print DART options
dart_print_options()

#===============================================================================
# CodeCov settings
#===============================================================================
if(DART_CODECOV)
  # Set up custom targets for code coverage
  dart_coverage(
    INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/dart
    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/dart
  )

  # Code Coverage Configuration
  add_library(coverage_config INTERFACE)

  # CodeCov can only be enabled in Debug mode
  if(NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
    message(FATAL_ERROR "CodeCov can only be enabled in Debug mode")
  endif()

  # CodeCov can only be enabled with GCC or Clang
  if(NOT CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    message(FATAL_ERROR "CodeCov can only be enabled with GCC or Clang")
  endif()

  # Add required flags (GCC & LLVM/Clang)
  target_compile_options(coverage_config INTERFACE
    -O0        # no optimization
    -g         # generate debug info
    --coverage # sets all required flags
  )

  # Add required flags (GCC & LLVM/Clang)
  target_link_options(coverage_config INTERFACE --coverage)

  # Export CodeCov configuration
  install(TARGETS coverage_config DESTINATION lib EXPORT coverage_config)
  install(EXPORT coverage_config DESTINATION ${CONFIG_INSTALL_DIR})
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
  message(WARNING "CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} unknown. Valid options are: Debug | Release | RelWithDebInfo | MinSizeRel")
endif()

# Active log level:
#
# - TRACE: To enable log with DART_TRACE() and below
# - DEBUG: To enable log with DART_DEBUG() and below
# - INFO: To enable log with DART_INFO() and below
# - WARN: To enable log with DART_WARN() and below
# - ERROR: To enable log with DART_ERROR() and below
# - FATAL: To enable log with DART_FATAL()
# - OFF: To turn off all the logs
if(BUILD_TYPE_DEBUG)
  set(DART_ACTIVE_LOG_LEVEL "DEBUG" CACHE STRING "Compile time active log level to enable")
else()
  set(DART_ACTIVE_LOG_LEVEL "INFO" CACHE STRING "Compile time active log level to enable")
endif()
set_property(CACHE DART_ACTIVE_LOG_LEVEL PROPERTY STRINGS TRACE DEBUG INFO WARN ERROR FATAL OFF)

if(BUILD_TYPE_DEBUG)
  option(DART_TREAT_WARNINGS_AS_ERRORS "Treat warnings as errors" OFF)
else()
  option(DART_TREAT_WARNINGS_AS_ERRORS "Treat warnings as errors" ON)
endif()

#===============================================================================
# Find dependencies
#===============================================================================
include(DARTFindDependencies)

message(STATUS "[ Configurations]")
if(DART_HAS_OPENCL AND DART_ENABLE_GPU)
  message(STATUS "- DART_ENABLED_GPU: ON")
else()
  message(STATUS "- DART_ENABLED_GPU: OFF")
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

  # Visual Studio enables C++17 support by default
  set(msvc_required_version 1930) # 1930 = VS 17.0 (https://cmake.org/cmake/help/latest/variable/MSVC_VERSION.html)
  if(MSVC_VERSION VERSION_LESS ${msvc_required_version})
    message(FATAL_ERROR "Visual Studio ${MSVC_VERSION} is detected, but "
      "${PROJECT_NAME_UPPERCASE} requires ${msvc_required_version} or greater."
    )
  endif()

  if(DART_TREAT_WARNINGS_AS_ERRORS)
    add_compile_options(/WX)
  endif()
  
  # Required for full compiler support for the Standard C++ exception handling
  # model that safely unwinds stack objects.
  add_compile_options(/EHsc)

  # Required to specify standards-conforming compiler behavior. This option
  # disables permissive behaviors.
  add_compile_options(/permissive-)

  # Required to tell the compiler to use the original, non-conforming Microsoft
  # C++ compiler behavior to parse and instantiate class templates and function
  # templates.
  add_compile_options(/Zc:twoPhase-)
  
  add_definitions(-D_CRT_SECURE_NO_WARNINGS -D_ENABLE_EXTENDED_ALIGNED_STORAGE)
  add_compile_options(/wd4005)
  add_compile_options(/wd4099)
  add_compile_options(/wd4146)  # for FCL warnings: https://github.com/dartsim/dart/runs/4568423649?check_suite_focus=true#step:5:407
  add_compile_options(/wd4244)
  add_compile_options(/wd4250)
  add_compile_options(/wd4267)
  add_compile_options(/wd4305)
  add_compile_options(/wd4334)
  add_compile_options(/wd4506)
  add_compile_options(/wd4838)
  add_compile_options(/wd4996)
  
  add_compile_options(/bigobj)

  if(DART_IN_CI)
    # Increase the maximum number of sections in an object file.
    add_compile_options(/Zm2000)  # 2000 max
  else()
    # Enable multi-threaded compilation.
    add_compile_options(/MP)
  endif()

  add_link_options(/OPT:NOREF /FORCE:MULTIPLE)

  set(CMAKE_INSTALL_UCRT_LIBRARIES ON)

elseif(CMAKE_COMPILER_IS_GNUCXX)

  add_compile_options(-Wall -Wextra -fPIC)
  if(DART_TREAT_WARNINGS_AS_ERRORS)
    add_compile_options(-Werror)
  endif()
  execute_process(
    COMMAND ${CMAKE_CXX_COMPILER} -dumpfullversion -dumpversion OUTPUT_VARIABLE GCC_VERSION)
  if(DART_BUILD_WHEELS)
    set(gcc_required_version 10.2.1) # Lowered from 11.2.0 to support building in manylinux2014
  else()
    set(gcc_required_version 11.2.0)
  endif()
  if(GCC_VERSION VERSION_LESS ${gcc_required_version})
    message(FATAL_ERROR "The installed g++ version is ${GCC_VERSION}. ${PROJECT_NAME} requires g++ ${gcc_required_version} or greater.")
  endif()
  set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
  set(CMAKE_CXX_FLAGS_DEBUG "-g -fno-omit-frame-pointer -fno-inline-functions -fno-inline-functions-called-once -fno-optimize-sibling-calls")
  if(DART_FAST_DEBUG)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O1")
  endif()
  set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELEASE} ${CMAKE_CXX_FLAGS_DEBUG}")
  set(CMAKE_CXX_FLAGS_PROFILE "${CMAKE_CXX_FLAGS_DEBUG} -pg")
  set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--no-undefined")

  # Enforce to colorize compilation output
  if(${DART_FORCE_COLORED_OUTPUT})
    add_compile_options(-fdiagnostics-color=always)
  endif()

elseif("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")

  add_compile_options(-Wall -Wextra -fPIC)
  if(DART_TREAT_WARNINGS_AS_ERRORS)
    add_compile_options(-Werror)
    # TODO(JS): Remove "-Wno-error=deprecated-declarations" once glut is removed.
    if("${CMAKE_SYSTEM_NAME}" MATCHES "Darwin")
      add_compile_options(-Wno-error=deprecated-declarations)
    endif()
  endif()
  execute_process(
    COMMAND ${CMAKE_CXX_COMPILER} -dumpversion OUTPUT_VARIABLE CLANG_VERSION)
  set(clang_required_version 13.0.0)
  if(CLANG_VERSION VERSION_LESS ${clang_required_version})
    message(FATAL_ERROR "The installed Clang version is ${CLANG_VERSION}. ${PROJECT_NAME} requires clang ${clang_required_version} or greater.")
  endif()
  if("${CMAKE_SYSTEM_NAME}" MATCHES "Darwin")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
  endif()
  set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
  set(CMAKE_CXX_FLAGS_DEBUG "-g -fno-omit-frame-pointer -fno-inline-functions -fno-optimize-sibling-calls")
  if(DART_FAST_DEBUG)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O1")
  endif()
  set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELEASE} ${CMAKE_CXX_FLAGS_DEBUG}")
  set(CMAKE_CXX_FLAGS_PROFILE "${CMAKE_CXX_FLAGS_DEBUG} -pg")

  # Enforce to colorize compilation output
  if(${DART_FORCE_COLORED_OUTPUT})
     add_compile_options(-fcolor-diagnostics)
  endif()

else()

  message(SEND_ERROR "Compiler[${CMAKE_CXX_COMPILER_ID}] not supported.")

endif()

#===============================================================================
# Print build summary
#===============================================================================
if(DART_VERBOSE)
  message(STATUS "")
  message(STATUS "[ Build summary ]")
  message(STATUS "CMAKE_GENERATOR  : ${CMAKE_GENERATOR}")
  message(STATUS "Compiler ID      : ${CMAKE_CXX_COMPILER_ID}")
  message(STATUS "Compiler version : ${CMAKE_CXX_COMPILER_VERSION}")
  message(STATUS "Build type       : ${CMAKE_BUILD_TYPE}")
  message(STATUS "BUILD_SHARED_LIBS: ${BUILD_SHARED_LIBS}")
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
  message(STATUS "DART_SOURCE_DIR : ${DART_SOURCE_DIR}")
  message(STATUS "DART_BINARY_DIR : ${DART_BINARY_DIR}")
endif(DART_VERBOSE)

#===============================================================================
# Add sub-directories
#===============================================================================

add_subdirectory(tools)

add_subdirectory(dart)

set(DART_IN_SOURCE_BUILD TRUE)

# Add a "tests" target to build unit tests.
enable_testing()
if(DART_DBUILD_TESTING)
  if(MSVC)
    add_subdirectory(tests)
  else()
    add_subdirectory(tests EXCLUDE_FROM_ALL)
  endif()
endif()
# Add example subdirectories and an "examples" target.
if(MSVC)
  # add_subdirectory(examples)
else()
  add_subdirectory(examples EXCLUDE_FROM_ALL)
  get_property(examples GLOBAL PROPERTY DART_EXAMPLES)
  add_custom_target(examples DEPENDS ${examples})
endif()
if(DART_VERBOSE)
  message(STATUS "")
  message(STATUS "[ Examples ]")
  foreach(example ${examples})
    message(STATUS "Adding example: ${example}")
  endforeach(example ${examples})
else(DART_VERBOSE)
  list(LENGTH examples examples_length)
  message(STATUS "Adding ${examples_length} examples")
endif(DART_VERBOSE)
# Add a "tutorials" target to build tutorials.
if(MSVC)
  # add_subdirectory(tutorials)
else()
  add_subdirectory(tutorials EXCLUDE_FROM_ALL)
  get_property(tutorials GLOBAL PROPERTY DART_TUTORIALS)
  add_custom_target(tutorials DEPENDS ${tutorials})
endif()
if(DART_VERBOSE)
  message(STATUS "")
  message(STATUS "[ Tutorials ]")
  foreach(tutorial ${tutorials})
    message(STATUS "Adding tutorial: ${tutorial}")
  endforeach(tutorial ${tutorials})
else(DART_VERBOSE)
  list(LENGTH tutorials tutorials_length)
  message(STATUS "Adding ${tutorials_length} tutorials")
endif(DART_VERBOSE)

if(DART_BUILD_DARTPY)
  add_subdirectory(python)
endif()

# Add 'ALL' target that builds everything
set(all_target_candidates tests_and_run dartpy pytest)
foreach(target_candidate ${all_target_candidates})
  if(TARGET ${target_candidate})
    list(APPEND all_targets ${target_candidate})
  endif()
endforeach()
foreach(target_candidate ${examples})
  if(TARGET ${target_candidate})
    list(APPEND all_targets ${target_candidate})
  endif()
endforeach()
foreach(target_candidate ${tutorials})
  if(TARGET ${target_candidate})
    list(APPEND all_targets ${target_candidate})
  endif()
endforeach()
add_custom_target(ALL DEPENDS ${all_targets})

#===============================================================================
# CMake configuration files for components and targets
#===============================================================================
# Generate and install CMake configuration files for each component <C>:
# - <C>Component.cmake, which defines:
#   - dart_<C>_DEPENDENCIES: list of component dependencies
#   - dart_<C>_LIBRARIES: list of library targets in this component
# - <C>Targets.cmake, which creates IMPORTED targets
install_component_exports(${PROJECT_NAME})

#===============================================================================
# Configure files
#===============================================================================
if(DART_VERBOSE)
  message(STATUS "")
  message(STATUS "[ Configured files ]")
endif()

# Generate and install a Config.cmake file. This file includes the
# <C>Component.cmake and <C>Targets.cmake created above. It also uses the
# following variables:
#
# - PACKAGE_INCLUDE_INSTALL_DIR
# - PACKAGE_INCLUDE_DIRS
get_property(PACKAGE_INCLUDE_DIRS GLOBAL
  PROPERTY "${PROJECT_NAME_UPPERCASE}${DART_MAJOR_VERSION}_INCLUDE_DIRS"
)

# Generate the DART CMake Config and version files
include(CMakePackageConfigHelpers)
set(DART_CONFIG_IN ${DART_SOURCE_DIR}/cmake/${PROJECT_NAME_UPPERCASE}Config.cmake.in)
set(DART_CONFIG_OUT ${DART_BINARY_DIR}/${PROJECT_NAME_UPPERCASE}${DART_MAJOR_VERSION}Config.cmake)
set(DART_VERSION_OUT ${DART_BINARY_DIR}/cmake/${PROJECT_NAME_UPPERCASE}${DART_MAJOR_VERSION}ConfigVersion.cmake)
if(DART_VERBOSE)
  message(STATUS ${DART_CONFIG_OUT})
  message(STATUS ${DART_VERSION_OUT})
endif()
configure_package_config_file(
  ${DART_CONFIG_IN}
  ${DART_CONFIG_OUT}
  INSTALL_DESTINATION "${CONFIG_INSTALL_DIR}"
  PATH_VARS INCLUDE_INSTALL_DIR
)
write_basic_config_version_file(
  ${DART_VERSION_OUT}
  VERSION ${${PROJECT_NAME_UPPERCASE}_VERSION}
  COMPATIBILITY SameMajorVersion
)
install(
  FILES ${DART_CONFIG_OUT} ${DART_VERSION_OUT}
  DESTINATION "${CONFIG_INSTALL_DIR}"
)

# Generate the DART pkg-config
set(PC_CONFIG_IN ${DART_SOURCE_DIR}/cmake/dart.pc.in)
set(PC_CONFIG_OUT ${DART_BINARY_DIR}/cmake/dart${DART_MAJOR_VERSION}.pc)
set(PC_CONFIG_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/pkgconfig)
file(RELATIVE_PATH
  RELATIVE_PATH_TO_INSTALL_PREFIX
  "${PC_CONFIG_INSTALL_DIR}"
  "${CMAKE_INSTALL_PREFIX}"
)
if(DART_VERBOSE)
  message(STATUS ${PC_CONFIG_OUT})
endif()
configure_file(${PC_CONFIG_IN} ${PC_CONFIG_OUT} @ONLY)
install(FILES ${PC_CONFIG_OUT} DESTINATION ${PC_CONFIG_INSTALL_DIR})

# Install a Catkin 'package.xml' file. This is required by REP-136.
install(FILES package.xml DESTINATION
  ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}${DART_MAJOR_VERSION}
)

#===============================================================================
# Install sample data, examples, and tutorials
#===============================================================================

# Sample data
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/data"
  DESTINATION ${DART_ADDITIONAL_DOCUMENTATION_INSTALL_PATH}
)

# Examples source
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/examples"
  DESTINATION ${DART_ADDITIONAL_DOCUMENTATION_INSTALL_PATH}
)

# Tutorials source
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/tutorials"
  DESTINATION ${DART_ADDITIONAL_DOCUMENTATION_INSTALL_PATH}
)

#===============================================================================
# Uninstall
#===============================================================================

# Add an "uninstall" target
# Ref: http://www.cmake.org/Wiki/CMake_FAQ#Can_I_do_.22make_uninstall.22_with_CMake.3F
configure_file("${PROJECT_SOURCE_DIR}/cmake/uninstall_target.cmake.in" "${PROJECT_BINARY_DIR}/uninstall_target.cmake" IMMEDIATE @ONLY)
add_custom_target(uninstall "${CMAKE_COMMAND}" -P "${PROJECT_BINARY_DIR}/uninstall_target.cmake")

#===============================================================================
# Code Formatting
#===============================================================================
if(DART_VERBOSE)
  message(STATUS "")
  message(STATUS "[ Code Formatting ]")
endif()

find_program(
  CLANG_FORMAT_EXECUTABLE
  NAMES clang-format-14
)

get_property(formatting_files GLOBAL PROPERTY DART_FORMAT_FILES)
list(LENGTH formatting_files formatting_files_length)

if(CLANG_FORMAT_EXECUTABLE)

  if(DART_VERBOSE)
    message(STATUS "Looking for clang-format - found")
  endif()

  message(STATUS "Formatting on ${formatting_files_length} source files.")

  if(formatting_files)
    add_custom_target(format
                      COMMAND ${CMAKE_COMMAND} -E echo "Formatting ${formatting_files_length} files... "
                      COMMAND ${CLANG_FORMAT_EXECUTABLE} -style=file -i ${formatting_files}
                      COMMAND ${CMAKE_COMMAND} -E echo "Done."
                      DEPENDS ${CLANG_FORMAT_EXECUTABLE}
                      WORKING_DIRECTORY ${DART_SOURCE_DIR}/dart)

    add_custom_target(check-format
                      COMMAND ${CMAKE_COMMAND} -E echo "Checking ${formatting_files_length} files... "
                      COMMAND ${DART_SOURCE_DIR}/tools/check_format.sh ${CLANG_FORMAT_EXECUTABLE} ${formatting_files}
                      COMMAND ${CMAKE_COMMAND} -E echo "Done."
                      DEPENDS ${CLANG_FORMAT_EXECUTABLE}
                      WORKING_DIRECTORY ${DART_SOURCE_DIR}/dart)
  else()
    add_custom_target(format
                      COMMAND ${CMAKE_COMMAND} -E echo "Warning: Not found any source files to format.")

    add_custom_target(check-format
                      COMMAND ${CMAKE_COMMAND} -E echo "Warning: Not found any source files to check.")
  endif()

else()

  if(DART_VERBOSE)
    message(WARNING "Looking for clang-format - NOT found, please install clang-format to enable automatic code formatting")
  endif()

endif()

#===============================================================================
# API Document using Doxygen
# References:
#   http://mementocodex.wordpress.com/2013/01/19/how-to-generate-code-documentation-with-doxygen-and-cmake-a-slightly-improved-approach/
#   http://www.cmake.org/pipermail/cmake/2007-February/012796.html
#===============================================================================
if(DOXYGEN_FOUND)

  set(DOXYGEN_DOXYFILE_IN      "${PROJECT_SOURCE_DIR}/doxygen/Doxyfile.in"    )
  set(DOXYGEN_DOXYFILE         "${PROJECT_BINARY_DIR}/doxygen/Doxyfile"       )
  set(DOXYGEN_HTML_INDEX       "${PROJECT_BINARY_DIR}/doxygen/html/index.html")
  set(DOXYGEN_OUTPUT_ROOT      "${PROJECT_BINARY_DIR}/doxygen/html"           )
  set(DOXYGEN_GENERATE_TAGFILE "${DOXYGEN_OUTPUT_ROOT}/${PROJECT_NAME}.tag"   )
  set(DOXYGEN_INCLUDE_PATH     "${PROJECT_SOURCE_DIR}"                        )
  set(DOXYGEN_INPUT_ROOT       "${PROJECT_SOURCE_DIR}/dart"                   )
  set(DOXYGEN_EXTRA_INPUTS     "${PROJECT_SOURCE_DIR}/doxygen/mainpage.dox"   )
  set(DOXYGEN_EXCLUDE          "${PROJECT_SOURCE_DIR}/dart/external"          )
  set(DOXYGEN_STRIP_FROM_PATH  "${CMAKE_CURRENT_SOURCE_DIR}"                  )

  # Generate a Doxyfile. This uses the variables:
  #
  # - DOXYGEN_OUTPUT_ROOT
  # - DOXYGEN_GENERATE_TAGFILE
  # - DOXYGEN_EXTRA_INPUTS
  # - DOXYGEN_INPUT_ROOT
  # - DOXYGEN_EXCLUDE
  # - DOXYGEN_STRIP_FROM_PATH
  configure_file(${DOXYGEN_DOXYFILE_IN} ${DOXYGEN_DOXYFILE} @ONLY)
  file(
    COPY "${PROJECT_SOURCE_DIR}/doxygen/DART logo.png"
    DESTINATION ${DOXYGEN_OUTPUT_ROOT}
  )
  add_custom_command(
    OUTPUT ${DOXYGEN_HTML_INDEX}
    COMMAND ${CMAKE_COMMAND} -E echo_append "Building API Documentation..."
    COMMAND ${DOXYGEN_EXECUTABLE} -u ${DOXYGEN_DOXYFILE}
    COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_DOXYFILE}
    # Strip path prefix from all paths in dart.tag
    COMMAND ${CMAKE_COMMAND} -E echo "Stripping paths from"
        "${DOXYGEN_GENERATE_TAGFILE}"
    COMMAND sed -i s:${DOXYGEN_STRIP_FROM_PATH}::g ${DOXYGEN_GENERATE_TAGFILE}
    # Strip all doxygen="path" HTML tags
    COMMAND ${CMAKE_COMMAND} -E echo "Stripping Doxygen HTML tags"
    COMMAND find "${DOXYGEN_OUTPUT_ROOT}" -type f -name "*.html"
        -exec sed -i 's: doxygen=\"[^\"]*\"::g' {} \\$<SEMICOLON>
    COMMAND ${CMAKE_COMMAND} -E echo "Done."
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/doxygen
    DEPENDS ${DOXYGEN_DOXYFILE}
  )
  # add_custom_target(docs ALL DEPENDS ${DOXYGEN_HTML_INDEX})
  add_custom_target(docs DEPENDS ${DOXYGEN_HTML_INDEX})
  add_custom_target(
    docs_forced
    COMMAND ${CMAKE_COMMAND} -E echo_append "Building API Documentation..."
    COMMAND ${DOXYGEN_EXECUTABLE} -u ${DOXYGEN_DOXYFILE}
    COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_DOXYFILE}
    COMMAND ${CMAKE_COMMAND} -E echo "Done."
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/doxygen
  )

  # Add the "view_docs" target that opens the generated API documentation.
  if(APPLE)
    set(OPEN_COMMAND "open")
  else()
    set(OPEN_COMMAND "xdg-open")
  endif()

  add_custom_target(view_docs "${OPEN_COMMAND}" "${DOXYGEN_HTML_INDEX}"
    DEPENDS "${DOXYGEN_HTML_INDEX}"
    COMMENT "Opening documentation in a web browser.")

endif()

#===============================================================================
# Build Instructions
#===============================================================================
message(STATUS "")
message(STATUS "[ Build Instructions ]")
if(MSVC)
  message(STATUS "To build targets : cmake --build . [--target <target> [, <target2>, ...]] [-j<num_core>]")
  message(STATUS "To run executable: cmake --build . --target RUN_<target>")
else()
  message(STATUS "To build targets : make [<target> [, <target2>, ...]] [-j<num_core>]")
  message(STATUS "To run executable: make RUN_<target>")
endif()
message(STATUS "Targets:")
message(STATUS "- 'ALL'          : build all the targets and run tests")
message(STATUS "- 'all'          : build core targets (w/o tests)")
message(STATUS "- 'tests'        : build tests")
message(STATUS "- 'test'         : run tests (build 'tests' first)")
message(STATUS "- 'tests_and_run': build tests and run them")
message(STATUS "- 'examples'     : build all the examples")
message(STATUS "- 'tutorials'    : build all the tutorials")
message(STATUS "- 'view_docs'    : open the API documentation")
message(STATUS "- 'install'      : install all the C++ components")
if(TARGET dartpy)
  message(STATUS "- 'dartpy'       : build dartpy")
  message(STATUS "- 'pytest'       : test dartpy")
endif()
if(TARGET coverage)
  message(STATUS "- 'coverage'     : generage coverage report")
  message(STATUS "- 'coverage_html': generage coverage report in html")
  message(STATUS "- 'coverage_view': view generaged coverage report in a browser")
endif()

#===============================================================================
# END
#===============================================================================
message(STATUS "")
