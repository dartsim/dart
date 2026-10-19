#===============================================================================
# CMake settings
#===============================================================================
cmake_minimum_required(VERSION 3.22.1)

project(dart)

string(TOUPPER ${PROJECT_NAME} PROJECT_NAME_UPPERCASE)

# DART 6 is a C++17 code base. Newer compilers default to C++20 or later, where
# some of this code is deprecated (for example the implicit `this` capture of
# `[=]` lambdas), so build DART itself as C++17 regardless of the compiler
# default. Consumers still get the `cxx_std_17` minimum from the exported
# targets.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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

# Use DART_SOURCE/BINARY_DIR instead of CMAKE_SOURCE/BINARY_DIR to support the
# case that DART is built as a sub-project in another project.
set(DART_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(DART_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR})

include(GNUInstallDirs)

# Variables used in Components.cmake
set(INCLUDE_INSTALL_DIR ${CMAKE_INSTALL_INCLUDEDIR})
set(LIBRARY_INSTALL_DIR ${CMAKE_INSTALL_LIBDIR})
set(CONFIG_INSTALL_DIR "${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake")

# Set relative location to install additional documentation (sample data,
# examples, and tutorials)
set(
  DART_ADDITIONAL_DOCUMENTATION_INSTALL_PATH
  "${CMAKE_INSTALL_DATAROOTDIR}/doc/${PROJECT_NAME}"
)

set(
  DART_EXAMPLES_INSTALL_PATH
  "${DART_ADDITIONAL_DOCUMENTATION_INSTALL_PATH}"
  CACHE PATH
  "Install destination (relative to CMAKE_INSTALL_PREFIX) for DART example sources. Set to an empty string to skip installing example sources."
)
mark_as_advanced(DART_EXAMPLES_INSTALL_PATH)

set(CMAKE_DEBUG_POSTFIX "d")
set(CMAKE_MODULE_PATH "${DART_SOURCE_DIR}/cmake")

include(DARTMacros)
include(dart_defs)
include(compiler_cache)

# CMake component helper. It uses the following variables:
# - LIBRARY_INSTALL_DIR
# - CONFIG_INSTALL_DIR
include(Components)
initialize_component_helpers(${PROJECT_NAME})

#===============================================================================
# Project settings
#===============================================================================

# Extract version numbers from package.xml
file(READ package.xml PACKAGE_XML)
string(
  REGEX MATCH
  "<version>([0-9]+)\\.([0-9]+)\\.([0-9]+)</version>"
  DIRTY_VERSION_STRING
  "${PACKAGE_XML}"
)
set(DART_VERSION "${CMAKE_MATCH_1}.${CMAKE_MATCH_2}.${CMAKE_MATCH_3}")
set(
  DART_VERSION_OVERRIDE
  ""
  CACHE STRING
  "CI-only DART version override (X.Y.Z); never use for packaging"
)
if(NOT DART_VERSION_OVERRIDE STREQUAL "")
  if(
    NOT
      DART_VERSION_OVERRIDE
        MATCHES
        "^(0|[1-9][0-9]*)\\.(0|[1-9][0-9]*)\\.(0|[1-9][0-9]*)$"
  )
    message(
      FATAL_ERROR
      "DART_VERSION_OVERRIDE must be X.Y.Z without leading zeros"
    )
  endif()
  if(DART_VERSION_OVERRIDE VERSION_LESS DART_VERSION)
    message(
      FATAL_ERROR
      "DART_VERSION_OVERRIDE must not be older than package.xml (${DART_VERSION})"
    )
  endif()
  message(STATUS "CI-only DART version override: ${DART_VERSION_OVERRIDE}")
  set(DART_VERSION "${DART_VERSION_OVERRIDE}")
endif()
string(REPLACE "." ";" DART_VERSION_PARTS "${DART_VERSION}")
list(GET DART_VERSION_PARTS 0 DART_MAJOR_VERSION)
list(GET DART_VERSION_PARTS 1 DART_MINOR_VERSION)
list(GET DART_VERSION_PARTS 2 DART_PATCH_VERSION)

set(DART_PKG_DESC "Dynamic Animation and Robotics Toolkit.")
set(
  DART_PKG_EXTERNAL_DEPS
  "assimp >= 5.2.0, ccd, eigen3 >= 3.4.0, fcl >= 0.7.0, fmt >= 8.1.1"
)

#===============================================================================
# Build options
#===============================================================================
if(MSVC)
  set(
    DART_RUNTIME_LIBRARY
    "/MD"
    CACHE STRING
    "MSVC runtime family used when DART_MSVC_FORCE_RELEASE_RUNTIME is OFF"
  )
  set_property(CACHE DART_RUNTIME_LIBRARY PROPERTY STRINGS /MD /MT)
  dart_option(
    DART_MSVC_DEFAULT_OPTIONS
    "Use Visual Studio default Debug/Release tuning instead of DART's MSVC tuning"
    OFF
  )
  # Pre-built package-manager dependencies are generally built with /MD. Keep
  # every configuration on that runtime by default to avoid LNK2038 mismatches.
  dart_option(
    DART_MSVC_FORCE_RELEASE_RUNTIME
    "Force /MD (Release DLL runtime) for all configurations to match pre-built packages"
    ON
  )
  # Whole-program optimization (/GL) defers code generation to a link-time /LTCG
  # pass that optimizes across every translation unit at once, which roughly
  # triples the MSVC Release build time. Keep it ON by default so shipped wheels
  # stay optimized, but let the CI C++ test build disable it to stay under the
  # Windows job timeout (the wheel jobs still exercise an optimized build).
  #
  # Seed the cache from the environment BEFORE dart_option() so CI can toggle it
  # for every cmake configure in the job without editing each pixi config task.
  # dart_option() also sets a normal (non-cache) variable of the same name, which
  # would shadow a cache value forced after it and leave the first configure on
  # the default; seeding first makes dart_option() normalize the intended value.
  # An explicit -D still applies when the environment variable is unset.
  if(DEFINED ENV{DART_MSVC_DISABLE_WHOLE_PROGRAM_OPTIMIZATION})
    set(
      DART_MSVC_DISABLE_WHOLE_PROGRAM_OPTIMIZATION
      "$ENV{DART_MSVC_DISABLE_WHOLE_PROGRAM_OPTIMIZATION}"
      CACHE BOOL
      "Disable MSVC whole-program optimization (/GL, /LTCG) in Release builds"
      FORCE
    )
  endif()
  dart_option(
    DART_MSVC_DISABLE_WHOLE_PROGRAM_OPTIMIZATION
    "Disable MSVC whole-program optimization (/GL, /LTCG) in Release builds"
    OFF
  )
  dart_configure_msvc_runtime_library()
else()
  dart_option(BUILD_SHARED_LIBS "Build shared libraries" ON)
endif()
dart_option(DART_BUILD_DARTPY "Build dartpy" OFF)
dart_option(DART_BUILD_UTILS_ASSETS "Build the versioned robot model retriever" OFF)
if(DEFINED DART_DARTPY_BINDER)
  if(NOT DART_DARTPY_BINDER STREQUAL "nanobind")
    message(
      FATAL_ERROR
      "DART 7.0 removed the pybind11 binder. dartpy now uses nanobind; remove DART_DARTPY_BINDER from your configuration."
    )
  endif()
  unset(DART_DARTPY_BINDER CACHE)
  unset(DART_DARTPY_BINDER)
endif()
# DART 7 removed the OpenSceneGraph GUI. The legacy DART_BUILD_GUI_OSG=OFF
# still selects a headless build; ON fails with migration text.
get_property(_dart_gui_osg_help CACHE DART_BUILD_GUI_OSG PROPERTY HELPSTRING)
if(DART_BUILD_GUI_OSG AND _dart_gui_osg_help STREQUAL "Build osgDart library")
  message(
    STATUS
    "Removing the cached legacy OSG GUI default; using DART_BUILD_GUI."
  )
  unset(DART_BUILD_GUI_OSG CACHE)
  unset(DART_BUILD_GUI_OSG)
endif()
unset(_dart_gui_osg_help)
if(DEFINED DART_BUILD_GUI_OSG)
  if(DART_BUILD_GUI_OSG)
    message(
      FATAL_ERROR
      "DART 7 removed the OpenSceneGraph GUI; use DART_BUILD_GUI for the Filament GUI, or set DART_BUILD_GUI_OSG=OFF for a headless build."
    )
  endif()
  set(
    DART_BUILD_GUI
    OFF
    CACHE BOOL
    "Build the Filament GUI and ImGui support"
    FORCE
  )
  # A normal variable from an embedding project would shadow the cache entry.
  set(DART_BUILD_GUI OFF)
  unset(DART_BUILD_GUI_OSG CACHE)
  unset(DART_BUILD_GUI_OSG)
endif()
dart_option(DART_BUILD_IO "Build model loading and file parsers" ON)
dart_option(DART_BUILD_GUI "Build the Filament GUI and ImGui support" ON)
dart_option(DART_BUILD_EXAMPLES "Add the C++ example targets" ON)
dart_option(DART_USE_SYSTEM_FILAMENT "Use an installed Filament SDK" ON)
if(DART_BUILD_DARTPY AND NOT DART_BUILD_IO)
  message(FATAL_ERROR "dartpy requires DART_BUILD_IO.")
endif()
if(DART_BUILD_UTILS_ASSETS AND NOT DART_BUILD_IO)
  message(FATAL_ERROR "DART_BUILD_UTILS_ASSETS requires DART_BUILD_IO.")
endif()
if(DART_BUILD_DARTPY AND NOT DART_BUILD_GUI)
  message(FATAL_ERROR "The all-in-one dartpy package requires DART_BUILD_GUI.")
endif()
dart_option(DART_ENABLE_GUI_SMOKE_TESTS "Build the off-screen GUI capture smoke test (needs a graphics device)" OFF)
dart_option(DART_BUILD_PROFILE "Build DART with profiling options" OFF)
dart_option(DART_PROFILE_BUILTIN "Enable the built-in text profiling backend" ON)
dart_option(DART_PROFILE_TRACY "Enable the Tracy profiling backend for local developer profiling" OFF)
# Keep legacy variable names in scope for consumers (the profile macros expect
# DART_PROFILE_ENABLE_*). Accept the legacy DART_PROFILE_TEXT spelling too.
if(DEFINED DART_PROFILE_TEXT)
  set(DART_PROFILE_BUILTIN "${DART_PROFILE_TEXT}")
endif()
set(DART_PROFILE_ENABLE_TEXT "${DART_PROFILE_BUILTIN}")
set(DART_PROFILE_ENABLE_TRACY "${DART_PROFILE_TRACY}")
dart_option(DART_CODECOV "Turn on codecov support" OFF)
# Warning: DART_ENABLE_SIMD should be ON only when you build DART and the DART
# dependent projects on the same machine. If this option is on, then compile
# option `-march=native` is added to the target `dart` that enables all
# instruction subsets supported by the local machine. If the architecture of
# local machines are different then the projects will be built with different
# compile options, which may cause runtime errors especially memory alignment
# errors.
dart_option(DART_ENABLE_SIMD
  "Build DART with all SIMD instructions on the current local machine" OFF
)
# Force the header-only dart/simd module to fall back to its scalar backend
# regardless of the instruction sets the compiler advertises (used by CI to
# exercise the scalar code path). Unrelated to DART_ENABLE_SIMD above.
dart_option(DART_SIMD_FORCE_SCALAR
  "Force the dart/simd module to use the scalar fallback backend" OFF
)
if(DART_SIMD_FORCE_SCALAR)
  add_compile_definitions(DART_SIMD_FORCE_SCALAR=1)
endif()
dart_option(DART_FAST_DEBUG "Add -O1 option for DEBUG mode build" OFF)
# GCC and Clang add ANSI-formatted colors when they detect the output medium is a
# terminal. However, this doesn't work in some cases such as when the makefile is
# invoked by Ninja. DART_FORCE_COLORED_OUTPUT can be used in this case to enforce
# to always generate ANSI-formatted colors regardless of the output medium types.
# See: https://medium.com/@alasher/colored-c-compiler-output-with-ninja-clang-gcc-10bfe7f2b949
dart_option(DART_FORCE_COLORED_OUTPUT
  "Always produce ANSI-colored output (GNU/Clang only)." OFF
)
dart_option(
  DART_USE_SYSTEM_FMT
  "Use system-installed fmt instead of building it from source (set OFF only when system fmt is unavailable or broken)"
  ON
)
dart_option(DART_USE_SYSTEM_ODE "Use system ODE" ON)
dart_option(DART_USE_SYSTEM_BULLET "Use system Bullet" ON)
dart_option(DART_BULLET_FLOAT64
  "Link system Bullet's double-precision (-float64) libraries when installed"
  ON
)
dart_option(DART_USE_SYSTEM_IMGUI "Use system ImGui" OFF)
# The bundled ImGui is always the docking release; drop the removed option.
if(DEFINED DART_IMGUI_DOCKING)
  message(
    STATUS
    "DART_IMGUI_DOCKING was removed: the bundled ImGui is the docking release"
  )
  unset(DART_IMGUI_DOCKING CACHE)
endif()
dart_option(DART_USE_SYSTEM_GOOGLEBENCHMARK "Use system GoogleBenchmark" OFF)
dart_option(DART_USE_SYSTEM_GOOGLETEST "Use system GoogleTest" OFF)
dart_option(DART_USE_SYSTEM_NANOBIND "Use system nanobind >= 3.1" OFF)
dart_option(
  DART_DARTPY_STABLE_ABI
  "Build the nanobind dartpy for the CPython 3.12+ stable ABI (abi3)"
  OFF
)
dart_option(DART_USE_SYSTEM_TRACY "Use system Tracy" OFF)
dart_option(
  DART_ALLOW_SINGLE_PRECISION_LIBCCD
  "Build even if FCL's libccd uses single precision"
  OFF
)
dart_option(DART_VERBOSE "Whether print detailed information in CMake process" OFF)

dart_configure_compiler_cache()

#===============================================================================
# Print intro
#===============================================================================
message(STATUS "")
message(STATUS "============================================")
message(STATUS "                DART ${DART_VERSION}")
message(STATUS "============================================")
message(STATUS "")

dart_print_options()

# Print build tool information
message(STATUS "[ Build Tools ]")
message(STATUS "- CMake               : ${CMAKE_VERSION}")
message(
  STATUS
  "- C++                 : ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}"
)
if(MSVC)
  message(STATUS "- CMAKE_TOOLCHAIN_FILE: ${CMAKE_TOOLCHAIN_FILE}")
endif()
message(STATUS "")

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
  target_compile_options(
    coverage_config
    INTERFACE
      -O0 # no optimization
      -g # generate debug info
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
  set(
    CMAKE_BUILD_TYPE
    Release
    CACHE STRING
    "Choose the type of build, options are: Debug | Release | RelWithDebInfo | MinSizeRel | None"
    FORCE
  )
endif()
string(TOUPPER ${CMAKE_BUILD_TYPE} CMAKE_BUILD_TYPE_UPPERCASE)

if(
  NOT
    "${CMAKE_BUILD_TYPE_UPPERCASE}"
      MATCHES
      "^(DEBUG|RELEASE|RELWITHDEBINFO|MINSIZEREL|NONE)$"
)
  message(
    WARNING
    "CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} unknown. Valid options are: Debug | Release | RelWithDebInfo | MinSizeRel | None"
  )
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
if("${CMAKE_BUILD_TYPE_UPPERCASE}" STREQUAL "DEBUG")
  set(
    DART_ACTIVE_LOG_LEVEL
    "DEBUG"
    CACHE STRING
    "Compile time active log level to enable"
  )
else()
  set(
    DART_ACTIVE_LOG_LEVEL
    "INFO"
    CACHE STRING
    "Compile time active log level to enable"
  )
endif()
set_property(
  CACHE DART_ACTIVE_LOG_LEVEL
  PROPERTY STRINGS TRACE DEBUG INFO WARN ERROR FATAL OFF
)

if("${CMAKE_BUILD_TYPE_UPPERCASE}" STREQUAL "DEBUG")
  option(DART_TREAT_WARNINGS_AS_ERRORS "Treat warnings as errors" OFF)
else()
  option(DART_TREAT_WARNINGS_AS_ERRORS "Treat warnings as errors" ON)
endif()

option(DART_BUILD_WHEELS "Indicate building dartpy for wheels" OFF)

dart_check_compiler_version()

# Configure MSVC compile policy before dependency discovery so any dependency
# targets created during configure inherit the same directory options.
if(MSVC)
  dart_configure_msvc_toolchain(
    REQUIRED_VERSION 1930
    REQUIRED_LABEL "Visual Studio 2022 v143"
  )
endif()

#===============================================================================
# Find dependencies
#===============================================================================
include(DARTFindDependencies)

#===============================================================================
# Check for non-case-sensitive filesystems
#===============================================================================
execute_process(
  COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/scripts/case_sensitive_filesystem
  RESULT_VARIABLE FILESYSTEM_CASE_SENSITIVE_RETURN
)
if(${FILESYSTEM_CASE_SENSITIVE_RETURN} EQUAL 0)
  set(FILESYSTEM_CASE_SENSITIVE TRUE)
else()
  set(FILESYSTEM_CASE_SENSITIVE FALSE)
endif()

#===============================================================================
# Compiler flags
#===============================================================================
dart_configure_compiler_options()

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
  message(STATUS "Build gui        : ${DART_BUILD_GUI}")
  message(STATUS "Install path     : ${CMAKE_INSTALL_PREFIX}")
  message(STATUS "CXX_FLAGS        : ${CMAKE_CXX_FLAGS}")
  if(${CMAKE_BUILD_TYPE_UPPERCASE} STREQUAL "RELEASE")
    message(STATUS "CXX_FLAGS_RELEASE: ${CMAKE_CXX_FLAGS_RELEASE}")
  elseif(${CMAKE_BUILD_TYPE_UPPERCASE} STREQUAL "DEBUG")
    message(STATUS "CXX_FLAGS_DEBUG  : ${CMAKE_CXX_FLAGS_DEBUG}")
  elseif(${CMAKE_BUILD_TYPE_UPPERCASE} STREQUAL "RELWITHDEBINFO")
    message(
      STATUS
      "CXX_FLAGS_RELWITHDEBINFO: ${CMAKE_CXX_FLAGS_RELWITHDEBINFO}"
    )
  elseif(${CMAKE_BUILD_TYPE_UPPERCASE} STREQUAL "PROFILE")
    message(STATUS "CXX_FLAGS_PROFILE: ${CMAKE_CXX_FLAGS_PROFILE}")
  endif()
  message(STATUS "DART_SOURCE_DIR : ${DART_SOURCE_DIR}")
  message(STATUS "DART_BINARY_DIR : ${DART_BINARY_DIR}")
endif(DART_VERBOSE)

#===============================================================================
# Add sub-directories
#===============================================================================

include(CTest)
add_subdirectory(dart)

set(DART_IN_SOURCE_BUILD TRUE)

if(TARGET dart)
  # Add a "tests" target to build unit tests.
  if(BUILD_TESTING)
    if(MSVC)
      add_subdirectory(tests)
    else()
      add_subdirectory(tests EXCLUDE_FROM_ALL)
    endif()
  endif()

  # Add example subdirectories and an "examples" target.
  if(NOT DART_BUILD_EXAMPLES)
    # Package builds can skip the examples entirely.
  elseif(MSVC)
    add_subdirectory(examples)
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
endif()

add_subdirectory(python)

# Add 'ALL' target that builds everything
set(all_target_candidates dartpy)
if(BUILD_TESTING)
  list(APPEND all_target_candidates tests_and_run pytest)
endif()
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
get_property(
  PACKAGE_INCLUDE_DIRS
  GLOBAL
  PROPERTY "${PROJECT_NAME_UPPERCASE}_INCLUDE_DIRS"
)

# Generate the DART CMake Config and version files
include(CMakePackageConfigHelpers)
set(
  DART_CONFIG_IN
  ${DART_SOURCE_DIR}/cmake/${PROJECT_NAME_UPPERCASE}Config.cmake.in
)
set(DART_CONFIG_OUT ${DART_BINARY_DIR}/${PROJECT_NAME_UPPERCASE}Config.cmake)
set(
  DART_VERSION_OUT
  ${DART_BINARY_DIR}/cmake/${PROJECT_NAME_UPPERCASE}ConfigVersion.cmake
)
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
configure_file(
  ${DART_SOURCE_DIR}/cmake/DARTConfigVersion.cmake.in
  ${DART_VERSION_OUT}
  @ONLY
)
install(
  FILES
    ${DART_CONFIG_OUT}
    ${DART_VERSION_OUT}
    "${DART_SOURCE_DIR}/cmake/DARTFindPackageVersion.cmake"
    # Removed-component stub that reports the OSG GUI migration.
    "${DART_SOURCE_DIR}/cmake/dart_gui-osgComponent.cmake"
    "${DART_SOURCE_DIR}/cmake/dart_gui-osgTargets.cmake"
  DESTINATION "${CONFIG_INSTALL_DIR}"
)

# Generate the DART pkg-config
set(DART_PKG_CFLAGS "${CMAKE_CXX_FLAGS}")
set(DART_PKG_PRIVATE_LIBS)
foreach(library ${CMAKE_DL_LIBS})
  string(APPEND DART_PKG_PRIVATE_LIBS " -l${library}")
endforeach()
if(HAVE_OCTOMAP)
  string(APPEND DART_PKG_EXTERNAL_DEPS ", octomap >= 1.9.7")
endif()
if(spdlog_FOUND)
  string(APPEND DART_PKG_EXTERNAL_DEPS ", spdlog >= 1.9.2")
endif()
if(HAVE_BULLET)
  set(bullet_pc bullet)
  if(BT_USE_DOUBLE_PRECISION)
    string(APPEND DART_PKG_CFLAGS " -DBT_USE_DOUBLE_PRECISION")
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
      pkg_check_modules(BULLET_FLOAT64 QUIET bullet-float64)
    endif()
    if(BULLET_FLOAT64_FOUND)
      set(bullet_pc bullet-float64)
    elseif(DART_BULLET_USES_FLOAT64)
      # bullet.pc would add the single-precision libraries; name the -float64
      # ones directly.
      set(bullet_pc)
      string(APPEND DART_PKG_CFLAGS "${DART_BULLET_PC_CFLAGS}")
      string(APPEND DART_PKG_PRIVATE_LIBS "${DART_BULLET_PC_LIBS}")
    endif()
  endif()
  if(bullet_pc)
    set(bullet_pc_minimum 3.05)
    if(DART_BULLET_MINIMUM_VERSION)
      set(bullet_pc_minimum ${DART_BULLET_MINIMUM_VERSION})
    endif()
    string(
      APPEND DART_PKG_EXTERNAL_DEPS
      ", ${bullet_pc} >= ${bullet_pc_minimum}"
    )
  endif()
endif()
if(HAVE_ODE)
  string(APPEND DART_PKG_EXTERNAL_DEPS ", ode >= 0.16.2")
endif()
set(PC_CONFIG_IN ${DART_SOURCE_DIR}/cmake/dart.pc.in)
set(PC_CONFIG_OUT ${DART_BINARY_DIR}/cmake/dart.pc)
cmake_path(IS_ABSOLUTE CMAKE_INSTALL_LIBDIR ABSOLUTE_CMAKE_INSTALL_LIBDIR)
if(ABSOLUTE_CMAKE_INSTALL_LIBDIR)
  # Absolute "CMAKE_INSTALL_*DIR" can be provided by user
  # eg. to install libs and headers in different prefixes
  # to split the package between propagated runtime dependency package
  # and buildtime-only -dev package.
  # In that case, the package can not be relocatable,
  # and the installed files must not contain any "CMAKE_INSTALL_PREFIX"
  # concatenated with a "CMAKE_INSTALL_*DIR". Use "CMAKE_INSTALL_FULL_*DIR".
  set(PC_CONFIG_INSTALL_DIR ${CMAKE_INSTALL_FULL_LIBDIR}/pkgconfig)
  set(PC_LIBDIR ${CMAKE_INSTALL_FULL_LIBDIR})
  set(PC_INCLUDEDIR ${CMAKE_INSTALL_FULL_INCLUDEDIR})
else()
  # When "CMAKE_INSTALL_*DIR" are relative, we should ensure the installed
  # files contain relocatable references to install paths.
  set(
    PC_CONFIG_INSTALL_DIR
    ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/pkgconfig
  )
  set(PC_LIBDIR "\${prefix}/${CMAKE_INSTALL_LIBDIR}")
  set(PC_INCLUDEDIR "\${prefix}/${CMAKE_INSTALL_INCLUDEDIR}")
endif()
file(
  RELATIVE_PATH
  RELATIVE_PATH_TO_INSTALL_PREFIX
  "${PC_CONFIG_INSTALL_DIR}"
  "${CMAKE_INSTALL_PREFIX}"
)
if(DART_VERBOSE)
  message(STATUS ${PC_CONFIG_OUT})
endif()
configure_file(${PC_CONFIG_IN} ${PC_CONFIG_OUT} @ONLY)
install(FILES ${PC_CONFIG_OUT} DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig)

# Install a Catkin 'package.xml' file. This is required by REP-136.
install(
  FILES package.xml
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}
)

#===============================================================================
# Install sample data, examples, and tutorials
#===============================================================================

# Sample data
install(
  DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/data"
  DESTINATION ${DART_ADDITIONAL_DOCUMENTATION_INSTALL_PATH}
)

# Examples source
if(NOT DART_EXAMPLES_INSTALL_PATH STREQUAL "")
  install(
    DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/examples"
    DESTINATION "${DART_EXAMPLES_INSTALL_PATH}"
  )
endif()

# Tutorials source
install(
  DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/python/tutorials"
  DESTINATION ${DART_ADDITIONAL_DOCUMENTATION_INSTALL_PATH}
  PATTERN "__pycache__" EXCLUDE
)

#===============================================================================
# Uninstall
#===============================================================================

# Add an "uninstall" target
# https://gitlab.kitware.com/cmake/community/-/wikis/doc/cmake/recipe/AddUninstallTarget
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/uninstall_target.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/cmake/uninstall_target.cmake"
  IMMEDIATE
  @ONLY
)
add_custom_target(
  uninstall
  "${CMAKE_COMMAND}" -P
  "${CMAKE_CURRENT_BINARY_DIR}/cmake/cmake_uninstall.cmake"
)

#===============================================================================
# Code Formatting
#===============================================================================
if(DART_VERBOSE)
  message(STATUS "")
  message(STATUS "[ Code Formatting ]")
endif()

if(CLANG_FORMAT_EXECUTABLE)
  get_filename_component(_clang_format_name "${CLANG_FORMAT_EXECUTABLE}" NAME)
  if(
    NOT EXISTS "${CLANG_FORMAT_EXECUTABLE}"
    OR NOT _clang_format_name STREQUAL "clang-format-14"
  )
    unset(CLANG_FORMAT_EXECUTABLE CACHE)
  endif()
endif()

find_program(CLANG_FORMAT_EXECUTABLE NAMES clang-format-14)

get_property(formatting_files GLOBAL PROPERTY DART_FORMAT_FILES)
# GUI sources register before their build gates; GUI builds register again.
list(REMOVE_DUPLICATES formatting_files)
list(LENGTH formatting_files formatting_files_length)

if(CLANG_FORMAT_EXECUTABLE)
  if(DART_VERBOSE)
    message(STATUS "Looking for clang-format - found")
  endif()

  message(STATUS "Formatting on ${formatting_files_length} source files.")

  if(formatting_files)
    add_custom_target(
      format
      COMMAND
        ${CMAKE_COMMAND} -E echo
        "Formatting ${formatting_files_length} files... "
      COMMAND ${CLANG_FORMAT_EXECUTABLE} -style=file -i ${formatting_files}
      COMMAND ${CMAKE_COMMAND} -E echo "Done."
      DEPENDS ${CLANG_FORMAT_EXECUTABLE}
      WORKING_DIRECTORY ${DART_SOURCE_DIR}/dart
    )

    add_custom_target(
      check-format
      COMMAND
        ${CMAKE_COMMAND} -E echo "Checking ${formatting_files_length} files... "
      COMMAND
        ${DART_SOURCE_DIR}/scripts/check_format.sh ${CLANG_FORMAT_EXECUTABLE}
        ${formatting_files}
      COMMAND ${CMAKE_COMMAND} -E echo "Done."
      DEPENDS ${CLANG_FORMAT_EXECUTABLE}
      WORKING_DIRECTORY ${DART_SOURCE_DIR}/dart
    )
  else()
    add_custom_target(
      format
      COMMAND
        ${CMAKE_COMMAND} -E echo
        "Warning: Not found any source files to format."
    )

    add_custom_target(
      check-format
      COMMAND
        ${CMAKE_COMMAND} -E echo "Warning: Not found any source files to check."
    )
  endif()
else()
  if(DART_VERBOSE)
    message(
      WARNING
      "Looking for clang-format - NOT found, please install clang-format to enable automatic code formatting"
    )
  endif()
endif()

#===============================================================================
# API Document using Doxygen
# References:
#   http://mementocodex.wordpress.com/2013/01/19/how-to-generate-code-documentation-with-doxygen-and-cmake-a-slightly-improved-approach/
#   http://www.cmake.org/pipermail/cmake/2007-February/012796.html
#===============================================================================
if(DOXYGEN_FOUND)
  set(DOXYGEN_DOXYFILE_IN "${PROJECT_SOURCE_DIR}/docs/doxygen/Doxyfile.in")
  set(DOXYGEN_DOXYFILE "${PROJECT_BINARY_DIR}/docs/doxygen/Doxyfile")
  set(DOXYGEN_HTML_INDEX "${PROJECT_BINARY_DIR}/docs/doxygen/html/index.html")
  set(DOXYGEN_OUTPUT_ROOT "${PROJECT_BINARY_DIR}/docs/doxygen/html")
  set(DOXYGEN_GENERATE_TAGFILE "${DOXYGEN_OUTPUT_ROOT}/${PROJECT_NAME}.tag")
  set(DOXYGEN_INCLUDE_PATH "${PROJECT_SOURCE_DIR}")
  set(DOXYGEN_INPUT_ROOT "${PROJECT_SOURCE_DIR}/dart")
  set(DOXYGEN_EXTRA_INPUTS "${PROJECT_SOURCE_DIR}/docs/doxygen/mainpage.dox")
  # No in-tree third-party sources to exclude: the bundled ImGui is fetched into
  # the build tree (or taken from the system package), not vendored under dart/.
  set(DOXYGEN_EXCLUDE "${PROJECT_SOURCE_DIR}/dart/gui/app")
  set(DOXYGEN_STRIP_FROM_PATH "${CMAKE_CURRENT_SOURCE_DIR}")

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
    COPY "${PROJECT_SOURCE_DIR}/docs/doxygen/DART logo.png"
    DESTINATION ${DOXYGEN_OUTPUT_ROOT}
  )
  add_custom_command(
    OUTPUT ${DOXYGEN_HTML_INDEX}
    COMMAND ${CMAKE_COMMAND} -E echo_append "Building API Documentation..."
    COMMAND ${DOXYGEN_EXECUTABLE} -u ${DOXYGEN_DOXYFILE}
    COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_DOXYFILE}
    # Strip path prefix from all paths in dart.tag
    COMMAND
      ${CMAKE_COMMAND} -E echo "Stripping paths from"
      "${DOXYGEN_GENERATE_TAGFILE}"
    COMMAND sed -i s:${DOXYGEN_STRIP_FROM_PATH}::g ${DOXYGEN_GENERATE_TAGFILE}
    # Strip all doxygen="path" HTML tags
    COMMAND ${CMAKE_COMMAND} -E echo "Stripping Doxygen HTML tags"
    COMMAND
      find "${DOXYGEN_OUTPUT_ROOT}" -type f -name "*.html" -exec sed -i 's:
      doxygen=\"[^\"]*\"::g' {} \\$<SEMICOLON>
    COMMAND ${CMAKE_COMMAND} -E echo "Done."
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/docs/doxygen
    DEPENDS ${DOXYGEN_DOXYFILE}
  )
  add_custom_target(docs DEPENDS ${DOXYGEN_HTML_INDEX})
  add_custom_target(
    docs_forced
    COMMAND ${CMAKE_COMMAND} -E echo_append "Building API Documentation..."
    COMMAND ${DOXYGEN_EXECUTABLE} -u ${DOXYGEN_DOXYFILE}
    COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_DOXYFILE}
    COMMAND ${CMAKE_COMMAND} -E echo "Done."
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/docs/doxygen
  )

  # Add the "view_docs" target that opens the generated API documentation.
  if(APPLE)
    set(OPEN_COMMAND "open")
  else()
    set(OPEN_COMMAND "xdg-open")
  endif()

  add_custom_target(
    view_docs
    "${OPEN_COMMAND}" "${DOXYGEN_HTML_INDEX}"
    DEPENDS "${DOXYGEN_HTML_INDEX}"
    COMMENT "Opening documentation in a web browser."
  )
endif()

#===============================================================================
# Build Instructions
#===============================================================================
message(STATUS "")
message(STATUS "Run 'make' to build all the components")
if(BUILD_TESTING)
  message(STATUS "Run 'make tests' to build all the tests")
endif()
message(STATUS "Run 'make examples' to build all the examples")
message(STATUS "Run 'make view_docs' to see the API documentation")
message(STATUS "Run 'make install' to install all the C++ components")
if(TARGET dartpy)
  message(STATUS "Run 'make dartpy' to build dartpy")
  message(STATUS "Run 'make install' to install dartpy")
endif()
if(TARGET coverage)
  message(STATUS "- 'coverage'     : generate coverage report")
  message(STATUS "- 'coverage_html': generate coverage report in html")
  message(
    STATUS
    "- 'coverage_view': view generated coverage report in a browser"
  )
endif()

#===============================================================================
# END
#===============================================================================
message(STATUS "")
