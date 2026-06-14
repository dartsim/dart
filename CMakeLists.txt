#===============================================================================
# CMake settings
#===============================================================================
cmake_minimum_required(VERSION 4.2.3)

project(dart)

string(TOUPPER ${PROJECT_NAME} PROJECT_NAME_UPPERCASE)

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

include(dart_defs)
include(compiler_cache)
include(local_build_perf)

# Initialize CMake component helpers (now part of dart_defs.cmake)
# Uses the following variables:
# - LIBRARY_INSTALL_DIR
# - CONFIG_INSTALL_DIR
initialize_component_helpers(${PROJECT_NAME})

if(BUILD_SHARED_LIBS)
  set(DART_BUILD_SHARED_VALUE 1)
else()
  set(DART_BUILD_SHARED_VALUE 0)
endif()

#===============================================================================
# Project settings
#===============================================================================

# Extract version numbers from package.xml
# Supports both release versions (X.Y.Z) and dev versions (X.Y.Z.devN)
file(READ package.xml PACKAGE_XML)
string(
  REGEX MATCH
  "<version>[0-9]+\\.[0-9]+\\.[0-9]+(\\.(dev|alpha|beta|rc)[0-9]+)?</version>"
  DIRTY_VERSION_STRING
  "${PACKAGE_XML}"
)
# Remove <version> tags to get clean version string
string(
  REGEX REPLACE
  "^<version>([0-9]+\\.[0-9]+\\.[0-9]+(\\.(dev|alpha|beta|rc)[0-9]+)?)</version>$"
  "\\1"
  DART_VERSION_FULL
  "${DIRTY_VERSION_STRING}"
)
# Extract numeric components
string(
  REGEX REPLACE
  "^([0-9]+)\\.([0-9]+)\\.([0-9]+)(\\.(dev|alpha|beta|rc)[0-9]+)?$"
  "\\1"
  DART_MAJOR_VERSION
  "${DART_VERSION_FULL}"
)
string(
  REGEX REPLACE
  "^([0-9]+)\\.([0-9]+)\\.([0-9]+)(\\.(dev|alpha|beta|rc)[0-9]+)?$"
  "\\2"
  DART_MINOR_VERSION
  "${DART_VERSION_FULL}"
)
string(
  REGEX REPLACE
  "^([0-9]+)\\.([0-9]+)\\.([0-9]+)(\\.(dev|alpha|beta|rc)[0-9]+)?$"
  "\\3"
  DART_PATCH_VERSION
  "${DART_VERSION_FULL}"
)
# Extract suffix if present (e.g., "dev0" from "7.0.0.dev0")
string(
  REGEX MATCH
  "\\.(dev|alpha|beta|rc)[0-9]+"
  DART_VERSION_SUFFIX
  "${DART_VERSION_FULL}"
)
if(DART_VERSION_SUFFIX)
  string(SUBSTRING "${DART_VERSION_SUFFIX}" 1 -1 DART_VERSION_SUFFIX) # Remove leading dot
else()
  set(DART_VERSION_SUFFIX "")
endif()
# DART_VERSION: Short version for CMake compatibility (X.Y.Z)
set(
  DART_VERSION
  "${DART_MAJOR_VERSION}.${DART_MINOR_VERSION}.${DART_PATCH_VERSION}"
)
# DART_VERSION_FULL: Complete version including suffix (X.Y.Z or X.Y.Z.devN)
# Already set above

set(DART_PKG_DESC "Dynamic Animation and Robotics Toolkit.")
set(DART_PKG_EXTERNAL_DEPS "assimp, eigen3, entt")
set_property(GLOBAL PROPERTY DART_PKG_LINK_LIBS "")
set_property(GLOBAL PROPERTY DART_PUBLIC_INCLUDE_DIRS "")
set_property(GLOBAL PROPERTY DART_PUBLIC_SYSTEM_INCLUDE_DIRS "")
set_property(GLOBAL PROPERTY DART_EXTRA_LINK_LIBS "")
set_property(GLOBAL PROPERTY DART_EXTRA_COMPILE_OPTIONS "")

function(_dart_pkgconfig_collect output_var)
  set(_converted "")
  set(_expect_framework FALSE)
  foreach(_lib IN LISTS ARGN)
    if(NOT _lib)
      continue()
    endif()
    if(_expect_framework)
      list(APPEND _converted "-framework" "${_lib}")
      set(_expect_framework FALSE)
      continue()
    endif()
    if(
      _lib STREQUAL "optimized"
      OR _lib STREQUAL "debug"
      OR _lib STREQUAL "general"
    )
      continue()
    elseif(_lib STREQUAL "-framework")
      set(_expect_framework TRUE)
      continue()
    elseif(TARGET "${_lib}")
      get_target_property(_iface "${_lib}" INTERFACE_LINK_LIBRARIES)
      if(_iface)
        _dart_pkgconfig_collect(_nested ${_iface})
        list(APPEND _converted ${_nested})
      endif()
      get_target_property(_imported "${_lib}" IMPORTED_LOCATION)
      if(NOT _imported)
        get_target_property(_imported "${_lib}" IMPORTED_IMPLIB)
      endif()
      if(_imported)
        list(APPEND _converted "${_imported}")
      endif()
    elseif(_lib MATCHES "^-.*")
      list(APPEND _converted "${_lib}")
    elseif(IS_ABSOLUTE "${_lib}")
      list(APPEND _converted "${_lib}")
    else()
      list(APPEND _converted "-l${_lib}")
    endif()
  endforeach()
  set("${output_var}" "${_converted}" PARENT_SCOPE)
endfunction()

function(dart_pkgconfig_append_libraries)
  if(NOT ARGN)
    return()
  endif()
  _dart_pkgconfig_collect(_converted ${ARGN})
  if(_converted)
    set_property(GLOBAL APPEND PROPERTY DART_PKG_LINK_LIBS ${_converted})
  endif()
endfunction()

#===============================================================================
# Build options
#===============================================================================

# Simulation options
option(
  DART_SIMULATION_VERBOSE
  "Show detailed simulation dependency information"
  OFF
)

if(MSVC)
  set(
    DART_RUNTIME_LIBRARY
    "/MD"
    CACHE STRING
    "BaseName chosen by the user at CMake configure time"
  )
  set_property(CACHE DART_RUNTIME_LIBRARY PROPERTY STRINGS /MD /MT)
  dart_option(DART_MSVC_DEFAULT_OPTIONS "Build DART with default Visual Studio options" OFF CATEGORY msvc)

  # Option to force /MD (Release runtime) for all configurations including Debug.
  # This is useful when linking against pre-built libraries (e.g., conda-forge packages)
  # that are built with /MD to avoid LNK2038 runtime library mismatch errors.
  # Default: ON (recommended for most users using package managers like conda/pixi)
  # Set to OFF if you're building all dependencies from source with matching runtime libraries.
  dart_option(DART_MSVC_FORCE_RELEASE_RUNTIME
    "Force /MD (Release DLL runtime) for all configurations to match pre-built packages (conda-forge, vcpkg, etc.)" ON
    CATEGORY msvc
  )

  if(DART_MSVC_FORCE_RELEASE_RUNTIME)
    # Use CMAKE_MSVC_RUNTIME_LIBRARY so all configurations match pre-built
    # package-manager dependencies.
    # This sets /MD for all configurations (Debug, Release, RelWithDebInfo, MinSizeRel)
    set(
      CMAKE_MSVC_RUNTIME_LIBRARY
      "MultiThreadedDLL"
      CACHE STRING
      "MSVC runtime library - forced to /MD for all configs"
      FORCE
    )
    message(
      STATUS
      "DART_MSVC_FORCE_RELEASE_RUNTIME=ON: Using /MD for all configurations"
    )
    message(
      STATUS
      "  This avoids LNK2038 errors when linking pre-built packages built with /MD"
    )
    message(
      STATUS
      "  Set -DDART_MSVC_FORCE_RELEASE_RUNTIME=OFF if building all deps from source"
    )
  endif()
else()
  dart_option(BUILD_SHARED_LIBS "Build shared libraries" ON CATEGORY build)
endif()
dart_option(DART_BUILD_DARTPY "Build dartpy" OFF CATEGORY build)
set(_dart_default_gui OFF)
if(
  CMAKE_SYSTEM_NAME STREQUAL "Linux"
  AND CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|AMD64)$"
)
  set(_dart_default_gui ON)
endif()
dart_option(DART_BUILD_GUI "Build the Filament-backed GUI library" ${_dart_default_gui} CATEGORY build)
if(DEFINED DART_BUILD_GUI_RAYLIB AND DART_BUILD_GUI_RAYLIB)
  message(
    FATAL_ERROR
    "DART_BUILD_GUI_RAYLIB has been removed. Use the Filament-backed DART GUI instead."
  )
endif()
if(DEFINED DART_BUILD_RAYLIB AND DART_BUILD_RAYLIB)
  message(
    FATAL_ERROR
    "DART_BUILD_RAYLIB has been removed. Use the Filament-backed DART GUI instead."
  )
endif()
if(DEFINED DART_USE_SYSTEM_RAYLIB AND DART_USE_SYSTEM_RAYLIB)
  message(
    FATAL_ERROR
    "DART_USE_SYSTEM_RAYLIB has been removed with the Raylib GUI experiment."
  )
endif()
if(DEFINED DART_RAYLIB_GIT_TAG AND NOT DART_RAYLIB_GIT_TAG STREQUAL "")
  message(
    FATAL_ERROR
    "DART_RAYLIB_GIT_TAG has been removed with the Raylib GUI experiment."
  )
endif()
unset(DART_BUILD_GUI_RAYLIB CACHE)
unset(DART_BUILD_RAYLIB CACHE)
unset(DART_USE_SYSTEM_RAYLIB CACHE)
unset(DART_RAYLIB_GIT_TAG CACHE)
set(DART_BUILD_GUI_RAYLIB OFF)
unset(_dart_user_set_build_gui_raylib)
dart_option(DART_STRICT_SYMBOL_VISIBILITY
  "Build shared libraries with hidden default symbol visibility so exported public ABI must use component *_API macros" OFF
  CATEGORY build
)
dart_option(
  DART_COLLISION_DEPRECATE_LEGACY_NAMES
  "Emit deprecation warnings for legacy collision detector names that now route to the built-in DART detector"
  ON
  CATEGORY build
)
dart_option(
  DART_BUILD_COLLISION_REFERENCE_TESTS
  "Build collision tests that use FCL, Bullet, or ODE as reference engines"
  OFF
  CATEGORY build
)
dart_option(
  DART_BUILD_COLLISION_REFERENCE_BENCHMARKS
  "Build collision benchmarks that compare native collision against FCL, Bullet, or ODE"
  OFF
  CATEGORY build
)
set(_dart_build_collision_references OFF)
if(
  DART_BUILD_COLLISION_REFERENCE_TESTS
  OR DART_BUILD_COLLISION_REFERENCE_BENCHMARKS
)
  set(_dart_build_collision_references ON)
endif()
dart_option(DART_BUILD_EXAMPLES "Build example targets" ON CATEGORY build)
dart_option(DART_BUILD_TUTORIALS "Build tutorial targets" ON CATEGORY build)
dart_option(DART_BUILD_TESTS "Build C++ tests" ON CATEGORY build)
dart_option(DART_ENABLE_GUI_FILAMENT_SMOKE_TESTS
  "Register opt-in headless smoke tests for the DART GUI executable" OFF
  CATEGORY diagnostics
)
dart_option(DART_BUILD_PROFILE "Build DART with profiling options" OFF CATEGORY performance)
dart_option(DART_PROFILE_BUILTIN "Enable built-in text profiling backend" ON CATEGORY performance)
dart_option(DART_PROFILE_TRACY "Enable Tracy profiling backend for local developer profiling" OFF CATEGORY performance)
dart_option(DART_CODECOV "Turn on codecov support" OFF CATEGORY performance)

# Backward compatibility: accept legacy DART_PROFILE_TEXT if provided.
if(NOT DEFINED DART_PROFILE_BUILTIN AND DEFINED DART_PROFILE_TEXT)
  set(DART_PROFILE_BUILTIN "${DART_PROFILE_TEXT}")
endif()

# Keep legacy variable names in scope for consumers (macros expect ENABLE_*).
set(DART_PROFILE_TEXT "${DART_PROFILE_BUILTIN}")
set(DART_PROFILE_ENABLE_TEXT "${DART_PROFILE_BUILTIN}")
set(DART_PROFILE_ENABLE_TRACY "${DART_PROFILE_TRACY}")
# Warning: DART_ENABLE_SIMD should be ON only when you build DART and the DART
# dependent projects on the same machine. If this option is on, then compile
# option `-march=native` is added to the target `dart` that enables all
# instruction subsets supported by the local machine. If the architecture of
# local machines are different then the projects will be built with different
# compile options, which may cause runtime errors especially memory alignment
# errors.
dart_option(DART_ENABLE_SIMD
  "Build DART with all SIMD instructions on the current local machine" OFF CATEGORY performance
)
dart_option(DART_SIMD_FORCE_SCALAR
  "Force dart::simd to use scalar fallback (for testing)" OFF CATEGORY performance
)
dart_option(DART_FAST_DEBUG "Add -O1 option for DEBUG mode build" OFF CATEGORY performance)
dart_option(DART_ENABLE_ASAN "Build DART with AddressSanitizer instrumentation" OFF CATEGORY diagnostics)
# GCC and Clang add ANSI-formatted colors when they detect the output medium is a
# terminal. However, this doesn't work in some cases such as when the makefile is
# invoked by Ninja. DART_FORCE_COLORED_OUTPUT can be used in this case to enforce
# to always generate ANSI-formatted colors regardless of the output medium types.
# See: https://medium.com/@alasher/colored-c-compiler-output-with-ninja-clang-gcc-10bfe7f2b949
dart_option(DART_FORCE_COLORED_OUTPUT
  "Always produce ANSI-colored output (GNU/Clang only)." OFF CATEGORY diagnostics
)
dart_option(DART_USE_SYSTEM_IMGUI
  "Use system-installed ImGui instead of fetching from GitHub (recommended for package distributions)" OFF CATEGORY system
)
dart_option(DART_USE_SYSTEM_FILAMENT
  "Use system-installed Filament instead of an explicit fetched copy (recommended for package distributions)" ON CATEGORY system
)
set(_dart_fetch_filament_default OFF)
if(
  CMAKE_SYSTEM_NAME STREQUAL "Linux"
  AND CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|AMD64)$"
)
  set(_dart_fetch_filament_default ON)
endif()
dart_option(DART_FETCH_FILAMENT
  "Fetch the pinned Filament archive when a packaged Filament install is unavailable" ${_dart_fetch_filament_default} CATEGORY system
)
unset(_dart_default_gui)
unset(_dart_fetch_filament_default)
set(
  DART_FILAMENT_VERSION
  "1.71.3"
  CACHE STRING
  "Filament version for future explicit fetch support"
)
mark_as_advanced(DART_FILAMENT_VERSION)
dart_option(DART_USE_SYSTEM_GOOGLEBENCHMARK "Use system GoogleBenchmark" OFF CATEGORY system)
dart_option(DART_USE_SYSTEM_GOOGLETEST "Use system GoogleTest" OFF CATEGORY system)
dart_option(DART_USE_SYSTEM_NANOBIND "Use system nanobind" OFF CATEGORY system)
dart_option(DART_USE_SYSTEM_TRACY "Use system Tracy" OFF CATEGORY system)
dart_option(DART_USE_SYSTEM_ODE
  "Use system-installed ODE instead of fetching from Bitbucket (set OFF only when system ODE is unavailable)" ON CATEGORY system
)
dart_option(DART_USE_SYSTEM_BULLET
  "Use system-installed Bullet instead of fetching from GitHub (set OFF only when system Bullet is unavailable)" ON CATEGORY system
)
dart_option(DART_VERBOSE "Whether print detailed information in CMake process" OFF CATEGORY diagnostics)
dart_option(
  DART_ENABLE_SDFORMAT
  "Enable sdformat support in dart-utils (disable on platforms without sdformat)"
  ON
  CATEGORY build
)
dart_option(
  DART_ENABLE_EXPERIMENTAL_CUDA
  "Build opt-in CUDA support for dart::simulation"
  OFF
  CATEGORY build
)
dart_option(
  DART_BUILD_DIFF
  "Build opt-in differentiable simulation support for dart::simulation"
  OFF
  CATEGORY build
)
set(
  DART_CUDA_ARCHITECTURES
  ""
  CACHE STRING
  "CUDA architectures for experimental CUDA targets; leave empty for CMake/nvcc default"
)

if(DART_ENABLE_EXPERIMENTAL_CUDA)
  enable_language(CUDA)
endif()

dart_configure_compiler_cache()
dart_configure_linker()
dart_configure_path_normalization()

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
  set(
    DART_COVERAGE_COMPILE_OPTIONS
    -O0 # no optimization
    -g # generate debug info
    --coverage # sets all required flags
  )
  set(DART_COVERAGE_LINK_OPTIONS --coverage)

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
    INTERFACE ${DART_COVERAGE_COMPILE_OPTIONS}
  )

  # Add required flags (GCC & LLVM/Clang)
  target_link_options(coverage_config INTERFACE ${DART_COVERAGE_LINK_OPTIONS})

  # Export CodeCov configuration
  install(TARGETS coverage_config DESTINATION lib EXPORT coverage_config)
  install(EXPORT coverage_config DESTINATION ${CONFIG_INSTALL_DIR})
endif()

#===============================================================================
# Build type settings
#===============================================================================
set(
  _dart_known_build_types
  Debug
  Release
  RelWithDebInfo
  MinSizeRel
  None
)
if(NOT CMAKE_CONFIGURATION_TYPES)
  if(NOT CMAKE_BUILD_TYPE)
    set(
      CMAKE_BUILD_TYPE
      Release
      CACHE STRING
      "Choose the type of build, options are: Debug | Release | RelWithDebInfo | MinSizeRel | None"
      FORCE
    )
    message(STATUS "Setting CMAKE_BUILD_TYPE to '${CMAKE_BUILD_TYPE}'.")
  else()
    set(
      CMAKE_BUILD_TYPE
      "${CMAKE_BUILD_TYPE}"
      CACHE STRING
      "Choose the type of build, options are: Debug | Release | RelWithDebInfo | MinSizeRel | None"
      FORCE
    )
  endif()
  set_property(
    CACHE CMAKE_BUILD_TYPE
    PROPERTY STRINGS ${_dart_known_build_types}
  )
endif()
string(TOUPPER "${CMAKE_BUILD_TYPE}" CMAKE_BUILD_TYPE_UPPERCASE)

set(DART_BUILD_MODE_DEBUG FALSE)
set(DART_BUILD_MODE_RELEASE FALSE)

if("${CMAKE_BUILD_TYPE_UPPERCASE}" STREQUAL "DEBUG")
  set(DART_BUILD_MODE_DEBUG TRUE)
elseif("${CMAKE_BUILD_TYPE_UPPERCASE}" STREQUAL "RELEASE")
  set(DART_BUILD_MODE_RELEASE TRUE)
elseif("${CMAKE_BUILD_TYPE_UPPERCASE}" STREQUAL "RELWITHDEBINFO")
  set(DART_BUILD_MODE_RELEASE TRUE)
elseif("${CMAKE_BUILD_TYPE_UPPERCASE}" STREQUAL "MINSIZEREL")
  set(DART_BUILD_MODE_RELEASE TRUE)
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
if(DART_BUILD_MODE_DEBUG)
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

if(DART_BUILD_MODE_DEBUG)
  option(DART_TREAT_WARNINGS_AS_ERRORS "Treat warnings as errors" OFF)
else()
  option(DART_TREAT_WARNINGS_AS_ERRORS "Treat warnings as errors" ON)
endif()

option(DART_BUILD_WHEELS "Indicate building dartpy for wheels" OFF)

#===============================================================================
# Uninstall
#===============================================================================

# Add an "uninstall" target early so that fetched dependencies
# don't create their own conflicting target.
# https://gitlab.kitware.com/cmake/community/-/wikis/doc/cmake/recipe/AddUninstallTarget
if(NOT TARGET uninstall)
  file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/cmake")
  configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/uninstall_target.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake/cmake_uninstall.cmake"
    IMMEDIATE
    @ONLY
  )
  add_custom_target(
    uninstall
    COMMAND
      "${CMAKE_COMMAND}" -P
      "${CMAKE_CURRENT_BINARY_DIR}/cmake/cmake_uninstall.cmake"
  )
endif()

#===============================================================================
# Find dependencies
#===============================================================================
include(dart_find_dependencies)

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
if(MSVC)
  # Visual Studio 2026 required for supported Windows builds.
  set(msvc_required_version 1950)
  if(
    MSVC_VERSION VERSION_LESS ${msvc_required_version}
    AND ${CMAKE_CXX_COMPILER_ID} STREQUAL "MSVC"
  )
    message(
      FATAL_ERROR
      "MSVC ${MSVC_VERSION} is detected, but "
      "${PROJECT_NAME_UPPERCASE} requires MSVC ${msvc_required_version} or greater (Visual Studio 2026)."
    )
  endif()

  if(DART_TREAT_WARNINGS_AS_ERRORS)
    add_compile_options(/WX)
  endif()
  # /MP - Multi-processor compilation (uses all available cores)
  # /FS - Force synchronous PDB writes (prevents PDB conflicts in parallel builds with /MP)
  # /utf-8 - Required by fmt's Unicode support on MSVC.
  set(
    CMAKE_CXX_FLAGS
    "${CMAKE_CXX_FLAGS} /EHsc /permissive- /Zc:twoPhase- /utf-8 /MP /FS"
  )
  set(CMAKE_EXE_LINKER_FLAGS_RELEASE "/INCREMENTAL:NO")
  if(NOT DART_MSVC_DEFAULT_OPTIONS)
    set(
      CMAKE_CXX_FLAGS_DEBUG
      "${CMAKE_CXX_FLAGS_DEBUG} ${DART_RUNTIME_LIBRARY}d /Zi /Gy /W1 /EHsc"
    )
    set(
      CMAKE_CXX_FLAGS_RELEASE
      "${CMAKE_CXX_FLAGS_RELEASE} ${DART_RUNTIME_LIBRARY} /Zi /GL /Gy /W1 /EHsc"
    )
  endif()
  add_definitions(-D_CRT_SECURE_NO_WARNINGS -D_ENABLE_EXTENDED_ALIGNED_STORAGE)
  add_compile_options(/wd4005)
  add_compile_options(/wd4099)
  add_compile_options(/wd4146) # for FCL warnings: https://github.com/dartsim/dart/runs/4568423649?check_suite_focus=true#step:5:407
  add_compile_options(/wd4244)
  add_compile_options(/wd4250)
  add_compile_options(/wd4267)
  add_compile_options(/wd4305)
  add_compile_options(/wd4334)
  add_compile_options(/wd4838)
  add_compile_options(/wd4996)
  add_compile_options(/bigobj)
elseif(CMAKE_COMPILER_IS_GNUCXX)
  # There is a known bug in GCC 12.1 and above that leads to spurious
  # -Wmaybe-uninitialized warnings from gcc/x86_64-linux-gnu/12/include/avxintrin.h and
  # -Warray-bounds warnings from gcc/x86_64-linux-gnu/12/include/avx512fintrin.h.
  # The bug is tracked here: https://gcc.gnu.org/bugzilla/show_bug.cgi?id=105593
  # The following workaround can be removed once the bug is fixed.
  if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 12.1)
    add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-Wno-array-bounds>)
    add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-Wno-dangling-pointer>)
    add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-Wno-maybe-uninitialized>)
    add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-Wno-stringop-overflow>)
    add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-Wno-uninitialized>)
  endif()

  add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-Wall>)
  add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-Wextra>)
  add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-fPIC>)
  if(DART_TREAT_WARNINGS_AS_ERRORS)
    add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-Werror>)
    # Also treat preprocessor warnings (like #warning directives in deprecated
    # headers) as errors
    add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-Werror=cpp>)
  endif()
  execute_process(
    COMMAND ${CMAKE_CXX_COMPILER} -dumpfullversion -dumpversion
    OUTPUT_VARIABLE GCC_VERSION
  )
  if(DART_BUILD_WHEELS)
    set(gcc_required_version 10.2.1) # Lowered from 11.2.0 to support older glibc-based wheel builds
  else()
    set(gcc_required_version 11.2.0)
  endif()
  if(GCC_VERSION VERSION_LESS ${gcc_required_version})
    message(
      FATAL_ERROR
      "The installed g++ version is ${GCC_VERSION}. ${PROJECT_NAME} requires g++ ${gcc_required_version} or greater."
    )
  endif()
  if(GCC_VERSION VERSION_GREATER_EQUAL 13.2.0)
    # GCC 13 currently reports noisy diagnostics in upstream dependencies; silence
    # them until Eigen/FCL updates remove the false positives.
    add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-Wno-overloaded-virtual>)
    add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-Wno-alloc-size-larger-than>)
    add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-Wno-dangling-pointer>)
    add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-Wno-restrict>)
  endif()
  if(GCC_VERSION VERSION_GREATER_EQUAL 15.0)
    # TODO(GCC15): Remove once upstream reference dependencies stop using
    # <ciso646>, which is deprecated in C++20. GCC 15 emits a #warning that our
    # -Werror=cpp flag treats as an error.
    # Tracking: FCL and OctoMap reference targets can pull it transitively.
    add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-Wno-error=cpp>)
  endif()
  set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
  set(
    CMAKE_CXX_FLAGS_DEBUG
    "-g -fno-omit-frame-pointer -fno-inline-functions -fno-inline-functions-called-once -fno-optimize-sibling-calls"
  )
  if(DART_FAST_DEBUG)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O1")
  endif()
  set(
    CMAKE_CXX_FLAGS_RELWITHDEBINFO
    "${CMAKE_CXX_FLAGS_RELEASE} ${CMAKE_CXX_FLAGS_DEBUG}"
  )
  set(CMAKE_CXX_FLAGS_PROFILE "${CMAKE_CXX_FLAGS_DEBUG} -pg")
  set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--no-undefined")

  # Enforce to colorize compilation output
  if(DART_FORCE_COLORED_OUTPUT)
    add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-fdiagnostics-color=always>)
  endif()
elseif("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
  add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-Wall>)
  add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-Wextra>)
  add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-fPIC>)
  if(DART_TREAT_WARNINGS_AS_ERRORS)
    add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-Werror>)
    # Also treat preprocessor warnings (like #warning directives in deprecated
    # headers) as errors
    add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-Werror=cpp>)
  endif()
  execute_process(
    COMMAND ${CMAKE_CXX_COMPILER} -dumpversion
    OUTPUT_VARIABLE CLANG_VERSION
  )
  if(CLANG_VERSION VERSION_LESS 6.0)
    message(
      FATAL_ERROR
      "The installed Clang version is ${CLANG_VERSION}. ${PROJECT_NAME} requires clang 6.0 or greater."
    )
  endif()
  if("${CMAKE_SYSTEM_NAME}" MATCHES "Darwin")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
  endif()
  set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
  set(
    CMAKE_CXX_FLAGS_DEBUG
    "-g -fno-omit-frame-pointer -fno-inline-functions -fno-optimize-sibling-calls"
  )
  if(DART_FAST_DEBUG)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O1")
  endif()
  set(
    CMAKE_CXX_FLAGS_RELWITHDEBINFO
    "${CMAKE_CXX_FLAGS_RELEASE} ${CMAKE_CXX_FLAGS_DEBUG}"
  )
  set(CMAKE_CXX_FLAGS_PROFILE "${CMAKE_CXX_FLAGS_DEBUG} -pg")

  # Enforce to colorize compilation output
  if(DART_FORCE_COLORED_OUTPUT)
    add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-fcolor-diagnostics>)
  endif()
else()
  message(SEND_ERROR "Compiler[${CMAKE_CXX_COMPILER_ID}] not supported.")
endif()

if(DART_ENABLE_ASAN)
  if(
    NOT
      (
        CMAKE_CXX_COMPILER_ID MATCHES "Clang"
        OR CMAKE_CXX_COMPILER_ID STREQUAL "GNU"
      )
  )
    message(FATAL_ERROR "DART_ENABLE_ASAN is only supported with GCC or Clang.")
  endif()

  set(_DART_ASAN_FLAGS "-fsanitize=address")
  add_compile_options(${_DART_ASAN_FLAGS})
  add_link_options(${_DART_ASAN_FLAGS})

  foreach(
    link_var
    CMAKE_EXE_LINKER_FLAGS
    CMAKE_SHARED_LINKER_FLAGS
    CMAKE_MODULE_LINKER_FLAGS
  )
    set(${link_var} "${${link_var}} ${_DART_ASAN_FLAGS}")
  endforeach()

  foreach(lang C CXX)
    set(CMAKE_${lang}_FLAGS "${CMAKE_${lang}_FLAGS} -fno-omit-frame-pointer")
  endforeach()

  set(CMAKE_REQUIRED_FLAGS "${CMAKE_REQUIRED_FLAGS} ${_DART_ASAN_FLAGS}")
  set(
    CMAKE_REQUIRED_LINK_OPTIONS
    "${CMAKE_REQUIRED_LINK_OPTIONS} ${_DART_ASAN_FLAGS}"
  )

  # Disable Boost's embedded GDB scripts, which are known to trigger quoting issues
  # with recent GCC versions when AddressSanitizer instrumentation is enabled.
  add_compile_definitions(BOOST_ALL_NO_EMBEDDED_GDB_SCRIPTS)
endif()

if(DART_SIMD_FORCE_SCALAR)
  add_compile_definitions(DART_SIMD_FORCE_SCALAR=1)
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
  message(STATUS "Build gui        : ${DART_BUILD_GUI}")
  message(STATUS "Install path     : ${CMAKE_INSTALL_PREFIX}")
  message(STATUS "CXX_FLAGS        : ${CMAKE_CXX_FLAGS}")
  if(CMAKE_BUILD_TYPE_UPPERCASE STREQUAL "RELEASE")
    message(STATUS "CXX_FLAGS_RELEASE: ${CMAKE_CXX_FLAGS_RELEASE}")
  elseif(CMAKE_BUILD_TYPE_UPPERCASE STREQUAL "DEBUG")
    message(STATUS "CXX_FLAGS_DEBUG  : ${CMAKE_CXX_FLAGS_DEBUG}")
  elseif(CMAKE_BUILD_TYPE_UPPERCASE STREQUAL "RELWITHDEBINFO")
    message(
      STATUS
      "CXX_FLAGS_RELWITHDEBINFO: ${CMAKE_CXX_FLAGS_RELWITHDEBINFO}"
    )
  elseif(CMAKE_BUILD_TYPE_UPPERCASE STREQUAL "PROFILE")
    message(STATUS "CXX_FLAGS_PROFILE: ${CMAKE_CXX_FLAGS_PROFILE}")
  endif()
  message(STATUS "DART_SOURCE_DIR : ${DART_SOURCE_DIR}")
  message(STATUS "DART_BINARY_DIR : ${DART_BINARY_DIR}")
endif()

#===============================================================================
# Add sub-directories
#===============================================================================

add_subdirectory(dart)

if(TARGET dart AND TARGET dart-collision-native AND TARGET dart-io)
  add_subdirectory(dart/simulation)
elseif(DART_BUILD_DIFF OR DART_ENABLE_EXPERIMENTAL_CUDA)
  message(
    FATAL_ERROR
    "DART_BUILD_DIFF/DART_ENABLE_EXPERIMENTAL_CUDA require the "
    "simulation module, but it cannot be built: it needs the "
    "dart, dart-collision-native and dart-io components, which are not all "
    "being built in this configuration. Install the missing prerequisites "
    "(e.g. dart-io's tinyxml2) or disable the opt-in subfeature."
  )
else()
  message(
    WARNING
    "Skipping dart-simulation: it requires the dart, "
    "dart-collision-native and dart-io components, which are not all being "
    "built in this configuration. Install the missing prerequisites."
  )
endif()

set(DART_IN_SOURCE_BUILD TRUE)

if(TARGET dart)
  include(CTest)

  # dartsim application (headless editor engine + GUI). Added before tests so
  # tests/unit/dartsim_engine can link the dartsim_engine target. The engine is
  # headless; the GUI library and executable build only when dart-gui exists.
  if(TARGET dart-simulation)
    add_subdirectory(dartsim)
  endif()

  if(DART_BUILD_TESTS AND BUILD_TESTING)
    # Add a "tests" target to build unit tests.
    if(MSVC)
      add_subdirectory(tests)
    else()
      add_subdirectory(tests EXCLUDE_FROM_ALL)
    endif()

    # The simulation test meta-target is normally defined by the simulation
    # test subdirectory. It can be absent when the module was gracefully skipped
    # because a core sibling component (dart-io/dart-collision-native) is not
    # built. Define an empty stub so build/test entry points resolve the target
    # and skip gracefully instead of failing on an unknown target.
    if(NOT TARGET dart_simulation_tests)
      add_custom_target(dart_simulation_tests)
    endif()
  elseif(NOT DART_BUILD_TESTS)
    message(STATUS "Skipping tests (DART_BUILD_TESTS == OFF)")
  else()
    message(STATUS "Skipping tests (BUILD_TESTING == OFF)")
  endif()

  if(DART_BUILD_EXAMPLES)
    # Add example subdirectories and an "examples" target. Individual example
    # CMake files skip themselves when their optional component targets are
    # unavailable, so non-GUI examples still build when dart-gui is disabled.
    if(MSVC)
      add_subdirectory(examples)
    else()
      add_subdirectory(examples EXCLUDE_FROM_ALL)
      get_property(examples GLOBAL PROPERTY DART_EXAMPLES)
      add_custom_target(examples DEPENDS ${examples})
    endif()

    if(DART_VERBOSE)
      message(STATUS "")
      message(STATUS "[ Examples ]")
      foreach(example IN LISTS examples)
        message(STATUS "Adding example: ${example}")
      endforeach()
    else()
      list(LENGTH examples examples_length)
      message(STATUS "Adding ${examples_length} examples")
    endif()
  else()
    message(STATUS "Skipping examples (DART_BUILD_EXAMPLES == OFF)")
  endif()

  if(DART_BUILD_TUTORIALS)
    message(STATUS "Skipping removed legacy GUI tutorials")
  else()
    message(STATUS "Skipping tutorials (DART_BUILD_TUTORIALS == OFF)")
  endif()

  # The dartsim application is added above (before tests) via
  # add_subdirectory(dartsim); its GUI library and executable are gated on
  # dart-gui inside dartsim/CMakeLists.txt.
endif()

add_subdirectory(python)

# Add 'ALL' target that builds everything
set(all_target_candidates)
if(DART_BUILD_DARTPY)
  list(APPEND all_target_candidates dartpy)
endif()
if(DART_BUILD_TESTS AND BUILD_TESTING)
  list(APPEND all_target_candidates tests_and_run pytest)
endif()
foreach(target_candidate IN LISTS all_target_candidates)
  if(TARGET ${target_candidate})
    list(APPEND all_targets ${target_candidate})
  endif()
endforeach()
foreach(target_candidate IN LISTS examples)
  if(TARGET ${target_candidate})
    list(APPEND all_targets ${target_candidate})
  endif()
endforeach()
foreach(target_candidate IN LISTS tutorials)
  if(TARGET ${target_candidate})
    list(APPEND all_targets ${target_candidate})
  endif()
endforeach()
if(TARGET dartsim)
  list(APPEND all_targets dartsim)
endif()
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
  DART_VERSION_IN
  ${DART_SOURCE_DIR}/cmake/${PROJECT_NAME_UPPERCASE}ConfigVersion.cmake.in
)
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
set(DART_CMAKE_MIN_COMPATIBLE_VERSION "6.10.0")
configure_file(${DART_VERSION_IN} ${DART_VERSION_OUT} @ONLY)
install(
  FILES ${DART_CONFIG_OUT} ${DART_VERSION_OUT}
  DESTINATION "${CONFIG_INSTALL_DIR}"
)

# Generate the DART pkg-config
set(PC_CONFIG_IN ${DART_SOURCE_DIR}/cmake/dart.pc.in)
set(PC_CONFIG_OUT ${DART_BINARY_DIR}/cmake/dart.pc)
set(DART_PKG_LINK_LIBS "")
get_property(_dart_pkg_link_libs GLOBAL PROPERTY DART_PKG_LINK_LIBS)
if(_dart_pkg_link_libs)
  string(JOIN " " DART_PKG_LINK_LIBS ${_dart_pkg_link_libs})
  if(DART_PKG_LINK_LIBS)
    set(DART_PKG_LINK_LIBS " ${DART_PKG_LINK_LIBS}")
  endif()
endif()
set(DART_PKG_EXTRA_CFLAGS "")
get_property(_dart_pkg_includes GLOBAL PROPERTY DART_PUBLIC_INCLUDE_DIRS)
get_property(
  _dart_pkg_system_includes
  GLOBAL
  PROPERTY DART_PUBLIC_SYSTEM_INCLUDE_DIRS
)
set(_dart_pkg_all_includes ${_dart_pkg_includes} ${_dart_pkg_system_includes})
if(_dart_pkg_all_includes)
  list(REMOVE_DUPLICATES _dart_pkg_all_includes)
  foreach(_inc IN LISTS _dart_pkg_all_includes)
    if(_inc)
      string(APPEND DART_PKG_EXTRA_CFLAGS " -I${_inc}")
    endif()
  endforeach()
endif()
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
install(FILES ${PC_CONFIG_OUT} DESTINATION ${PC_CONFIG_INSTALL_DIR})

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
  DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/tutorials"
  DESTINATION ${DART_ADDITIONAL_DOCUMENTATION_INSTALL_PATH}
)

#===============================================================================
# Code Formatting
#===============================================================================
if(DART_VERBOSE)
  message(STATUS "")
  message(STATUS "[ Code Formatting ]")
endif()

# find_program caches its result and will not re-search once
# CLANG_FORMAT_EXECUTABLE is set, even if the cached path no longer exists. When
# the pinned clang-format is upgraded under an existing build tree (e.g. the
# pixi env moves clang-format-21 -> clang-format-22), the stale cache leaves a
# dangling DEPENDS and breaks the `format`/`check-format` targets with
# "needed by 'CMakeFiles/format', missing". Drop a vanished cache entry so the
# search re-runs against the currently installed toolchain.
if(CLANG_FORMAT_EXECUTABLE AND NOT EXISTS "${CLANG_FORMAT_EXECUTABLE}")
  unset(CLANG_FORMAT_EXECUTABLE CACHE)
endif()
find_program(CLANG_FORMAT_EXECUTABLE NAMES clang-format-22 clang-format)

get_property(formatting_files GLOBAL PROPERTY DART_FORMAT_FILES)
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
  set(DOXYGEN_EXCLUDE "${PROJECT_SOURCE_DIR}/dart/external")
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
  # Use dart-docs as the internal target name so an existing docs target can
  # depend on it without target-name conflicts.
  add_custom_target(dart-docs DEPENDS ${DOXYGEN_HTML_INDEX})
  if(NOT TARGET docs)
    add_custom_target(docs DEPENDS dart-docs)
  else()
    add_dependencies(docs dart-docs)
  endif()
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
set(DART_CMAKE_BUILD_CMD_BASE "cmake --build .")
set(DART_CMAKE_BUILD_CMD_SUFFIX "")

if(CMAKE_CONFIGURATION_TYPES)
  if(CMAKE_BUILD_TYPE)
    set(DART_CMAKE_BUILD_CMD_SUFFIX " --config ${CMAKE_BUILD_TYPE}")
  else()
    list(JOIN CMAKE_CONFIGURATION_TYPES ", " DART_CMAKE_KNOWN_CONFIGS)
    set(DART_CMAKE_BUILD_CMD_SUFFIX " --config <config>")
    message(
      STATUS
      "Available build configurations: ${DART_CMAKE_KNOWN_CONFIGS}"
    )
  endif()
endif()

set(
  DART_CMAKE_BUILD_CMD
  "${DART_CMAKE_BUILD_CMD_BASE}${DART_CMAKE_BUILD_CMD_SUFFIX}"
)

message(STATUS "Run '${DART_CMAKE_BUILD_CMD}' to build all the components")
if(BUILD_TESTING)
  message(
    STATUS
    "Run '${DART_CMAKE_BUILD_CMD} --target tests' to build all the tests"
  )
endif()
message(
  STATUS
  "Run '${DART_CMAKE_BUILD_CMD} --target examples' to build all the examples"
)
message(
  STATUS
  "Run '${DART_CMAKE_BUILD_CMD} --target tutorials' to build all the tutorials"
)
message(
  STATUS
  "Run '${DART_CMAKE_BUILD_CMD} --target view_docs' to see the API documentation"
)
message(
  STATUS
  "Run '${DART_CMAKE_BUILD_CMD} --target install' to install all the C++ components"
)
if(TARGET dartpy)
  message(
    STATUS
    "Run '${DART_CMAKE_BUILD_CMD} --target dartpy' to build the python bindings"
  )
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
