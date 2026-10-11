# Find Filament for the DART GUI and create:
#   Filament::filament
#   Filament::matc
#
# The preferred provider is the conda-forge `filament` package, whose CMake
# package config (FilamentConfig.cmake) already defines both targets; the
# CONFIG-first find below picks it up from the environment prefix.
#
# Upstream Filament release archives do not ship CMake package files, so this
# finder also accepts Filament_ROOT or FILAMENT_ROOT pointing at the archive
# layout:
#   include/
#   lib/<arch>/libfilament.a
#   bin/matc
# Linux SDKs must use libstdc++; native macOS archives use libc++.

include(FindPackageHandleStandardArgs)
get_property(
  _dart_filament_previous_imports
  DIRECTORY
  PROPERTY IMPORTED_TARGETS
)

# Fixtures in sibling directories must use the same SDK selected for the GUI.
# Promote only imports created by this finder, including SDK dependencies.
macro(_dart_filament_promote_imports)
  get_property(
    _dart_filament_current_imports
    DIRECTORY
    PROPERTY IMPORTED_TARGETS
  )
  foreach(_dart_filament_import IN LISTS _dart_filament_current_imports)
    if(NOT _dart_filament_import IN_LIST _dart_filament_previous_imports)
      set_property(
        TARGET ${_dart_filament_import}
        PROPERTY IMPORTED_GLOBAL TRUE
      )
    endif()
  endforeach()
endmacro()

set(_dart_filament_roots)
foreach(_root_var Filament_ROOT FILAMENT_ROOT)
  if(DEFINED ${_root_var} AND NOT "${${_root_var}}" STREQUAL "")
    list(APPEND _dart_filament_roots "${${_root_var}}")
  endif()
endforeach()
if(
  (NOT DEFINED DART_USE_SYSTEM_FILAMENT OR DART_USE_SYSTEM_FILAMENT)
  AND DEFINED ENV{Filament_ROOT}
  AND NOT "$ENV{Filament_ROOT}" STREQUAL ""
)
  list(APPEND _dart_filament_roots "$ENV{Filament_ROOT}")
endif()
if(
  (NOT DEFINED DART_USE_SYSTEM_FILAMENT OR DART_USE_SYSTEM_FILAMENT)
  AND DEFINED ENV{FILAMENT_ROOT}
  AND NOT "$ENV{FILAMENT_ROOT}" STREQUAL ""
)
  list(APPEND _dart_filament_roots "$ENV{FILAMENT_ROOT}")
endif()

if(
  DEFINED DART_USE_SYSTEM_FILAMENT
  AND NOT DART_USE_SYSTEM_FILAMENT
  AND Filament_ROOT
)
  # The prepared provider root must not be replaced by an unrelated SDK hint.
  set(_dart_filament_roots "${Filament_ROOT}")
endif()

# Cached archive paths must not mix SDKs when the provider/root changes.
if(NOT "${DART_FILAMENT_SEARCH_ROOTS}" STREQUAL "${_dart_filament_roots}")
  unset(Filament_DIR CACHE)
  unset(filament_DIR CACHE)
  get_cmake_property(_dart_filament_cached_variables CACHE_VARIABLES)
  foreach(
    _dart_filament_cached_variable
    IN
    LISTS _dart_filament_cached_variables
  )
    if(
      _dart_filament_cached_variable
        MATCHES
        "^Filament_.*(LIBRARY|INCLUDE_DIR|EXECUTABLE)$"
    )
      unset(${_dart_filament_cached_variable} CACHE)
    endif()
  endforeach()
  set(
    DART_FILAMENT_SEARCH_ROOTS
    "${_dart_filament_roots}"
    CACHE INTERNAL
    "Filament SDK search roots"
    FORCE
  )
endif()

macro(_dart_filament_unset_missing_cache_path variable)
  if(
    DEFINED ${variable}
    AND NOT "${${variable}}" STREQUAL ""
    AND NOT "${${variable}}" MATCHES "-NOTFOUND$"
    AND NOT EXISTS "${${variable}}"
  )
    unset(${variable} CACHE)
    unset(${variable})
  endif()
endmacro()

foreach(
  _dart_filament_cache_var
  IN
  ITEMS
    Filament_INCLUDE_DIR
    Filament_MATC_EXECUTABLE
    Filament_filament_LIBRARY
    Filament_backend_LIBRARY
    Filament_filabridge_LIBRARY
    Filament_filaflat_LIBRARY
    Filament_utils_LIBRARY
    Filament_geometry_LIBRARY
    Filament_bluegl_LIBRARY
    Filament_bluevk_LIBRARY
    Filament_smol_v_LIBRARY
    Filament_shaders_LIBRARY
    Filament_webgpu_dawn_LIBRARY
    Filament_abseil_LIBRARY
    Filament_zstd_LIBRARY
    Filament_Foundation_LIBRARY
    Filament_objc_LIBRARY
    Filament_cxx_LIBRARY
    Filament_cxxabi_LIBRARY
)
  _dart_filament_unset_missing_cache_path(${_dart_filament_cache_var})
endforeach()

# An explicit root must win. Filament_ROOT only *hints* the config search, so
# a root without CMake package files would silently lose to a packaged config
# elsewhere on the prefix path (e.g. a conda environment): when roots are
# given, probe them for package configs exclusively and otherwise fall through
# to the archive-layout search below. The default config search also stays off
# when DART explicitly selects the fetched archive
# (DART_USE_SYSTEM_FILAMENT=OFF).
if(_dart_filament_roots)
  find_package(
    filament
    CONFIG
    QUIET
    PATHS ${_dart_filament_roots}
    NO_DEFAULT_PATH
  )
  find_package(
    Filament
    CONFIG
    QUIET
    PATHS ${_dart_filament_roots}
    NO_DEFAULT_PATH
  )
elseif(NOT DEFINED DART_USE_SYSTEM_FILAMENT OR DART_USE_SYSTEM_FILAMENT)
  find_package(filament CONFIG QUIET)
  find_package(Filament CONFIG QUIET)
endif()
_dart_filament_promote_imports()

if(TARGET filament::filament AND NOT TARGET Filament::filament)
  add_library(Filament::filament ALIAS filament::filament)
endif()
if(TARGET filament AND NOT TARGET Filament::filament)
  add_library(Filament::filament ALIAS filament)
endif()
if(TARGET filament::matc AND NOT TARGET Filament::matc)
  add_executable(Filament::matc ALIAS filament::matc)
endif()
if(TARGET matc AND NOT TARGET Filament::matc)
  add_executable(Filament::matc ALIAS matc)
endif()

# When explicit roots are given, every Filament-owned component (headers, the
# Filament libraries, and matc) is searched in the roots exclusively: default
# locations (CMAKE_PREFIX_PATH/CMAKE_LIBRARY_PATH/PATH) rank above HINTS, so a
# conda environment on the search path could otherwise shadow the requested
# tree — or, for a component missing from an incomplete root, silently mix
# installations. Components missing from the roots therefore stay NOTFOUND and
# fail the package check below instead of falling through. Support libraries
# that Filament does not ship in every tree (zstd, libc++/libc++abi) keep a
# root-preferred search with the regular fallback.
if(_dart_filament_roots)
  find_path(
    Filament_INCLUDE_DIR
    NAMES filament/Engine.h
    PATHS ${_dart_filament_roots}
    PATH_SUFFIXES include
    NO_DEFAULT_PATH
  )
else()
  find_path(Filament_INCLUDE_DIR NAMES filament/Engine.h PATH_SUFFIXES include)
endif()

set(Filament_LIBRARIES)

set(
  _dart_filament_library_suffixes
  lib
  lib/x86_64
  lib/arm64
  lib/aarch64
  lib/${CMAKE_SYSTEM_PROCESSOR}
)
if(MSVC)
  list(
    PREPEND _dart_filament_library_suffixes
    lib/x86_64/md
    lib/${CMAKE_SYSTEM_PROCESSOR}/md
    lib/x86_64/mt
    lib/${CMAKE_SYSTEM_PROCESSOR}/mt
  )
endif()

foreach(
  _lib
  IN
  ITEMS
    filament
    backend
    filabridge
    filaflat
    utils
    geometry
    bluegl
    bluevk
    smol-v
    shaders
    webgpu_dawn
    abseil
)
  string(REPLACE "-" "_" _lib_var "${_lib}")
  if(_dart_filament_roots)
    find_library(
      Filament_${_lib_var}_LIBRARY
      NAMES ${_lib}
      PATHS ${_dart_filament_roots}
      PATH_SUFFIXES ${_dart_filament_library_suffixes}
      NO_DEFAULT_PATH
    )
  else()
    find_library(
      Filament_${_lib_var}_LIBRARY
      NAMES ${_lib}
      PATH_SUFFIXES ${_dart_filament_library_suffixes}
    )
  endif()
  if(Filament_${_lib_var}_LIBRARY)
    list(APPEND Filament_LIBRARIES "${Filament_${_lib_var}_LIBRARY}")
  endif()
endforeach()

# zstd is a support library: prefer the copy bundled with the roots, but allow
# the regular search to provide it for trees that do not ship one.
if(_dart_filament_roots)
  find_library(
    Filament_zstd_LIBRARY
    NAMES zstd
    PATHS ${_dart_filament_roots}
    PATH_SUFFIXES ${_dart_filament_library_suffixes}
    NO_DEFAULT_PATH
  )
endif()
find_library(
  Filament_zstd_LIBRARY
  NAMES zstd
  HINTS ${_dart_filament_roots}
  PATH_SUFFIXES ${_dart_filament_library_suffixes}
)
if(Filament_zstd_LIBRARY)
  list(APPEND Filament_LIBRARIES "${Filament_zstd_LIBRARY}")
endif()

if(APPLE)
  # Filament's macOS backends pull in several system frameworks: Metal +
  # QuartzCore (CAMetalLayer) for the Metal backend, CoreVideo for the
  # texture-cache interop, Cocoa for the NSView/NSWindow surface, plus
  # IOKit/CoreGraphics/OpenGL. The conda-forge `Filament::filament` CONFIG
  # target does not always carry these in its link interface, so a missing
  # framework surfaces as undefined Metal/CoreVideo/QuartzCore/AppKit symbols
  # when linking GUI consumers such as `_dartpy.so`. Collect them so they reach
  # every Filament consumer.
  set(_dart_filament_apple_frameworks)
  foreach(
    _dart_fw
    Foundation
    Cocoa
    Metal
    CoreVideo
    QuartzCore
    IOKit
    IOSurface
    CoreGraphics
    OpenGL
  )
    find_library(Filament_${_dart_fw}_FRAMEWORK NAMES ${_dart_fw})
    mark_as_advanced(Filament_${_dart_fw}_FRAMEWORK)
    if(Filament_${_dart_fw}_FRAMEWORK)
      list(
        APPEND _dart_filament_apple_frameworks
        "${Filament_${_dart_fw}_FRAMEWORK}"
      )
    endif()
  endforeach()

  find_library(Filament_objc_LIBRARY NAMES objc)
  if(Filament_objc_LIBRARY)
    list(APPEND _dart_filament_apple_frameworks "${Filament_objc_LIBRARY}")
  endif()

  list(APPEND Filament_LIBRARIES ${_dart_filament_apple_frameworks})
elseif(WIN32)
  list(APPEND Filament_LIBRARIES opengl32)
endif()

# Recent static SDKs include the Linux WebGPU platform's direct Xlib calls.
if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND Filament_webgpu_dawn_LIBRARY)
  find_package(X11 REQUIRED)
  list(APPEND Filament_LIBRARIES X11::X11)
endif()

if(WIN32 AND Filament_webgpu_dawn_LIBRARY)
  list(APPEND Filament_LIBRARIES user32 onecore_apiset dxguid)
endif()

set(_dart_filament_requires_libcxx FALSE)
if(CMAKE_NM AND Filament_utils_LIBRARY)
  execute_process(
    COMMAND "${CMAKE_NM}" -u -C "${Filament_utils_LIBRARY}"
    OUTPUT_VARIABLE _dart_filament_nm_output
    ERROR_QUIET
  )
  if(_dart_filament_nm_output MATCHES "std::__1")
    set(_dart_filament_requires_libcxx TRUE)
  endif()
endif()
set(Filament_REQUIRES_LIBCXX "${_dart_filament_requires_libcxx}")

if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND _dart_filament_requires_libcxx)
  message(
    FATAL_ERROR
    "This Filament SDK uses libc++ and cannot safely translate exceptions through DART's libstdc++ runtime. Use a libstdc++ SDK such as conda-forge filament, or DART_USE_SYSTEM_FILAMENT=OFF to build the pinned source SDK."
  )
endif()

if(_dart_filament_requires_libcxx)
  find_library(
    Filament_cxx_LIBRARY
    NAMES c++
    HINTS ${_dart_filament_roots}
    PATH_SUFFIXES ${_dart_filament_library_suffixes}
  )
  if(Filament_cxx_LIBRARY)
    list(APPEND Filament_LIBRARIES "${Filament_cxx_LIBRARY}")
  endif()

  find_library(
    Filament_cxxabi_LIBRARY
    NAMES c++abi
    HINTS ${_dart_filament_roots}
    PATH_SUFFIXES ${_dart_filament_library_suffixes}
  )
  if(Filament_cxxabi_LIBRARY)
    list(APPEND Filament_LIBRARIES "${Filament_cxxabi_LIBRARY}")
  endif()
endif()

if(_dart_filament_roots)
  find_program(
    Filament_MATC_EXECUTABLE
    NAMES matc matc.exe
    PATHS ${_dart_filament_roots}
    PATH_SUFFIXES bin
    NO_DEFAULT_PATH
  )
else()
  find_program(Filament_MATC_EXECUTABLE NAMES matc matc.exe PATH_SUFFIXES bin)
endif()

set(_dart_filament_required_vars)
if(NOT TARGET Filament::filament)
  list(
    APPEND _dart_filament_required_vars
    Filament_INCLUDE_DIR
    Filament_filament_LIBRARY
    Filament_backend_LIBRARY
    Filament_filabridge_LIBRARY
    Filament_filaflat_LIBRARY
    Filament_utils_LIBRARY
    Filament_geometry_LIBRARY
  )
endif()
if(DART_FILAMENT_REQUIRE_MATC AND NOT TARGET Filament::matc)
  list(APPEND _dart_filament_required_vars Filament_MATC_EXECUTABLE)
endif()
if(_dart_filament_requires_libcxx AND NOT TARGET Filament::filament)
  list(
    APPEND _dart_filament_required_vars
    Filament_cxx_LIBRARY
    Filament_cxxabi_LIBRARY
  )
endif()

if(
  TARGET Filament::filament
  AND (NOT DART_FILAMENT_REQUIRE_MATC OR TARGET Filament::matc)
)
  set(Filament_FOUND TRUE)
else()
  find_package_handle_standard_args(
    Filament
    REQUIRED_VARS ${_dart_filament_required_vars}
    REASON_FAILURE_MESSAGE
      "Install a Filament development package such as conda-forge `filament`, set Filament_ROOT to a Filament install tree, or provide an upstream archive plus any required libc++/libc++abi libraries."
  )
endif()

if(Filament_FOUND AND NOT TARGET Filament::filament)
  find_package(Threads REQUIRED)
  add_library(Filament::filament INTERFACE IMPORTED)
  set_target_properties(
    Filament::filament
    PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${Filament_INCLUDE_DIR}"
      INTERFACE_LINK_LIBRARIES
        "${Filament_LIBRARIES};Threads::Threads;${CMAKE_DL_LIBS}"
  )
endif()

# When `Filament::filament` comes from a CONFIG package (e.g. conda-forge), its
# link interface may omit the macOS system frameworks discovered above. Append
# them so every consumer of Filament — including the dartpy extension built for
# the wheel — resolves Metal/CoreVideo/QuartzCore/AppKit and friends. The
# manual-find path already folds them into the interface via Filament_LIBRARIES;
# this also covers the CONFIG path, resolving the `Filament::filament` alias to
# its real target since properties cannot be set on an ALIAS.
if(APPLE AND TARGET Filament::filament AND _dart_filament_apple_frameworks)
  get_target_property(_dart_filament_real Filament::filament ALIASED_TARGET)
  if(NOT _dart_filament_real)
    set(_dart_filament_real Filament::filament)
  endif()
  set_property(
    TARGET ${_dart_filament_real}
    APPEND
    PROPERTY INTERFACE_LINK_LIBRARIES "${_dart_filament_apple_frameworks}"
  )
endif()

if(Filament_FOUND AND Filament_MATC_EXECUTABLE AND NOT TARGET Filament::matc)
  add_executable(Filament::matc IMPORTED)
  set_target_properties(
    Filament::matc
    PROPERTIES IMPORTED_LOCATION "${Filament_MATC_EXECUTABLE}"
  )
endif()

mark_as_advanced(
  Filament_INCLUDE_DIR
  Filament_MATC_EXECUTABLE
  Filament_filament_LIBRARY
  Filament_backend_LIBRARY
  Filament_filabridge_LIBRARY
  Filament_filaflat_LIBRARY
  Filament_utils_LIBRARY
  Filament_geometry_LIBRARY
  Filament_bluegl_LIBRARY
  Filament_bluevk_LIBRARY
  Filament_smol_v_LIBRARY
  Filament_shaders_LIBRARY
  Filament_webgpu_dawn_LIBRARY
  Filament_abseil_LIBRARY
  Filament_zstd_LIBRARY
  Filament_objc_LIBRARY
  Filament_cxx_LIBRARY
  Filament_cxxabi_LIBRARY
)

# The installed DART GUI is C++17; the SDK's C++20 requirement belongs only
# to DART's private implementation, including for static consumers.
if(Filament_FOUND AND NOT TARGET DARTFilament::Runtime)
  add_library(DARTFilament::Runtime INTERFACE IMPORTED)
  set_target_properties(
    DARTFilament::Runtime
    PROPERTIES INTERFACE_LINK_LIBRARIES "$<LINK_ONLY:Filament::filament>"
  )
  if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    # The release archive's BlueGL assembly omits the GNU-stack annotation.
    set_property(
      TARGET DARTFilament::Runtime
      PROPERTY INTERFACE_LINK_OPTIONS "-Wl,-z,noexecstack"
    )
  endif()
endif()

_dart_filament_promote_imports()
