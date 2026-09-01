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
# including the libc++/libc++abi link handling those upstream binaries need.

include(FindPackageHandleStandardArgs)

set(_dart_filament_roots)
foreach(_root_var Filament_ROOT FILAMENT_ROOT)
  if(DEFINED ${_root_var} AND NOT "${${_root_var}}" STREQUAL "")
    list(APPEND _dart_filament_roots "${${_root_var}}")
  endif()
endforeach()
if(DEFINED ENV{Filament_ROOT} AND NOT "$ENV{Filament_ROOT}" STREQUAL "")
  list(APPEND _dart_filament_roots "$ENV{Filament_ROOT}")
endif()
if(DEFINED ENV{FILAMENT_ROOT} AND NOT "$ENV{FILAMENT_ROOT}" STREQUAL "")
  list(APPEND _dart_filament_roots "$ENV{FILAMENT_ROOT}")
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

# When explicit roots are given they take precedence over every default search
# location (CMAKE_PREFIX_PATH/CMAKE_LIBRARY_PATH rank above HINTS, so e.g. a
# conda environment on the search path could otherwise shadow Filament_ROOT):
# probe the roots exclusively first, then fall back to the regular search for
# rootless setups. The second call is a no-op once the cache variable is set.
if(_dart_filament_roots)
  find_path(
    Filament_INCLUDE_DIR
    NAMES filament/Engine.h
    PATHS ${_dart_filament_roots}
    PATH_SUFFIXES include
    NO_DEFAULT_PATH
  )
endif()
find_path(
  Filament_INCLUDE_DIR
  NAMES filament/Engine.h
  HINTS ${_dart_filament_roots}
  PATH_SUFFIXES include
)

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
  endif()
  find_library(
    Filament_${_lib_var}_LIBRARY
    NAMES ${_lib}
    HINTS ${_dart_filament_roots}
    PATH_SUFFIXES ${_dart_filament_library_suffixes}
  )
  if(Filament_${_lib_var}_LIBRARY)
    list(APPEND Filament_LIBRARIES "${Filament_${_lib_var}_LIBRARY}")
  endif()
endforeach()

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
  # target does not always carry these in its link interface, and the dartpy
  # wheel is the only macOS build that links Filament with the GUI enabled
  # (the arm64 test jobs build GUI-off), so a missing framework surfaces there
  # as undefined Metal/CoreVideo/QuartzCore/AppKit symbols when linking
  # `_dartpy.so`. Collect them so they reach every Filament consumer.
  set(_dart_filament_apple_frameworks)
  foreach(
    _dart_fw
    Foundation
    Cocoa
    Metal
    CoreVideo
    QuartzCore
    IOKit
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
endif()
find_program(
  Filament_MATC_EXECUTABLE
  NAMES matc matc.exe
  HINTS ${_dart_filament_roots}
  PATH_SUFFIXES bin
)

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
if(NOT TARGET Filament::matc)
  list(APPEND _dart_filament_required_vars Filament_MATC_EXECUTABLE)
endif()
if(_dart_filament_requires_libcxx AND NOT TARGET Filament::filament)
  list(
    APPEND _dart_filament_required_vars
    Filament_cxx_LIBRARY
    Filament_cxxabi_LIBRARY
  )
endif()

if(TARGET Filament::filament AND TARGET Filament::matc)
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

if(Filament_FOUND AND NOT TARGET Filament::matc)
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
  Filament_zstd_LIBRARY
  Filament_objc_LIBRARY
  Filament_cxx_LIBRARY
  Filament_cxxabi_LIBRARY
)
