# Find a Filament install tree for the experimental Filament GUI example.
#
# This module accepts Filament_ROOT or FILAMENT_ROOT and creates:
#   Filament::filament
#   Filament::matc
#
# Upstream Filament release archives do not currently ship CMake package files,
# so this finder also supports the archive layout:
#   include/
#   lib/<arch>/libfilament.a
#   bin/matc
#
# The planned conda-forge recipe splits build artifacts into a `filament-static`
# development output and a `filament` tool output. Installing `filament-static`
# should make include/, lib/, and bin/matc visible from the same prefix.

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
  if(DEFINED ${variable}
      AND NOT "${${variable}}" STREQUAL ""
      AND NOT "${${variable}}" MATCHES "-NOTFOUND$"
      AND NOT EXISTS "${${variable}}")
    unset(${variable} CACHE)
    unset(${variable})
  endif()
endmacro()

foreach(_dart_filament_cache_var IN ITEMS
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
    Filament_cxx_LIBRARY
    Filament_cxxabi_LIBRARY)
  _dart_filament_unset_missing_cache_path(${_dart_filament_cache_var})
endforeach()

find_package(filament CONFIG QUIET)
find_package(Filament CONFIG QUIET)

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

find_path(Filament_INCLUDE_DIR
  NAMES filament/Engine.h
  HINTS ${_dart_filament_roots}
  PATH_SUFFIXES include
)

set(Filament_LIBRARIES)

set(_dart_filament_library_suffixes
  lib
  lib/x86_64
  lib/arm64
  lib/aarch64
  lib/${CMAKE_SYSTEM_PROCESSOR}
)

foreach(_lib IN ITEMS filament backend filabridge filaflat utils geometry bluegl bluevk smol-v shaders)
  string(REPLACE "-" "_" _lib_var "${_lib}")
  find_library(Filament_${_lib_var}_LIBRARY
    NAMES ${_lib}
    HINTS ${_dart_filament_roots}
    PATH_SUFFIXES ${_dart_filament_library_suffixes}
  )
  if(Filament_${_lib_var}_LIBRARY)
    list(APPEND Filament_LIBRARIES "${Filament_${_lib_var}_LIBRARY}")
  endif()
endforeach()

find_library(Filament_zstd_LIBRARY
  NAMES zstd
  HINTS ${_dart_filament_roots}
  PATH_SUFFIXES ${_dart_filament_library_suffixes}
)
if(Filament_zstd_LIBRARY)
  list(APPEND Filament_LIBRARIES "${Filament_zstd_LIBRARY}")
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
  find_library(Filament_cxx_LIBRARY
    NAMES c++
    HINTS ${_dart_filament_roots}
    PATH_SUFFIXES ${_dart_filament_library_suffixes}
  )
  if(Filament_cxx_LIBRARY)
    list(APPEND Filament_LIBRARIES "${Filament_cxx_LIBRARY}")
  endif()

  find_library(Filament_cxxabi_LIBRARY
    NAMES c++abi
    HINTS ${_dart_filament_roots}
    PATH_SUFFIXES ${_dart_filament_library_suffixes}
  )
  if(Filament_cxxabi_LIBRARY)
    list(APPEND Filament_LIBRARIES "${Filament_cxxabi_LIBRARY}")
  endif()
endif()

find_program(Filament_MATC_EXECUTABLE
  NAMES matc matc.exe
  HINTS ${_dart_filament_roots}
  PATH_SUFFIXES bin
)

set(_dart_filament_required_vars)
if(NOT TARGET Filament::filament)
  list(APPEND _dart_filament_required_vars
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
  list(APPEND
    _dart_filament_required_vars
    Filament_cxx_LIBRARY
    Filament_cxxabi_LIBRARY
  )
endif()

if(TARGET Filament::filament AND TARGET Filament::matc)
  set(Filament_FOUND TRUE)
else()
  find_package_handle_standard_args(Filament
    REQUIRED_VARS ${_dart_filament_required_vars}
    REASON_FAILURE_MESSAGE
      "Install a Filament development package such as conda-forge filament-static, set Filament_ROOT to a Filament install tree, or provide an upstream archive plus any required libc++/libc++abi libraries."
  )
endif()

if(Filament_FOUND AND NOT TARGET Filament::filament)
  find_package(Threads REQUIRED)
  add_library(Filament::filament INTERFACE IMPORTED)
  set_target_properties(Filament::filament PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${Filament_INCLUDE_DIR}"
    INTERFACE_LINK_LIBRARIES "${Filament_LIBRARIES};Threads::Threads;${CMAKE_DL_LIBS}"
  )
endif()

if(Filament_FOUND AND NOT TARGET Filament::matc)
  add_executable(Filament::matc IMPORTED)
  set_target_properties(Filament::matc PROPERTIES
    IMPORTED_LOCATION "${Filament_MATC_EXECUTABLE}"
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
  Filament_cxx_LIBRARY
  Filament_cxxabi_LIBRARY
)
