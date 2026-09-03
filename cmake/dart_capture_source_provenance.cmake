# Generate one build-time source identity shared by dartpy capture and the
# Figure 13 benchmark executable. The custom target intentionally runs on every
# requested build; the generator only updates the header when source bytes or
# the build commit identity change.

option(
  DART_FIGURE13_EVIDENCE_BUILD
  "Configure the canonical Figure 13 evidence benchmark build"
  OFF
)

set(
  _DART_CAPTURE_BUILD_CONFIGURATION_VARIABLES
  CMAKE_GENERATOR
  CMAKE_GENERATOR_PLATFORM
  CMAKE_GENERATOR_TOOLSET
  CMAKE_CXX_COMPILER
  CMAKE_CXX_COMPILER_ID
  CMAKE_CXX_COMPILER_VERSION
  CMAKE_CXX_COMPILER_TARGET
  CMAKE_CXX_COMPILER_EXTERNAL_TOOLCHAIN
  CMAKE_CXX_COMPILER_FRONTEND_VARIANT
  CMAKE_CXX_COMPILER_LAUNCHER
  CMAKE_SYSTEM_NAME
  CMAKE_SYSTEM_VERSION
  CMAKE_SYSTEM_PROCESSOR
  CMAKE_BUILD_TYPE
  CMAKE_CONFIGURATION_TYPES
  CMAKE_TOOLCHAIN_FILE
  CMAKE_SYSROOT
  CMAKE_CXX_FLAGS
  CMAKE_CXX_FLAGS_RELEASE
  CMAKE_EXE_LINKER_FLAGS
  CMAKE_EXE_LINKER_FLAGS_RELEASE
  CMAKE_SHARED_LINKER_FLAGS
  CMAKE_SHARED_LINKER_FLAGS_RELEASE
  CMAKE_MODULE_LINKER_FLAGS
  CMAKE_MODULE_LINKER_FLAGS_RELEASE
  CMAKE_LINKER_TYPE
  CMAKE_INTERPROCEDURAL_OPTIMIZATION
  BUILD_SHARED_LIBS
  BUILD_TESTING
  DARTPY_DEBUG_SYMBOLS
  DART_BUILD_COLLISION_REFERENCE_BENCHMARKS
  DART_BUILD_COLLISION_REFERENCE_TESTS
  DART_BUILD_DARTPY
  DART_BUILD_DEMOS_MEMORY_DIAGNOSTICS
  DART_BUILD_DIFF
  DART_BUILD_EXAMPLES
  DART_BUILD_GUI
  DART_BUILD_IO_USD
  DART_BUILD_MEMORY_DIAGNOSTICS
  DART_BUILD_PROFILE
  DART_BUILD_TESTS
  DART_BUILD_TUTORIALS
  DART_CODECOV
  DART_COLLISION_DEPRECATE_LEGACY_NAMES
  DART_COMPILER_CACHE
  DART_ENABLE_ASAN
  DART_ENABLE_EXPERIMENTAL_CUDA
  DART_ENABLE_GUI_FILAMENT_SMOKE_TESTS
  DART_ENABLE_SDFORMAT
  DART_ENABLE_SIMD
  DART_FAST_DEBUG
  DART_FIGURE13_EVIDENCE_BUILD
  DART_FORCE_COLORED_OUTPUT
  DART_NORMALIZE_BUILD_PATHS
  DART_PROFILE_BUILTIN
  DART_PROFILE_TRACY
  DART_SIMD_FORCE_SCALAR
  DART_SIMULATION_VERBOSE
  DART_STRICT_SYMBOL_VISIBILITY
  DART_USE_MOLD
  DART_USE_SYSTEM_BULLET
  DART_USE_SYSTEM_FILAMENT
  DART_USE_SYSTEM_FMT
  DART_USE_SYSTEM_GOOGLEBENCHMARK
  DART_USE_SYSTEM_GOOGLETEST
  DART_USE_SYSTEM_IMGUI
  DART_USE_SYSTEM_NANOBIND
  DART_USE_SYSTEM_ODE
  DART_USE_SYSTEM_TRACY
  DART_VERBOSE
)

function(dart_capture_build_configuration output_digest output_record)
  get_property(
    _dart_capture_configuration_digest
    GLOBAL
    PROPERTY DART_CAPTURE_BUILD_CONFIGURATION_DIGEST
  )
  get_property(
    _dart_capture_configuration_record
    GLOBAL
    PROPERTY DART_CAPTURE_BUILD_CONFIGURATION_RECORD
  )
  if(NOT _dart_capture_configuration_digest)
    set(
      _dart_capture_configuration_record
      "algorithm=sha256-cmake-build-configuration-record-v2\n"
    )
    foreach(
      _dart_capture_variable
      IN
      LISTS _DART_CAPTURE_BUILD_CONFIGURATION_VARIABLES
    )
      if(DEFINED ${_dart_capture_variable})
        set(_dart_capture_value "${${_dart_capture_variable}}")
      else()
        set(_dart_capture_value "<UNDEFINED>")
      endif()
      string(REPLACE "\\" "\\\\" _dart_capture_value "${_dart_capture_value}")
      string(REPLACE "\r" "\\r" _dart_capture_value "${_dart_capture_value}")
      string(REPLACE "\n" "\\n" _dart_capture_value "${_dart_capture_value}")
      string(
        APPEND _dart_capture_configuration_record
        "${_dart_capture_variable}=${_dart_capture_value}\n"
      )
    endforeach()
    string(
      SHA256 _dart_capture_configuration_digest
      "${_dart_capture_configuration_record}"
    )
    set_property(
      GLOBAL
      PROPERTY
        DART_CAPTURE_BUILD_CONFIGURATION_DIGEST
          "${_dart_capture_configuration_digest}"
    )
    set_property(
      GLOBAL
      PROPERTY
        DART_CAPTURE_BUILD_CONFIGURATION_RECORD
          "${_dart_capture_configuration_record}"
    )
  endif()
  set(${output_digest} "${_dart_capture_configuration_digest}" PARENT_SCOPE)
  set(${output_record} "${_dart_capture_configuration_record}" PARENT_SCOPE)
endfunction()

function(dart_enable_capture_source_provenance target)
  if(NOT TARGET ${target})
    message(FATAL_ERROR "Unknown provenance target: ${target}")
  endif()
  if(NOT Python3_EXECUTABLE)
    if(ARGV1 STREQUAL "REQUIRED")
      message(
        FATAL_ERROR
        "Python3 is required to generate capture source provenance"
      )
    endif()
    return()
  endif()

  set(
    _dart_capture_provenance_header
    "${CMAKE_BINARY_DIR}/generated/dart/capture_source_provenance.hpp"
  )
  if(NOT TARGET dart_capture_source_provenance_header)
    dart_capture_build_configuration(
      _dart_capture_configuration_digest
      _dart_capture_configuration_record
    )
    file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/generated/dart")
    file(
      WRITE "${CMAKE_BINARY_DIR}/generated/dart/capture_build_configuration.txt"
      "${_dart_capture_configuration_record}"
    )
    add_custom_target(
      dart_capture_source_provenance_header
      COMMAND
        "${Python3_EXECUTABLE}"
        "${CMAKE_SOURCE_DIR}/scripts/capture_source_provenance.py" --repo-root
        "${CMAKE_SOURCE_DIR}" --benchmark-source
        "${CMAKE_SOURCE_DIR}/tests/benchmark/simulation/bm_avbd_rigid_fixed_joint.cpp"
        --write-cpp-header "${_dart_capture_provenance_header}"
      BYPRODUCTS "${_dart_capture_provenance_header}"
      COMMENT "Refreshing capture source provenance"
      VERBATIM
    )
  endif()

  add_dependencies(${target} dart_capture_source_provenance_header)
  target_include_directories(${target} PRIVATE "${CMAKE_BINARY_DIR}/generated")
  dart_capture_build_configuration(
    _dart_capture_configuration_digest
    _dart_capture_configuration_record
  )
  target_compile_definitions(
    ${target}
    PRIVATE
      DART_CAPTURE_BUILD_CONFIGURATION_DIGEST="${_dart_capture_configuration_digest}"
  )
endfunction()

# Compile a queryable identity into each DART shared image. Hashing a loaded
# library only proves which bytes ran; the exported identity below additionally
# proves that those bytes were compiled from the capture-affecting source tree.
function(dart_embed_capture_source_provenance target)
  if(NOT TARGET ${target})
    message(FATAL_ERROR "Unknown provenance target: ${target}")
  endif()
  get_target_property(_dart_capture_target_type ${target} TYPE)
  if(
    NOT _dart_capture_target_type STREQUAL "SHARED_LIBRARY"
    AND NOT _dart_capture_target_type STREQUAL "MODULE_LIBRARY"
  )
    return()
  endif()
  if(NOT Python3_EXECUTABLE)
    # Ordinary C++-only configurations keep Python optional. Evidence builds
    # use the Pixi environment and therefore stamp every measured shared image.
    return()
  endif()

  dart_enable_capture_source_provenance(${target})
  target_sources(
    ${target}
    PRIVATE "${CMAKE_SOURCE_DIR}/cmake/dart_capture_source_provenance.cpp"
  )
  target_compile_definitions(
    ${target}
    PRIVATE
      DART_CAPTURE_BUILD_TARGET="${target}"
      DART_CAPTURE_CMAKE_BUILD_TYPE="$<CONFIG>"
      DART_CAPTURE_COMPILER_ID="${CMAKE_CXX_COMPILER_ID}"
      DART_CAPTURE_COMPILER_VERSION="${CMAKE_CXX_COMPILER_VERSION}"
  )
endfunction()
