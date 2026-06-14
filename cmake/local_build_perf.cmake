# Copyright (c) The DART development contributors
# Distributed under the BSD-style license; see LICENSE for details.

include_guard(GLOBAL)

# Opt-in local build-speed knobs. Both default OFF so CI and the default
# GUI/CUDA builds are byte-for-byte unchanged; developers enable them per host.
# See docs/design/local_verification_pipeline.md.

dart_option(
  DART_USE_MOLD
  "Use the mold linker when available (Linux, non-CUDA builds). Off by default."
  OFF
  CATEGORY performance
)

dart_option(
  DART_NORMALIZE_BUILD_PATHS
  "Rewrite absolute source paths in objects (-ffile-prefix-map) so multiple clones share one compiler cache. Off by default and changes __FILE__."
  OFF
  CATEGORY performance
)

# Select the mold linker. mold is GNU-ld compatible and dramatically cuts link
# time, which dominates DART's build (hundreds of test executables). Scoped away
# from the CUDA build, whose pixi config performs hand-rolled linker surgery.
function(dart_configure_linker)
  if(NOT DART_USE_MOLD)
    return()
  endif()

  if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
    message(STATUS "DART_USE_MOLD ignored: mold wiring is Linux-only")
    return()
  endif()

  if(DART_ENABLE_EXPERIMENTAL_CUDA)
    message(
      STATUS
      "DART_USE_MOLD ignored: the CUDA build path manages its own linker"
    )
    return()
  endif()

  find_program(DART_MOLD_EXECUTABLE NAMES mold)
  if(NOT DART_MOLD_EXECUTABLE)
    message(STATUS "DART_USE_MOLD requested but mold was not found on PATH")
    return()
  endif()

  # Confirm the compiler actually accepts the linker before committing to it.
  include(CheckLinkerFlag OPTIONAL RESULT_VARIABLE _dart_have_check_linker_flag)
  if(_dart_have_check_linker_flag)
    check_linker_flag(CXX "-fuse-ld=mold" DART_LINKER_ACCEPTS_MOLD)
    if(NOT DART_LINKER_ACCEPTS_MOLD)
      message(
        STATUS
        "DART_USE_MOLD requested but the compiler rejects -fuse-ld=mold"
      )
      return()
    endif()
  endif()

  # First-class linker selection; propagates to all targets defined later.
  set(CMAKE_LINKER_TYPE MOLD PARENT_SCOPE)
  message(STATUS "Linker: mold via CMAKE_LINKER_TYPE [${DART_MOLD_EXECUTABLE}]")
endfunction()

# Normalize the per-clone absolute source root in emitted objects so sccache/
# ccache entries are shared across clones/worktrees. This changes __FILE__ (and
# therefore assert/log paths), hence opt-in.
function(dart_configure_path_normalization)
  if(NOT DART_NORMALIZE_BUILD_PATHS)
    return()
  endif()

  if(NOT (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang"))
    message(
      STATUS
      "DART_NORMALIZE_BUILD_PATHS ignored: requires a GCC/Clang compiler"
    )
    return()
  endif()

  add_compile_options("-ffile-prefix-map=${CMAKE_SOURCE_DIR}=/dart")
  message(
    STATUS
    "Build-path normalization: -ffile-prefix-map=${CMAKE_SOURCE_DIR}=/dart"
  )
endfunction()
