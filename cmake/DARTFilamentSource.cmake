# Linux release archives require newer glibc and a different exception runtime.
# Keep SDK compiler flags private and use the active wheel environment's sysroot.
function(dart_filament_build_source_sdk output)
  include(FetchContent)
  set(_dart_filament_timestamp DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
  FetchContent_Declare(
    dart_filament_sdk
    URL "https://codeload.github.com/google/filament/tar.gz/refs/tags/v1.77.3"
    URL_HASH
      SHA256=6f0904a5092760a04831a6be4fb53884488d6deded8c3e6aa541ba711c50e61b
    ${_dart_filament_timestamp}
    # Populate without adding upstream targets or compiler flags to DART.
    SOURCE_SUBDIR
    dart-private-sdk
  )
  FetchContent_MakeAvailable(dart_filament_sdk)
  set(_source "${dart_filament_sdk_SOURCE_DIR}")
  set(_build "${dart_filament_sdk_BINARY_DIR}")
  set(_prefix "${_build}/sdk")
  if(NOT EXISTS "${_source}/filament/backend/CMakeLists.txt")
    message(
      FATAL_ERROR
      "The Linux Filament provider requires the pinned source tree"
    )
  endif()
  get_filename_component(_compiler_dir "${CMAKE_CXX_COMPILER}" DIRECTORY)
  get_filename_component(_compiler_name "${CMAKE_CXX_COMPILER}" NAME)
  string(
    REGEX REPLACE
    "(g\\+\\+|c\\+\\+)$"
    "clang++"
    _clang_name
    "${_compiler_name}"
  )
  set(_clang_candidates clang++-22 clang++-21 clang++-20 clang++)
  if(_clang_name MATCHES "clang\\+\\+" AND NOT _clang_name STREQUAL "clang++")
    list(PREPEND _clang_candidates "${_clang_name}")
  endif()
  # Take the first candidate that is Clang 20 or newer; versioned GCC names
  # and older Clang releases fall through to the next one.
  if(NOT DART_FILAMENT_SOURCE_CXX_COMPILER)
    foreach(_candidate IN LISTS _clang_candidates)
      unset(_clang)
      find_program(
        _clang
        NAMES "${_candidate}"
        HINTS "${_compiler_dir}" NO_CACHE
      )
      if(NOT _clang)
        continue()
      endif()
      execute_process(
        COMMAND "${_clang}" --version
        OUTPUT_VARIABLE _compiler_version
        RESULT_VARIABLE _compiler_result
      )
      if(
        _compiler_result EQUAL 0
        AND _compiler_version MATCHES "clang version ([0-9]+)"
        AND NOT CMAKE_MATCH_1 LESS 20
      )
        set(
          DART_FILAMENT_SOURCE_CXX_COMPILER
          "${_clang}"
          CACHE FILEPATH
          "Clang compiler for the private Linux Filament SDK"
        )
        break()
      endif()
    endforeach()
  endif()
  if(NOT DART_FILAMENT_SOURCE_CXX_COMPILER)
    message(
      FATAL_ERROR
      "The private Filament SDK requires Clang 20 or newer; install it or set "
      "DART_FILAMENT_SOURCE_CXX_COMPILER"
    )
  endif()
  execute_process(
    COMMAND "${DART_FILAMENT_SOURCE_CXX_COMPILER}" --version
    OUTPUT_VARIABLE _compiler_version
    RESULT_VARIABLE _compiler_result
  )
  if(
    NOT _compiler_result EQUAL 0
    OR NOT _compiler_version MATCHES "clang version ([0-9]+)"
  )
    message(FATAL_ERROR "DART_FILAMENT_SOURCE_CXX_COMPILER must be Clang")
  endif()
  if(CMAKE_MATCH_1 LESS 20)
    message(FATAL_ERROR "The private Filament SDK requires Clang 20 or newer")
  endif()
  get_filename_component(
    _clang_dir
    "${DART_FILAMENT_SOURCE_CXX_COMPILER}"
    DIRECTORY
  )
  get_filename_component(
    _clang_name
    "${DART_FILAMENT_SOURCE_CXX_COMPILER}"
    NAME
  )
  string(REPLACE "clang++" "clang" _clang_c_name "${_clang_name}")
  find_program(
    DART_FILAMENT_SOURCE_C_COMPILER
    NAMES "${_clang_c_name}"
    HINTS "${_clang_dir}"
    DOC "Matching C compiler for the private Linux Filament SDK"
    REQUIRED
  )
  find_program(_filament_ninja NAMES ninja REQUIRED)
  set(_log "${CMAKE_CURRENT_BINARY_DIR}/filament-source-build.log")
  message(
    STATUS
    "Preparing pinned Filament 1.77.3 with libstdc++; SDK build log: ${_log}"
  )
  execute_process(
    COMMAND
      "${CMAKE_COMMAND}" -S "${_source}" -B "${_build}" -G Ninja
      "-DCMAKE_MAKE_PROGRAM=${_filament_ninja}"
      "-DCMAKE_C_COMPILER=${DART_FILAMENT_SOURCE_C_COMPILER}"
      "-DCMAKE_CXX_COMPILER=${DART_FILAMENT_SOURCE_CXX_COMPILER}"
      "-DCMAKE_CXX_FLAGS=-stdlib=libstdc++"
      "-DCMAKE_EXE_LINKER_FLAGS=${CMAKE_EXE_LINKER_FLAGS}"
      "-DCMAKE_PROJECT_INCLUDE=${CMAKE_CURRENT_FUNCTION_LIST_DIR}/DARTFilamentLibstdcxx.cmake"
      "-DCMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}"
      "-DCMAKE_INSTALL_PREFIX=${_prefix}" -DCMAKE_BUILD_TYPE=Release
      # The static SDK, including its zstd, links into the shared dartpy module.
      -DBUILD_SHARED_LIBS=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=ON
      -DFILAMENT_SUPPORTS_OPENGL=ON -DFILAMENT_SUPPORTS_VULKAN=ON
      -DFILAMENT_SUPPORTS_WEBGPU=OFF -DFILAMENT_SUPPORTS_XLIB=ON
      -DFILAMENT_SUPPORTS_XCB=OFF -DFILAMENT_SKIP_SAMPLES=ON
      -DFILAMENT_SKIP_SDL2=ON -DFILAMENT_BUILD_TESTING=OFF
      -DUSE_STATIC_LIBCXX=OFF
    RESULT_VARIABLE _configure_result
    OUTPUT_FILE "${_log}"
    ERROR_FILE "${_log}"
  )
  if(NOT _configure_result EQUAL 0)
    message(FATAL_ERROR "Filament source SDK configure failed; see ${_log}")
  endif()
  set(
    DART_FILAMENT_SOURCE_JOBS
    4
    CACHE STRING
    "Parallel jobs for the private Filament SDK build"
  )
  execute_process(
    COMMAND
      "${CMAKE_COMMAND}" --build "${_build}" --parallel
      "${DART_FILAMENT_SOURCE_JOBS}" --target filament matc geometry
    RESULT_VARIABLE _build_result
    OUTPUT_FILE "${_log}"
    ERROR_FILE "${_log}"
  )
  if(NOT _build_result EQUAL 0)
    message(FATAL_ERROR "Filament source SDK build failed; see ${_log}")
  endif()
  # Install the required closure, excluding asset tools and samples.
  foreach(
    _directory
    IN
    ITEMS
      filament
      libs/utils
      libs/math
      libs/filabridge
      libs/filaflat
      libs/geometry
      libs/bluegl
      libs/bluevk
      shaders
      third_party/smol-v/tnt
      third_party/zstd/tnt
      tools/matc
  )
    execute_process(
      COMMAND
        "${CMAKE_COMMAND}" "-DCMAKE_INSTALL_PREFIX=${_prefix}"
        -DCMAKE_INSTALL_CONFIG_NAME=Release -P
        "${_build}/${_directory}/cmake_install.cmake"
      RESULT_VARIABLE _install_result
      OUTPUT_QUIET
    )
    if(NOT _install_result EQUAL 0)
      message(FATAL_ERROR "Filament SDK staging failed for ${_directory}")
    endif()
  endforeach()
  # BlueVK's public header includes the Vulkan headers Filament pins; hosts
  # such as the manylinux image provide none.
  foreach(_bluevk_include bluevk vk_video vulkan)
    file(
      COPY "${_source}/libs/bluevk/include/${_bluevk_include}"
      DESTINATION "${_prefix}/include"
    )
  endforeach()
  include(DARTFilamentNotices)
  dart_filament_stage_notices("${_prefix}" "${_source}")
  set(${output} "${_prefix}" PARENT_SCOPE)
endfunction()
