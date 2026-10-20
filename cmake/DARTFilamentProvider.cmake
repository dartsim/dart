# Use a packaged SDK for development and pinned static SDKs for wheels.
if(NOT DART_USE_SYSTEM_FILAMENT)
  if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    include(DARTFilamentSource)
    dart_filament_build_source_sdk(Filament_ROOT)
  else()
    if(APPLE)
      if(
        NOT CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm64|aarch64)$"
        OR
          (
            CMAKE_OSX_ARCHITECTURES
            AND NOT CMAKE_OSX_ARCHITECTURES STREQUAL "arm64"
          )
      )
        message(
          FATAL_ERROR
          "The pinned Filament 1.77.3 macOS archive is arm64 only; use DART_USE_SYSTEM_FILAMENT=ON and an Intel/universal SDK for this architecture"
        )
      endif()
      set(_filament_platform mac)
      set(
        _filament_sha256
        c1ef146a7efa504fcdc3aa06aa2db9b2f89d91436fcb87f2b4c15c95862ffeb8
      )
    elseif(WIN32 AND CMAKE_SIZEOF_VOID_P EQUAL 8)
      set(_filament_platform windows)
      set(
        _filament_sha256
        e8705d817b7a884fa2d185080ceb80dd42c4ba339930d49dedc79b5761215285
      )
    else()
      message(
        FATAL_ERROR
        "No pinned Filament SDK for ${CMAKE_SYSTEM_NAME}-${CMAKE_SYSTEM_PROCESSOR}; use DART_USE_SYSTEM_FILAMENT=ON and Filament_ROOT"
      )
    endif()
    include(FetchContent)
    # The timestamp option needs CMake 3.24; DART supports 3.22.1.
    set(_dart_filament_timestamp)
    if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.24)
      set(_dart_filament_timestamp DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
    endif()
    FetchContent_Declare(
      dart_filament_sdk
      URL
        "https://github.com/google/filament/releases/download/v1.77.3/filament-v1.77.3-${_filament_platform}.tgz"
      URL_HASH "SHA256=${_filament_sha256}"
      ${_dart_filament_timestamp}
    )
    FetchContent_MakeAvailable(dart_filament_sdk)
    set(Filament_ROOT "${dart_filament_sdk_SOURCE_DIR}")
    if(EXISTS "${Filament_ROOT}/filament/include")
      string(APPEND Filament_ROOT "/filament")
    endif()
    include(DARTFilamentNotices)
    dart_filament_stage_notices("${Filament_ROOT}")
  endif()
  if(NOT EXISTS "${Filament_ROOT}/LICENSE")
    message(FATAL_ERROR "The fetched Filament SDK is missing its license")
  endif()
  set(DART_FILAMENT_LICENSE_DIRECTORY "${Filament_ROOT}")
endif()

set(DART_FILAMENT_REQUIRE_MATC TRUE)
find_package(Filament MODULE)
unset(DART_FILAMENT_REQUIRE_MATC)
if(NOT Filament_FOUND)
  message(
    FATAL_ERROR
    "The DART GUI requires a Filament 1.77.3 SDK with matc: install one or set "
    "Filament_ROOT, use DART_USE_SYSTEM_FILAMENT=OFF for DART's pinned SDK, or "
    "set DART_BUILD_GUI=OFF for a headless build."
  )
endif()

if(DART_USE_SYSTEM_FILAMENT)
  # SDK packages often omit their installed license tree. Stage the supported
  # SDK notices for native distributions, reusing a complete SDK tree if present.
  include(DARTFilamentNotices)
  set(
    DART_FILAMENT_LICENSE_DIRECTORY
    "${CMAKE_CURRENT_BINARY_DIR}/filament_licenses"
  )
  set(_filament_notice_source "")
  get_filename_component(
    _filament_include_root
    "${Filament_INCLUDE_DIR}/.."
    ABSOLUTE
  )
  foreach(_filament_root "${Filament_ROOT}" "${_filament_include_root}")
    if(
      EXISTS "${_filament_root}/LICENSE"
      AND IS_DIRECTORY "${_filament_root}/notices"
    )
      file(MAKE_DIRECTORY "${DART_FILAMENT_LICENSE_DIRECTORY}")
      configure_file(
        "${_filament_root}/LICENSE"
        "${DART_FILAMENT_LICENSE_DIRECTORY}/LICENSE"
        COPYONLY
      )
      set(_filament_notice_source "${_filament_root}/notices")
      break()
    endif()
  endforeach()
  dart_filament_stage_notices("${DART_FILAMENT_LICENSE_DIRECTORY}" "${_filament_notice_source}")
endif()
file(
  WRITE "${PROJECT_BINARY_DIR}/dartpy-filament-license.txt"
  "${DART_FILAMENT_LICENSE_DIRECTORY}/LICENSE\n"
)

find_package(glfw3 3.4 CONFIG)
find_package(PNG)
find_package(JPEG)
foreach(_dart_gui_dependency glfw3 PNG JPEG)
  if(NOT ${_dart_gui_dependency}_FOUND)
    message(
      FATAL_ERROR
      "The DART GUI requires GLFW 3.4, libpng and libjpeg (${_dart_gui_dependency} "
      "was not found); install them or set DART_BUILD_GUI=OFF for a headless "
      "build."
    )
  endif()
endforeach()

# matc reports the material format version, which the runtime header must match.
get_target_property(_filament_matc Filament::matc IMPORTED_LOCATION)
get_target_property(
  _filament_includes
  Filament::filament
  INTERFACE_INCLUDE_DIRECTORIES
)
foreach(_filament_include IN LISTS _filament_includes)
  if(EXISTS "${_filament_include}/filament/MaterialEnums.h")
    file(
      STRINGS "${_filament_include}/filament/MaterialEnums.h"
      _filament_version_line
      REGEX "MATERIAL_VERSION = [0-9]+"
    )
    string(
      REGEX MATCH
      "MATERIAL_VERSION = ([0-9]+)"
      _filament_match
      "${_filament_version_line}"
    )
    set(_filament_material_version "${CMAKE_MATCH_1}")
    break()
  endif()
endforeach()
execute_process(
  COMMAND "${_filament_matc}" --version
  RESULT_VARIABLE _filament_matc_result
  OUTPUT_VARIABLE _filament_matc_version
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
if(
  NOT _filament_matc_result EQUAL 0
  OR NOT _filament_material_version
  OR NOT _filament_matc_version STREQUAL _filament_material_version
)
  message(
    FATAL_ERROR
    "Filament matc/runtime material versions disagree: ${_filament_matc_version}/${_filament_material_version}"
  )
endif()
# The SDK exposes no release version of its own: no version header, no CMake
# version file, and no pkg-config module, while `matc --version` prints the
# material format version. Upstream bumps that version with each minor release
# (1.77.x ships 77), so enforce the 1.77.3 pin through it.
if(NOT _filament_material_version STREQUAL "77")
  message(
    FATAL_ERROR
    "Found Filament SDK with material version ${_filament_material_version}, "
    "but DART requires Filament 1.77.3 (material version 77): install that "
    "SDK, use DART_USE_SYSTEM_FILAMENT=OFF for DART's pinned SDK, or set "
    "DART_BUILD_GUI=OFF for a headless build."
  )
endif()
