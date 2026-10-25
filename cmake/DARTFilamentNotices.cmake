# Preserve the pinned SDK's upstream runtime notices without changing their text.
function(dart_filament_validate_root_license filename expected_sha256)
  file(READ "${filename}" _license_text)
  string(REPLACE "\r\n" "\n" _license_text "${_license_text}")
  string(SHA256 _license_sha256 "${_license_text}")
  if(NOT _license_sha256 STREQUAL expected_sha256)
    message(FATAL_ERROR "The pinned Filament root license changed: ${filename}")
  endif()
endfunction()

# Offline, a notice that is neither staged nor in the supplied tree cannot be
# downloaded: name the ways to provide it, in dart_check_fetchcontent's style.
function(dart_filament_offline_notice_error notice)
  message(
    FATAL_ERROR
    "Cannot fetch the Filament runtime notice ${notice} with FETCHCONTENT_FULLY_DISCONNECTED=ON: pre-seed FETCHCONTENT_SOURCE_DIR_DART_FILAMENT_SDK with the Filament 1.77.3 source tree, or set DART_BUILD_GUI=OFF for a headless build."
  )
endfunction()

function(dart_filament_stage_notices prefix)
  set(_source "${ARGV1}")
  set(
    _notices
    "LICENSE|02d70b593dd8e9355dd8e1ca8d13c303bee5bcaf3065b66ff6e4f534e1e6a2c1"
    "third_party/smol-v/LICENSE|9cea538cc1ee6680f4d13e212bfef5b581cd5d873cbedf6dc07699dbe7e69651"
    "third_party/zstd/LICENSE|7055266497633c9025b777c78eb7235af13922117480ed5c674677adc381c9d8"
    "third_party/zstd/lib/common/xxhash.h|8cb837b21a8fe9a6b9dbcd0961ab16e733bfcbfa9e003f3a496ce07ae80aa8ee"
    "third_party/robin-map/LICENSE|11f2c685ba565a31aef1219f63a7a8845602c97bff70016b9e3278dfe51a2ec5"
    "third_party/vkmemalloc/LICENSE.txt|52df2c03d6cfc9ffec13c9d3626c530fc9ce0cbe41d5ea3d10cd46edeb1aeb38"
    "third_party/meshoptimizer/LICENSE|063805e3b8e24ceab4c5dea358b03245ee41c31be87cf65c61d836636cb494bb"
    "third_party/spirv-headers/LICENSE|ea43b1de38a6f90c488800d66dec1ed671e68cda530266bc96951fb5b6307613"
    "third_party/mikktspace/include/mikktspace/mikktspace.h|17fc433894f24c73753d548086cc4d8c5c0379f4a6edfb98b5da243e4f0bc3d0"
    "filament/src/materials/sgsr/LICENSE.txt|bc496633d3db8c6ca1362216ec382d361b69c7d5518acd6bbfd91503febfb3ef"
    "filament/src/materials/fsr/license.txt|db089274ce766da70f5b7d791029c3486f9f9e27c8c79c652689603d3192e802"
    "filament/src/materials/fsr/ffx_a.h|b909a073d53eb5319a6a3b36256bf961849d608bfa7bf78549c89de4eb2775d8"
    "filament/src/materials/antiAliasing/fxaa/fxaa.fs|8783fbba5dd0b6fc22368a6a8b8d407a2d99516afe0898ae93d4d990912da084"
    "shaders/src/surface_light_reflections.fs|e8c6ffcba3c5f36e872124f6afcf90b55779f3e69d7893af1e3f6c3911c3c02e"
    "filament/src/ColorSpaceUtils.cpp|30ce340920bea62d0897e490ce44b0d8c68885d4dc7142130dd77501d90e3e00"
    "filament/src/ToneMapper.cpp|1e3212b67f2954721a4336c68fef1904204873835896e25c9ba77f9030aa42cd"
    "libs/bluegl/include/GL/glcorearb.h|3d4d3bcde6b2661b4e519b37a8c6eb1b559dd210a40f0d50535b347a9d3e4062"
    "libs/bluegl/include/GL/glext.h|2b4231e98c3739e252dee96e6213dd8007d83bdeb9de3b8a5a7b0f4f9689cf83"
    "libs/bluevk/include/vulkan/vulkan_core.h|50af5a157c8aab7d90dcd929a05758b4dc3e78a619f46552bfd5cde67b2d46c1"
    "libs/bluevk/include/vulkan/vk_platform.h|c4cabbcf699c90cde344095ae54da9435d117951aad114ba3a540c5997d7906f"
    "third_party/abseil/LICENSE|c79a7fea0e3cac04cd43f20e7b648e5a0ff8fa5344e644b0ee09ca1162b62747"
  )
  foreach(_entry IN LISTS _notices)
    string(REPLACE "|" ";" _parts "${_entry}")
    list(GET _parts 0 _notice)
    list(GET _parts 1 _sha256)
    if(_notice STREQUAL "LICENSE")
      set(_target "${prefix}/LICENSE")
      # Release archives preserve their own root license, including CRLF.
      if(EXISTS "${_target}")
        dart_filament_validate_root_license("${_target}" "${_sha256}")
        continue()
      endif()
    else()
      set(_target "${prefix}/notices/${_notice}")
    endif()
    get_filename_component(_notice_dir "${_target}" DIRECTORY)
    file(MAKE_DIRECTORY "${_notice_dir}")
    if(_source)
      if(FETCHCONTENT_FULLY_DISCONNECTED AND NOT EXISTS "${_source}/${_notice}")
        dart_filament_offline_notice_error("${_notice}")
      endif()
      file(SHA256 "${_source}/${_notice}" _source_sha256)
      if(NOT _source_sha256 STREQUAL _sha256)
        message(
          FATAL_ERROR
          "The pinned Filament runtime notice changed: ${_notice}"
        )
      endif()
      configure_file("${_source}/${_notice}" "${_target}" COPYONLY)
    else()
      set(_cached_sha256 "")
      if(EXISTS "${_target}")
        file(SHA256 "${_target}" _cached_sha256)
      endif()
      if(NOT _cached_sha256 STREQUAL _sha256)
        if(FETCHCONTENT_FULLY_DISCONNECTED)
          dart_filament_offline_notice_error("${_notice}")
        endif()
        file(
          DOWNLOAD
            "https://raw.githubusercontent.com/google/filament/v1.77.3/${_notice}"
          "${_target}"
          EXPECTED_HASH "SHA256=${_sha256}"
          TLS_VERIFY TRUE
          TIMEOUT 60
          INACTIVITY_TIMEOUT 30
          STATUS _download_status
        )
        list(GET _download_status 0 _download_result)
        if(NOT _download_result EQUAL 0)
          message(
            FATAL_ERROR
            "Cannot fetch the pinned Filament runtime notice: ${_notice}; ${_download_status}"
          )
        endif()
      endif()
    endif()
  endforeach()
endfunction()
