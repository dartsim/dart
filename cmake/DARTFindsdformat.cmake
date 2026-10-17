# sdformat's package and target names vary between distributions and majors.
# Keep the selected major for installed consumers, including static IO builds.
set(sdformat_FOUND FALSE)
set(SDFORMAT_FOUND FALSE)
set(_dart_sdf_accepted FALSE)
set(SDFORMAT_LIBRARIES "")
set(DART_SDFORMAT_PACKAGE "")
set(DART_SDFORMAT_MAJOR "")

if(DART_SDFORMAT_REQUIRED_PACKAGE)
  set(_dart_sdf_packages "${DART_SDFORMAT_REQUIRED_PACKAGE}")
else()
  find_package(sdformat QUIET CONFIG)
  set(_dart_sdf_packages sdformat16 sdformat15 sdformat14)
endif()

foreach(_dart_sdf_package IN LISTS _dart_sdf_packages)
  find_package(${_dart_sdf_package} QUIET CONFIG)
  set(_dart_sdf_candidates "${_dart_sdf_package}")
  if(NOT DART_SDFORMAT_REQUIRED_PACKAGE AND sdformat_FOUND)
    list(APPEND _dart_sdf_candidates sdformat)
  endif()
  foreach(_dart_sdf_candidate IN LISTS _dart_sdf_candidates)
    set(_dart_sdf_version "${${_dart_sdf_candidate}_VERSION}")
    if(
      NOT ${_dart_sdf_candidate}_FOUND
      OR NOT _dart_sdf_version MATCHES "^([0-9]+)[.]"
    )
      continue()
    endif()
    set(_dart_sdf_major "${CMAKE_MATCH_1}")
    if(_dart_sdf_version VERSION_LESS 14.8 OR _dart_sdf_major GREATER 16)
      continue()
    endif()
    if(DART_SDFORMAT_REQUIRED_PACKAGE)
      if(NOT _dart_sdf_major STREQUAL DART_SDFORMAT_REQUIRED_MAJOR)
        continue()
      endif()
    else()
      string(
        REPLACE
        sdformat
        ""
        _dart_sdf_requested_major
        "${_dart_sdf_package}"
      )
      if(NOT _dart_sdf_major STREQUAL _dart_sdf_requested_major)
        continue()
      endif()
    endif()
    foreach(
      _dart_sdf_target
      "${_dart_sdf_candidate}::${_dart_sdf_candidate}"
      "sdformat${_dart_sdf_major}::sdformat${_dart_sdf_major}"
      "sdformat::sdformat${_dart_sdf_major}"
      sdformat::sdformat
    )
      if(TARGET "${_dart_sdf_target}")
        set(SDFORMAT_LIBRARIES "${_dart_sdf_target}")
        set(DART_SDFORMAT_PACKAGE "${_dart_sdf_candidate}")
        set(DART_SDFORMAT_MAJOR "${_dart_sdf_major}")
        set(sdformat_VERSION "${_dart_sdf_version}")
        set(_dart_sdf_accepted TRUE)
        break()
      endif()
    endforeach()
    if(_dart_sdf_accepted)
      break()
    endif()
  endforeach()
  if(_dart_sdf_accepted)
    break()
  endif()
endforeach()
set(sdformat_FOUND ${_dart_sdf_accepted})
set(SDFORMAT_FOUND ${_dart_sdf_accepted})
if(NOT _dart_sdf_accepted)
  message(
    STATUS
    "sdformat 14 (>= 14.8), 15 or 16 with a matching CMake target is required"
  )
endif()
