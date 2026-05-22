if(NOT DEFINED DART_COLLISION_SANDBOX_EXECUTABLE)
  message(FATAL_ERROR "DART_COLLISION_SANDBOX_EXECUTABLE is required")
endif()

if(NOT DEFINED DART_COLLISION_SANDBOX_OUTPUT_DIR)
  message(FATAL_ERROR "DART_COLLISION_SANDBOX_OUTPUT_DIR is required")
endif()

set(_width 640)
set(_height 480)
set(_ui_width 1280)
set(_ui_height 720)
set(_frames 2)
set(_pairs
  sphere_box
  box_box
  box_capsule
  sphere_plane
  sphere_sdf
  compound_compound
)

file(MAKE_DIRECTORY "${DART_COLLISION_SANDBOX_OUTPUT_DIR}")

function(_dart_collision_sandbox_check_ppm screenshot width height)
  if(NOT EXISTS "${screenshot}")
    message(FATAL_ERROR "collision_sandbox did not write ${screenshot}")
  endif()

  file(SIZE "${screenshot}" _screenshot_size)
  set(_expected_header "P6\n${width} ${height}\n255\n")
  string(LENGTH "${_expected_header}" _header_size)
  file(READ "${screenshot}" _actual_header LIMIT "${_header_size}")
  if(NOT "${_actual_header}" STREQUAL "${_expected_header}")
    message(
      FATAL_ERROR
      "${screenshot} does not start with the expected ${width}x${height} binary PPM header"
    )
  endif()

  math(EXPR _expected_size "${_header_size} + ${width} * ${height} * 3")
  if(_screenshot_size LESS _expected_size)
    message(
      FATAL_ERROR
      "${screenshot} is ${_screenshot_size} bytes, expected at least ${_expected_size}"
    )
  endif()

  file(READ "${screenshot}" _pixel_sample OFFSET "${_header_size}" LIMIT 8192 HEX)
  if(NOT _pixel_sample MATCHES "[1-9a-fA-F]")
    message(FATAL_ERROR "${screenshot} sample contains only zero-valued pixels")
  endif()
endfunction()

function(_dart_collision_sandbox_run_pair pair show_ui width height)
  set(_suffix "${pair}")
  set(_ui_args "")
  if(show_ui)
    set(_suffix "${pair}_ui")
    set(_ui_args --show-ui)
  endif()

  set(_screenshot "${DART_COLLISION_SANDBOX_OUTPUT_DIR}/${_suffix}.ppm")
  file(REMOVE "${_screenshot}")
  execute_process(
    COMMAND
      "${DART_COLLISION_SANDBOX_EXECUTABLE}"
      --headless
      ${_ui_args}
      --frames "${_frames}"
      --width "${width}"
      --height "${height}"
      --pair "${pair}"
      --screenshot "${_screenshot}"
    RESULT_VARIABLE _result
    OUTPUT_VARIABLE _stdout
    ERROR_VARIABLE _stderr
  )

  if(NOT _stdout STREQUAL "")
    message(STATUS "collision_sandbox ${pair} stdout:\n${_stdout}")
  endif()
  if(NOT _stderr STREQUAL "")
    message(STATUS "collision_sandbox ${pair} stderr:\n${_stderr}")
  endif()
  if(NOT _result STREQUAL "0")
    message(
      FATAL_ERROR
      "collision_sandbox ${pair} failed with exit code ${_result}"
    )
  endif()

  _dart_collision_sandbox_check_ppm("${_screenshot}" "${width}" "${height}")
endfunction()

foreach(_pair IN LISTS _pairs)
  _dart_collision_sandbox_run_pair("${_pair}" FALSE "${_width}" "${_height}")
endforeach()
_dart_collision_sandbox_run_pair(
  sphere_box TRUE "${_ui_width}" "${_ui_height}"
)

message(
  STATUS
  "collision_sandbox headless sweep wrote screenshots in ${DART_COLLISION_SANDBOX_OUTPUT_DIR}"
)
