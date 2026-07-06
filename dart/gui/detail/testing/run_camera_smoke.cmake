# Camera-control headless smoke: drives the WP-ASV.4 multi-view and turntable
# capture in a single process and asserts every per-view PPM is a valid,
# non-blank image, plus that two distinct views are actually framed
# differently. Mirrors run_headless_smoke.cmake's per-image checks and reuses
# analyze_headless_smoke.py (basic + compare modes). Software GL (llvmpipe) is
# supplied by the test ENVIRONMENT, matching the other GUI Filament smokes.

if(NOT DEFINED DART_GUI_FILAMENT_EXECUTABLE)
  message(FATAL_ERROR "DART_GUI_FILAMENT_EXECUTABLE is required")
endif()

if(NOT DEFINED DART_GUI_FILAMENT_SCREENSHOT)
  message(FATAL_ERROR "DART_GUI_FILAMENT_SCREENSHOT is required")
endif()

if(NOT DEFINED DART_GUI_FILAMENT_PYTHON)
  message(FATAL_ERROR "DART_GUI_FILAMENT_PYTHON is required")
endif()

if(NOT DEFINED DART_GUI_FILAMENT_ANALYZER)
  message(FATAL_ERROR "DART_GUI_FILAMENT_ANALYZER is required")
endif()

set(_width 640)
if(DEFINED DART_GUI_FILAMENT_WIDTH)
  set(_width "${DART_GUI_FILAMENT_WIDTH}")
endif()

set(_height 480)
if(DEFINED DART_GUI_FILAMENT_HEIGHT)
  set(_height "${DART_GUI_FILAMENT_HEIGHT}")
endif()

set(_frames 6)
if(DEFINED DART_GUI_FILAMENT_FRAMES)
  set(_frames "${DART_GUI_FILAMENT_FRAMES}")
endif()

# front (-90,0) vs three-quarter (-45,25) differ in both azimuth and elevation,
# so their framing diverges well clear of the epsilon while a duplicated frame
# would score ~0.
set(_views "front,three-quarter")
if(DEFINED DART_GUI_FILAMENT_VIEWS)
  set(_views "${DART_GUI_FILAMENT_VIEWS}")
endif()

set(_turntable 4)
if(DEFINED DART_GUI_FILAMENT_TURNTABLE)
  set(_turntable "${DART_GUI_FILAMENT_TURNTABLE}")
endif()

set(_min_divergence 1.5)
if(DEFINED DART_GUI_FILAMENT_MIN_DIVERGENCE)
  set(_min_divergence "${DART_GUI_FILAMENT_MIN_DIVERGENCE}")
endif()

if(NOT "${_width}" MATCHES "^[1-9][0-9]*$")
  message(FATAL_ERROR "DART_GUI_FILAMENT_WIDTH must be a positive integer")
endif()

if(NOT "${_height}" MATCHES "^[1-9][0-9]*$")
  message(FATAL_ERROR "DART_GUI_FILAMENT_HEIGHT must be a positive integer")
endif()

if(NOT "${_frames}" MATCHES "^[1-9][0-9]*$")
  message(FATAL_ERROR "DART_GUI_FILAMENT_FRAMES must be a positive integer")
endif()

if(NOT "${_turntable}" MATCHES "^[1-9][0-9]*$")
  message(FATAL_ERROR "DART_GUI_FILAMENT_TURNTABLE must be a positive integer")
endif()

get_filename_component(_dir "${DART_GUI_FILAMENT_SCREENSHOT}" DIRECTORY)
get_filename_component(_name "${DART_GUI_FILAMENT_SCREENSHOT}" NAME_WE)
get_filename_component(_ext "${DART_GUI_FILAMENT_SCREENSHOT}" EXT)
file(MAKE_DIRECTORY "${_dir}")

# Validate one written PPM: expected header, minimum size, and a non-blank
# pixel sample. Fails the whole smoke on any violation.
function(_dart_camera_smoke_validate_ppm path)
  if(NOT EXISTS "${path}")
    message(FATAL_ERROR "dartsim did not write ${path}")
  endif()
  file(SIZE "${path}" _size)
  set(_expected_header "P6\n${_width} ${_height}\n255\n")
  string(LENGTH "${_expected_header}" _header_size)
  file(READ "${path}" _actual_header LIMIT "${_header_size}")
  if(NOT "${_actual_header}" STREQUAL "${_expected_header}")
    message(
      FATAL_ERROR
      "${path} does not start with the expected ${_width}x${_height} binary PPM header"
    )
  endif()
  math(EXPR _expected_size "${_header_size} + ${_width} * ${_height} * 3")
  if(_size LESS _expected_size)
    message(
      FATAL_ERROR
      "${path} is ${_size} bytes, expected at least ${_expected_size}"
    )
  endif()
  file(READ "${path}" _sample OFFSET "${_header_size}" LIMIT 8192 HEX)
  if(NOT _sample MATCHES "[1-9a-fA-F]")
    message(FATAL_ERROR "${path} pixel sample contains only zero-valued pixels")
  endif()
  message(STATUS "camera smoke validated ${path} (${_size} bytes)")
endfunction()

# ---- Multi-view capture ----------------------------------------------------
set(
  _multiview_command
  "${DART_GUI_FILAMENT_EXECUTABLE}"
  --headless
  --frames
  "${_frames}"
  --width
  "${_width}"
  --height
  "${_height}"
  --screenshot
  "${DART_GUI_FILAMENT_SCREENSHOT}"
  --views
  "${_views}"
)
if(DEFINED DART_GUI_FILAMENT_SCENE)
  list(APPEND _multiview_command --scene "${DART_GUI_FILAMENT_SCENE}")
endif()

string(REPLACE "," ";" _view_list "${_views}")
foreach(_view IN LISTS _view_list)
  file(REMOVE "${_dir}/${_name}_${_view}${_ext}")
endforeach()

execute_process(
  COMMAND ${_multiview_command}
  RESULT_VARIABLE _result
  OUTPUT_VARIABLE _stdout
  ERROR_VARIABLE _stderr
)
if(NOT _stdout STREQUAL "")
  message(STATUS "multi-view stdout:\n${_stdout}")
endif()
if(NOT _stderr STREQUAL "")
  message(STATUS "multi-view stderr:\n${_stderr}")
endif()
if(NOT _result STREQUAL "0")
  message(
    FATAL_ERROR
    "dartsim multi-view capture failed with exit code ${_result}"
  )
endif()

set(_view_paths "")
foreach(_view IN LISTS _view_list)
  set(_view_path "${_dir}/${_name}_${_view}${_ext}")
  _dart_camera_smoke_validate_ppm("${_view_path}")
  list(APPEND _view_paths "${_view_path}")
endforeach()

# Two distinct views must be framed differently, not a duplicated frame.
list(LENGTH _view_paths _view_count)
if(_view_count LESS 2)
  message(FATAL_ERROR "camera smoke expected at least two views")
endif()
list(GET _view_paths 0 _view_a)
list(GET _view_paths 1 _view_b)
execute_process(
  COMMAND
    "${DART_GUI_FILAMENT_PYTHON}" "${DART_GUI_FILAMENT_ANALYZER}" "${_view_a}"
    --width "${_width}" --height "${_height}" --mode compare --compare
    "${_view_b}" --min-divergence "${_min_divergence}"
  RESULT_VARIABLE _compare_result
  OUTPUT_VARIABLE _compare_stdout
  ERROR_VARIABLE _compare_stderr
)
if(NOT _compare_stdout STREQUAL "")
  message(STATUS "two-view compare:\n${_compare_stdout}")
endif()
if(NOT _compare_stderr STREQUAL "")
  message(STATUS "two-view compare stderr:\n${_compare_stderr}")
endif()
if(NOT _compare_result STREQUAL "0")
  message(FATAL_ERROR "camera smoke two-view divergence check failed")
endif()

# ---- Turntable capture -----------------------------------------------------
set(_turntable_screenshot "${_dir}/${_name}_turntable${_ext}")
set(
  _turntable_command
  "${DART_GUI_FILAMENT_EXECUTABLE}"
  --headless
  --frames
  "${_frames}"
  --width
  "${_width}"
  --height
  "${_height}"
  --screenshot
  "${_turntable_screenshot}"
  --view
  three-quarter
  --turntable
  "${_turntable}"
)
if(DEFINED DART_GUI_FILAMENT_SCENE)
  list(APPEND _turntable_command --scene "${DART_GUI_FILAMENT_SCENE}")
endif()

execute_process(
  COMMAND ${_turntable_command}
  RESULT_VARIABLE _tt_result
  OUTPUT_VARIABLE _tt_stdout
  ERROR_VARIABLE _tt_stderr
)
if(NOT _tt_stdout STREQUAL "")
  message(STATUS "turntable stdout:\n${_tt_stdout}")
endif()
if(NOT _tt_stderr STREQUAL "")
  message(STATUS "turntable stderr:\n${_tt_stderr}")
endif()
if(NOT _tt_result STREQUAL "0")
  message(
    FATAL_ERROR
    "dartsim turntable capture failed with exit code ${_tt_result}"
  )
endif()

math(EXPR _last_frame "${_turntable} - 1")
foreach(_index RANGE 0 ${_last_frame})
  # Zero-pad to at least three digits (matches the C++ turntable naming).
  string(LENGTH "${_last_frame}" _index_width)
  if(_index_width LESS 3)
    set(_index_width 3)
  endif()
  string(LENGTH "${_index}" _cur_width)
  set(_padded "${_index}")
  while(_cur_width LESS _index_width)
    set(_padded "0${_padded}")
    string(LENGTH "${_padded}" _cur_width)
  endwhile()
  _dart_camera_smoke_validate_ppm("${_dir}/${_name}_turntable_turn${_padded}${_ext}")
endforeach()

message(
  STATUS
  "dartsim camera smoke passed (${_view_count} views, ${_turntable} turntable frames)"
)
