if(NOT DEFINED DART_GUI_FILAMENT_EXECUTABLE)
  message(FATAL_ERROR "DART_GUI_FILAMENT_EXECUTABLE is required")
endif()

if(NOT DEFINED DART_GUI_FILAMENT_SCREENSHOT)
  message(FATAL_ERROR "DART_GUI_FILAMENT_SCREENSHOT is required")
endif()

set(_width 640)
if(DEFINED DART_GUI_FILAMENT_WIDTH)
  set(_width "${DART_GUI_FILAMENT_WIDTH}")
endif()

set(_height 480)
if(DEFINED DART_GUI_FILAMENT_HEIGHT)
  set(_height "${DART_GUI_FILAMENT_HEIGHT}")
endif()

set(_frames 10)
if(DEFINED DART_GUI_FILAMENT_FRAMES)
  set(_frames "${DART_GUI_FILAMENT_FRAMES}")
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

set(_command
  "${DART_GUI_FILAMENT_EXECUTABLE}"
  --headless
  --frames "${_frames}"
  --width "${_width}"
  --height "${_height}"
  --profile
  --screenshot "${DART_GUI_FILAMENT_SCREENSHOT}"
)

if(DEFINED DART_GUI_FILAMENT_SCENE)
  list(APPEND _command --scene "${DART_GUI_FILAMENT_SCENE}")
endif()

get_filename_component(_screenshot_dir "${DART_GUI_FILAMENT_SCREENSHOT}" DIRECTORY)
file(MAKE_DIRECTORY "${_screenshot_dir}")
file(REMOVE "${DART_GUI_FILAMENT_SCREENSHOT}")

execute_process(
  COMMAND ${_command}
  RESULT_VARIABLE _result
  OUTPUT_VARIABLE _stdout
  ERROR_VARIABLE _stderr
)

if(NOT _stdout STREQUAL "")
  message(STATUS "dartsim stdout:\n${_stdout}")
endif()

if(NOT _stderr STREQUAL "")
  message(STATUS "dartsim stderr:\n${_stderr}")
endif()

if(NOT _result STREQUAL "0")
  message(FATAL_ERROR "dartsim headless smoke failed with exit code ${_result}")
endif()

if(NOT EXISTS "${DART_GUI_FILAMENT_SCREENSHOT}")
  message(FATAL_ERROR "dartsim did not write ${DART_GUI_FILAMENT_SCREENSHOT}")
endif()

file(SIZE "${DART_GUI_FILAMENT_SCREENSHOT}" _screenshot_size)
set(_expected_header "P6\n${_width} ${_height}\n255\n")
string(LENGTH "${_expected_header}" _header_size)
file(READ "${DART_GUI_FILAMENT_SCREENSHOT}" _actual_header LIMIT "${_header_size}")

if(NOT "${_actual_header}" STREQUAL "${_expected_header}")
  message(
    FATAL_ERROR
    "dartsim screenshot does not start with the expected ${_width}x${_height} binary PPM header"
  )
endif()

math(EXPR _expected_size "${_header_size} + ${_width} * ${_height} * 3")

if(_screenshot_size LESS _expected_size)
  message(
    FATAL_ERROR
    "dartsim screenshot is ${_screenshot_size} bytes, expected at least ${_expected_size}"
  )
endif()

file(
  READ "${DART_GUI_FILAMENT_SCREENSHOT}" _pixel_sample
  OFFSET "${_header_size}"
  LIMIT 8192
  HEX
)

if(NOT _pixel_sample MATCHES "[1-9a-fA-F]")
  message(FATAL_ERROR "dartsim screenshot sample contains only zero-valued pixels")
endif()

if(DEFINED DART_GUI_FILAMENT_PYTHON AND DEFINED DART_GUI_FILAMENT_ANALYZER)
  set(_analysis_mode contrast)
  if(DEFINED DART_GUI_FILAMENT_ANALYSIS_MODE)
    set(_analysis_mode "${DART_GUI_FILAMENT_ANALYSIS_MODE}")
  endif()
  if(NOT "${_analysis_mode}" MATCHES "^(basic|contrast)$")
    message(
      FATAL_ERROR
      "DART_GUI_FILAMENT_ANALYSIS_MODE must be 'basic' or 'contrast'"
    )
  endif()

  execute_process(
    COMMAND
      "${DART_GUI_FILAMENT_PYTHON}"
      "${DART_GUI_FILAMENT_ANALYZER}"
      "${DART_GUI_FILAMENT_SCREENSHOT}"
      --width "${_width}"
      --height "${_height}"
      --mode "${_analysis_mode}"
    RESULT_VARIABLE _analyzer_result
    OUTPUT_VARIABLE _analyzer_stdout
    ERROR_VARIABLE _analyzer_stderr
  )

  if(NOT _analyzer_stdout STREQUAL "")
    message(STATUS "dartsim screenshot analysis:\n${_analyzer_stdout}")
  endif()

  if(NOT _analyzer_stderr STREQUAL "")
    message(STATUS "dartsim screenshot analysis stderr:\n${_analyzer_stderr}")
  endif()

  if(NOT _analyzer_result STREQUAL "0")
    message(FATAL_ERROR "dartsim screenshot analysis failed")
  endif()
endif()

message(
  STATUS
  "dartsim headless smoke wrote ${DART_GUI_FILAMENT_SCREENSHOT} (${_screenshot_size} bytes)"
)
