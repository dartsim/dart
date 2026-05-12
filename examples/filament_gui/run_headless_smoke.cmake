if(NOT DEFINED DART_FILAMENT_GUI_EXECUTABLE)
  message(FATAL_ERROR "DART_FILAMENT_GUI_EXECUTABLE is required")
endif()

if(NOT DEFINED DART_FILAMENT_GUI_SCREENSHOT)
  message(FATAL_ERROR "DART_FILAMENT_GUI_SCREENSHOT is required")
endif()

set(_width 640)
if(DEFINED DART_FILAMENT_GUI_WIDTH)
  set(_width "${DART_FILAMENT_GUI_WIDTH}")
endif()

set(_height 480)
if(DEFINED DART_FILAMENT_GUI_HEIGHT)
  set(_height "${DART_FILAMENT_GUI_HEIGHT}")
endif()

set(_frames 10)
if(DEFINED DART_FILAMENT_GUI_FRAMES)
  set(_frames "${DART_FILAMENT_GUI_FRAMES}")
endif()

if(NOT "${_width}" MATCHES "^[1-9][0-9]*$")
  message(FATAL_ERROR "DART_FILAMENT_GUI_WIDTH must be a positive integer")
endif()

if(NOT "${_height}" MATCHES "^[1-9][0-9]*$")
  message(FATAL_ERROR "DART_FILAMENT_GUI_HEIGHT must be a positive integer")
endif()

if(NOT "${_frames}" MATCHES "^[1-9][0-9]*$")
  message(FATAL_ERROR "DART_FILAMENT_GUI_FRAMES must be a positive integer")
endif()

set(_command
  "${DART_FILAMENT_GUI_EXECUTABLE}"
  --headless
  --frames "${_frames}"
  --width "${_width}"
  --height "${_height}"
  --screenshot "${DART_FILAMENT_GUI_SCREENSHOT}"
)

if(DEFINED DART_FILAMENT_GUI_SCENE)
  list(APPEND _command --scene "${DART_FILAMENT_GUI_SCENE}")
endif()

get_filename_component(_screenshot_dir "${DART_FILAMENT_GUI_SCREENSHOT}" DIRECTORY)
file(MAKE_DIRECTORY "${_screenshot_dir}")
file(REMOVE "${DART_FILAMENT_GUI_SCREENSHOT}")

execute_process(
  COMMAND ${_command}
  RESULT_VARIABLE _result
  OUTPUT_VARIABLE _stdout
  ERROR_VARIABLE _stderr
)

if(NOT _stdout STREQUAL "")
  message(STATUS "filament_gui stdout:\n${_stdout}")
endif()

if(NOT _stderr STREQUAL "")
  message(STATUS "filament_gui stderr:\n${_stderr}")
endif()

if(NOT _result STREQUAL "0")
  message(FATAL_ERROR "filament_gui headless smoke failed with exit code ${_result}")
endif()

if(NOT EXISTS "${DART_FILAMENT_GUI_SCREENSHOT}")
  message(FATAL_ERROR "filament_gui did not write ${DART_FILAMENT_GUI_SCREENSHOT}")
endif()

file(SIZE "${DART_FILAMENT_GUI_SCREENSHOT}" _screenshot_size)
set(_expected_header "P6\n${_width} ${_height}\n255\n")
string(LENGTH "${_expected_header}" _header_size)
file(READ "${DART_FILAMENT_GUI_SCREENSHOT}" _actual_header LIMIT "${_header_size}")

if(NOT "${_actual_header}" STREQUAL "${_expected_header}")
  message(
    FATAL_ERROR
    "filament_gui screenshot does not start with the expected ${_width}x${_height} binary PPM header"
  )
endif()

math(EXPR _expected_size "${_header_size} + ${_width} * ${_height} * 3")

if(_screenshot_size LESS _expected_size)
  message(
    FATAL_ERROR
    "filament_gui screenshot is ${_screenshot_size} bytes, expected at least ${_expected_size}"
  )
endif()

file(
  READ "${DART_FILAMENT_GUI_SCREENSHOT}" _pixel_sample
  OFFSET "${_header_size}"
  LIMIT 8192
  HEX
)

if(NOT _pixel_sample MATCHES "[1-9a-fA-F]")
  message(FATAL_ERROR "filament_gui screenshot sample contains only zero-valued pixels")
endif()

if(DEFINED DART_FILAMENT_GUI_PYTHON AND DEFINED DART_FILAMENT_GUI_ANALYZER)
  execute_process(
    COMMAND
      "${DART_FILAMENT_GUI_PYTHON}"
      "${DART_FILAMENT_GUI_ANALYZER}"
      "${DART_FILAMENT_GUI_SCREENSHOT}"
      --width "${_width}"
      --height "${_height}"
    RESULT_VARIABLE _analyzer_result
    OUTPUT_VARIABLE _analyzer_stdout
    ERROR_VARIABLE _analyzer_stderr
  )

  if(NOT _analyzer_stdout STREQUAL "")
    message(STATUS "filament_gui screenshot analysis:\n${_analyzer_stdout}")
  endif()

  if(NOT _analyzer_stderr STREQUAL "")
    message(STATUS "filament_gui screenshot analysis stderr:\n${_analyzer_stderr}")
  endif()

  if(NOT _analyzer_result STREQUAL "0")
    message(FATAL_ERROR "filament_gui screenshot analysis failed")
  endif()
endif()

message(
  STATUS
  "filament_gui headless smoke wrote ${DART_FILAMENT_GUI_SCREENSHOT} (${_screenshot_size} bytes)"
)
