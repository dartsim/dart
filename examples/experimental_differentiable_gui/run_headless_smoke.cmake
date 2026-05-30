if(NOT DEFINED DART_DIFFERENTIABLE_GUI_EXECUTABLE)
  message(FATAL_ERROR "DART_DIFFERENTIABLE_GUI_EXECUTABLE is required")
endif()

if(NOT DEFINED DART_DIFFERENTIABLE_GUI_OUTPUT_DIR)
  message(FATAL_ERROR "DART_DIFFERENTIABLE_GUI_OUTPUT_DIR is required")
endif()

set(_width 640)
set(_height 480)
# Render well into the converged rollout so the animated projectile has moved
# along the optimized arc.
set(_frames 30)

file(MAKE_DIRECTORY "${DART_DIFFERENTIABLE_GUI_OUTPUT_DIR}")
set(_screenshot "${DART_DIFFERENTIABLE_GUI_OUTPUT_DIR}/throw_to_target.ppm")
file(REMOVE "${_screenshot}")

execute_process(
  COMMAND
    "${DART_DIFFERENTIABLE_GUI_EXECUTABLE}" --headless --frames "${_frames}"
    --width "${_width}" --height "${_height}" --screenshot "${_screenshot}"
  RESULT_VARIABLE _result
  OUTPUT_VARIABLE _stdout
  ERROR_VARIABLE _stderr
)

if(NOT _stdout STREQUAL "")
  message(STATUS "experimental_differentiable_gui stdout:\n${_stdout}")
endif()
if(NOT _stderr STREQUAL "")
  message(STATUS "experimental_differentiable_gui stderr:\n${_stderr}")
endif()
if(NOT _result STREQUAL "0")
  message(
    FATAL_ERROR
    "experimental_differentiable_gui headless smoke failed with exit code ${_result}"
  )
endif()

if(NOT EXISTS "${_screenshot}")
  message(
    FATAL_ERROR
    "experimental_differentiable_gui did not write ${_screenshot}"
  )
endif()

file(SIZE "${_screenshot}" _screenshot_size)
set(_expected_header "P6\n${_width} ${_height}\n255\n")
string(LENGTH "${_expected_header}" _header_size)
file(READ "${_screenshot}" _actual_header LIMIT "${_header_size}")
if(NOT "${_actual_header}" STREQUAL "${_expected_header}")
  message(
    FATAL_ERROR
    "${_screenshot} does not start with the expected ${_width}x${_height} binary PPM header"
  )
endif()

math(EXPR _expected_size "${_header_size} + ${_width} * ${_height} * 3")
if(_screenshot_size LESS _expected_size)
  message(
    FATAL_ERROR
    "${_screenshot} is ${_screenshot_size} bytes, expected at least ${_expected_size}"
  )
endif()

file(
  READ "${_screenshot}"
  _pixel_sample
  OFFSET "${_header_size}"
  LIMIT 8192
  HEX
)
if(NOT _pixel_sample MATCHES "[1-9a-fA-F]")
  message(FATAL_ERROR "${_screenshot} sample contains only zero-valued pixels")
endif()

message(
  STATUS
  "experimental_differentiable_gui headless smoke wrote ${_screenshot} (${_screenshot_size} bytes, non-blank)"
)
