# Make sure pytest is installed
execute_process(
  COMMAND ${Python3_EXECUTABLE} -c "import pytest; print(pytest.__version__)"
  RESULT_VARIABLE pytest_not_found
  OUTPUT_VARIABLE pytest_version
  ERROR_QUIET
)
if(pytest_not_found)
  message(
    WARNING
    "Running the tests requires pytest. Please install it manually"
    " (try: ${Python3_EXECUTABLE} -m pip install pytest)"
  )
  set(DARTPY_PYTEST_FOUND FALSE)
elseif(pytest_version VERSION_LESS 3.0)
  message(
    WARNING
    "Running the tests requires pytest >= 3.0. Found: ${pytest_version}"
    "Please update it (try: ${Python3_EXECUTABLE} -m pip install -U pytest)"
  )
  set(DARTPY_PYTEST_FOUND FALSE)
else()
  set(DARTPY_PYTEST_FOUND TRUE)
endif()
set(DARTPY_PYTEST_FOUND ${DARTPY_PYTEST_FOUND} PARENT_SCOPE)

set(dartpy_test_utils util.py)

file(GLOB_RECURSE dartpy_test_files CONFIGURE_DEPENDS "test_*.py")
# Rendering needs a graphics device; test-gui runs these modules explicitly.
# Lazy GUI construction, ownership, geometry, and tutorial tests stay here.
list(
  FILTER dartpy_test_files
  EXCLUDE
  REGEX
    "/unit/gui/test_(filament|gui_offscreen|agent_capture|agent_debug_overlay)\\.py$"
)

# Add custom target to run the tests
if(DARTPY_PYTEST_FOUND)
  enable_testing()
  set(
    dartpy_slow_transfer_node
    "python/tests/unit/utils/test_model_resource_retriever.py::test_healthy_slow_transfer_survives_one_minute"
  )
  set(dartpy_pytest_selection)
  set(dartpy_ctest_tests pytest)
  if(DART_BUILD_UTILS_ASSETS)
    # Overlap the mandatory one-minute transfer with the rest of the suite.
    set(dartpy_pytest_selection "--deselect=${dartpy_slow_transfer_node}")
    add_test(
      NAME pytest_slow_transfer
      COMMAND
        "${Python3_EXECUTABLE}" -I "${PROJECT_SOURCE_DIR}/scripts/run_pytest.py"
        --pythonpath "${DART_DARTPY_BUILD_DIR}" --pythonpath
        "${DART_PYTHON_BUILD_DIR}"
        "${PROJECT_SOURCE_DIR}/${dartpy_slow_transfer_node}" --expect-node
        "${dartpy_slow_transfer_node}" -v
    )
    list(APPEND dartpy_ctest_tests pytest_slow_transfer)
  endif()
  add_test(
    NAME pytest
    COMMAND
      "${Python3_EXECUTABLE}" -I "${PROJECT_SOURCE_DIR}/scripts/run_pytest.py"
      --pythonpath "${DART_DARTPY_BUILD_DIR}" --pythonpath
      "${DART_PYTHON_BUILD_DIR}" ${dartpy_test_files} ${dartpy_pytest_selection}
      -v
  )
  set_tests_properties(
    ${dartpy_ctest_tests}
    PROPERTIES LABELS pytest WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
  add_custom_target(
    pytest
    COMMAND
      "${CMAKE_CTEST_COMMAND}" --output-on-failure --no-tests=error --parallel 4
      -C "$<CONFIG>" -L "^pytest$"
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    SOURCES
      ${dartpy_test_files}
      ${dartpy_test_utils}
      "${PROJECT_SOURCE_DIR}/scripts/run_pytest.py"
    DEPENDS dartpy
    VERBATIM
  )
else()
  add_custom_target(
    pytest
    COMMAND
      ${CMAKE_COMMAND} -E echo
      "Warning: Failed to run pytest because pytest is not found!"
  )
endif()

#===============================================================================
# Usage:
#   dartpy_add_test(test_name) # assumed source is test_name.py
#   dartpy_add_test(test_name main.py)
#===============================================================================
function(dartpy_add_test test_name) # ARGN for source file
  if(${ARGC} GREATER 1)
    set(source ${ARGN})
  else()
    set(source "${test_name}.py")
  endif()
  add_custom_target(
    ${test_name}
    COMMAND
      ${CMAKE_COMMAND} -E echo
      "Running pytest by: PYTHONPATH=${DART_PYTHONPATH} ${Python3_EXECUTABLE} ${source}"
    COMMAND
      ${CMAKE_COMMAND} -E env "PYTHONPATH=${DART_PYTHONPATH}"
      ${Python3_EXECUTABLE} ${source}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    SOURCES ${source}
    DEPENDS dartpy
  )
endfunction()

foreach(dartpy_test_file ${dartpy_test_files})
  get_filename_component(dartpy_test_name ${dartpy_test_file} NAME_WE)
  dartpy_add_test(py_${dartpy_test_name} ${dartpy_test_file})
endforeach()
