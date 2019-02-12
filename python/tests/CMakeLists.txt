# Make sure pytest is installed
execute_process(
  COMMAND ${PYTHON_EXECUTABLE} -c "import pytest; print(pytest.__version__)"
  RESULT_VARIABLE pytest_not_found
  OUTPUT_VARIABLE pytest_version
  ERROR_QUIET
)
if(pytest_not_found)
  message(STATUS "Running the tests requires pytest. Please install it manually"
    " (try: ${PYTHON_EXECUTABLE} -m pip install pytest)"
  )
  set(DARTPY_PYTEST_FOUND FALSE)
elseif(pytest_version VERSION_LESS 3.0)
  message(STATUS "Running the tests requires pytest >= 3.0. Found: ${pytest_version}"
    "Please update it (try: ${PYTHON_EXECUTABLE} -m pip install -U pytest)"
  )
  set(DARTPY_PYTEST_FOUND FALSE)
else()
  set(DARTPY_PYTEST_FOUND TRUE)
endif()

set(dartpy_test_utils
  util.py
)

set(dartpy_test_files
  unit/common/test_uri.py
  # unit/math/test_random.py
  unit/simulation/test_world.py
  unit/utils/test_dart_loader.py
)

# Add custom target to run the tests
add_custom_target(pytest
  COMMAND ${CMAKE_COMMAND} -E echo "Running pytest by: PYTHONPATH=${DART_DARTPY_BUILD_DIR} ${PYTHON_EXECUTABLE} -m pytest [sources]"
  COMMAND PYTHONPATH=${DART_DARTPY_BUILD_DIR} ${PYTHON_EXECUTABLE} -m pytest ${dartpy_test_files} -v
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  SOURCES ${dartpy_test_files} ${dartpy_test_utils}
)
