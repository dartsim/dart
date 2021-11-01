# Copyright (c) 2011-2021, The DART development contributors

# Make sure pytest is installed
execute_process(
  COMMAND ${PYTHON_EXECUTABLE} -c "import pytest; print(pytest.__version__)"
  RESULT_VARIABLE pytest_not_found OUTPUT_VARIABLE pytest_version ERROR_QUIET
)
if(pytest_not_found)
  message(WARNING "Skipping test-python because pytest is not found")
  return()
elseif(pytest_version VERSION_LESS 3.0)
  message(
    WARNING
      "Skipping test-python because pytest version ${pytest_version} is less than 3.0"
  )
  return()
endif()

# Collect test files
file(GLOB_RECURSE test_files "py_test_*.py")
list(FILTER test_files EXCLUDE REGEX ".*__init__.py$")

# Add custom target to run the tests
add_custom_target(
  dartpy8-test
  COMMAND
    ${CMAKE_COMMAND} -E echo
    "Running pytest by: PYTHONPATH=${DART_PYTHON_PROJECT_BINARY_DIR}/src ${PYTHON_EXECUTABLE} -m pytest [sources]"
  COMMAND PYTHONPATH=${DART_PYTHON_PROJECT_BINARY_DIR}/src ${PYTHON_EXECUTABLE}
          -m pytest ${test_files} -v
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  SOURCES ${test_files}
  DEPENDS dartpy8
)
