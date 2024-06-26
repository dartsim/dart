# Make sure pytest is installed
execute_process(
  COMMAND ${Python3_EXECUTABLE} -c "import pytest; print(pytest.__version__)"
  RESULT_VARIABLE pytest_not_found
  OUTPUT_VARIABLE pytest_version
  ERROR_QUIET
)
if(pytest_not_found)
  message(WARNING "Running the tests requires pytest. Please install it manually"
    " (try: ${Python3_EXECUTABLE} -m pip install pytest)"
  )
  set(DARTPY_PYTEST_FOUND FALSE)
elseif(pytest_version VERSION_LESS 3.0)
  message(WARNING "Running the tests requires pytest >= 3.0. Found: ${pytest_version}"
    "Please update it (try: ${Python3_EXECUTABLE} -m pip install -U pytest)"
  )
  set(DARTPY_PYTEST_FOUND FALSE)
else()
  set(DARTPY_PYTEST_FOUND TRUE)
endif()
set(DARTPY_PYTEST_FOUND ${DARTPY_PYTEST_FOUND} PARENT_SCOPE)

set(dartpy_test_utils
  util.py
)

file(GLOB_RECURSE dartpy_test_files "test_*.py")

# Add custom target to run the tests
if(DARTPY_PYTEST_FOUND)
  add_custom_target(pytest
    COMMAND ${CMAKE_COMMAND} -E echo "Running pytest by: PYTHONPATH=${DART_DARTPY_BUILD_DIR} ${Python3_EXECUTABLE} -m pytest [sources]"
    COMMAND ${CMAKE_COMMAND} -E env PYTHONPATH="${DART_DARTPY_BUILD_DIR}" "${Python3_EXECUTABLE}" -m pytest ${dartpy_test_files} -v
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    SOURCES ${dartpy_test_files} ${dartpy_test_utils}
    DEPENDS dartpy
  )
else()
  add_custom_target(pytest
    COMMAND ${CMAKE_COMMAND} -E echo "Warning: Failed to run pytest because pytest is not found!"
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
  add_custom_target(${test_name}
    COMMAND ${CMAKE_COMMAND} -E echo "Running pytest by: PYTHONPATH=${DART_DARTPY_BUILD_DIR} ${Python3_EXECUTABLE} ${source}"
    COMMAND ${CMAKE_COMMAND} -E env PYTHONPATH=${DART_DARTPY_BUILD_DIR} ${Python3_EXECUTABLE} ${source}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    SOURCES ${source}
    DEPENDS dartpy
  )
endfunction()

foreach(dartpy_test_file ${dartpy_test_files})
  get_filename_component(dartpy_test_name ${dartpy_test_file} NAME_WE)
  dartpy_add_test(py_${dartpy_test_name} ${dartpy_test_file})
endforeach()
