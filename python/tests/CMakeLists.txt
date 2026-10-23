if(BUILD_TESTING)
  set(dartpy_gc_test_abi)
  if(DART_DARTPY_STABLE_ABI)
    set(dartpy_gc_test_abi STABLE_ABI)
  endif()
  nanobind_add_module(
    dartpy_gc_test NB_STATIC PROTECT_STACK ${dartpy_gc_test_abi}
    native/gc_owner.cpp
  )
  set_target_properties(
    dartpy_gc_test
    PROPERTIES
      OUTPUT_NAME _dartpy_gc_test
      DEBUG_POSTFIX ""
      EXCLUDE_FROM_ALL TRUE
      LIBRARY_OUTPUT_DIRECTORY "${DART_DARTPY_BUILD_DIR}"
  )
  target_compile_features(dartpy_gc_test PRIVATE cxx_std_17)
  target_include_directories(
    dartpy_gc_test
    PRIVATE "${PROJECT_SOURCE_DIR}/python/dartpy"
  )
  target_link_libraries(dartpy_gc_test PRIVATE dart)
  dart_format_add(native/gc_owner.cpp)
endif()

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
    "/unit/gui/test_(filament|gui_offscreen|agent_capture|agent_debug_overlay|target_shape_render|viewer_render)\\.py$"
)

# Add custom target to run the tests
if(DARTPY_PYTEST_FOUND)
  add_custom_target(
    pytest
    COMMAND
      ${CMAKE_COMMAND} -E echo
      "Running pytest by: ${Python3_EXECUTABLE} -I ${PROJECT_SOURCE_DIR}/scripts/run_pytest.py --pythonpath ${DART_PYTHONPATH} [sources]"
    COMMAND
      "${Python3_EXECUTABLE}" -I "${PROJECT_SOURCE_DIR}/scripts/run_pytest.py"
      --pythonpath "${DART_PYTHONPATH}" ${dartpy_test_files} -v
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    SOURCES
      ${dartpy_test_files}
      ${dartpy_test_utils}
      "${PROJECT_SOURCE_DIR}/scripts/run_pytest.py"
    DEPENDS dartpy
  )
  if(TARGET dartpy_gc_test)
    add_dependencies(pytest dartpy_gc_test)
  endif()
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

if(TARGET dartpy_gc_test)
  add_dependencies(py_test_gc dartpy_gc_test)
endif()
