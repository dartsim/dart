#===============================================================================
# Usage:
#   dartpy_add_example(example_name) # assumed source is example_name.py
#   dartpy_add_example(example_name main.py)
#===============================================================================
function(dartpy_add_example example_name) # ARGN for source file
  if(${ARGC} GREATER 1)
    set(source ${ARGN})
  else()
    set(source "${example_name}.py")
  endif()
  add_custom_target(${example_name}
    COMMAND ${CMAKE_COMMAND} -E echo "Running pytest by: PYTHONPATH=${DART_DARTPY_BUILD_DIR} ${PYTHON_EXECUTABLE} ${source}"
    COMMAND PYTHONPATH=${DART_DARTPY_BUILD_DIR} ${PYTHON_EXECUTABLE} ${source}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    SOURCES ${source}
  )
endfunction()

add_subdirectory(hello_world)
add_subdirectory(hello_world_gui)
