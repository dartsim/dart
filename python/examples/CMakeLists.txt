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
    COMMAND ${CMAKE_COMMAND} -E echo "Running pytest by: PYTHONPATH=${DART_PYTHONPATH} ${Python3_EXECUTABLE} ${source}"
    COMMAND ${CMAKE_COMMAND} -E env "PYTHONPATH=${DART_PYTHONPATH}" ${Python3_EXECUTABLE} ${source}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    SOURCES ${source}
  )
endfunction()

# Automatically identify all directories in the apps folder
file(GLOB children RELATIVE ${CMAKE_CURRENT_LIST_DIR} "*")
foreach(child ${children})
  if(IS_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/${child}")
    set(example_main "${CMAKE_CURRENT_LIST_DIR}/${child}/main.py")
    if(EXISTS "${example_main}")
      if(DART_VERBOSE)
        message(STATUS "Adding Python example: " py_${child})
      endif()
      dartpy_add_example(py_example_${child} "${example_main}")
    endif()
  endif()
endforeach()
