#===============================================================================
# Usage:
#   dartpy_add_tutorial(tutorial_name) # assumed source is tutorial_name.py
#   dartpy_add_tutorial(tutorial_name main.py)
#===============================================================================
function(dartpy_add_tutorial tutorial_name) # ARGN for source file
  if(${ARGC} GREATER 1)
    set(source ${ARGN})
  else()
    set(source "${tutorial_name}.py")
  endif()
  add_custom_target(${tutorial_name}
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
    set(tutorial_main "${CMAKE_CURRENT_LIST_DIR}/${child}/main.py")
    set(tutorial_main_finished
      "${CMAKE_CURRENT_LIST_DIR}/${child}/main_finished.py")
    if(EXISTS "${tutorial_main}")
      if(DART_VERBOSE)
        message(STATUS "Adding Python tutorial: " py_${child})
      endif()
      dartpy_add_tutorial(py_tutorial_${child} "${tutorial_main}")
    endif()
    if(EXISTS "${tutorial_main_finished}")
      dartpy_add_tutorial(py_tutorial_${child}_finished
        "${tutorial_main_finished}")
    endif()
  endif()
endforeach()
