# A list of examples
set_property(DIRECTORY PROPERTY FOLDER Examples)

# Automatically identify all directories in the examples folder
file(GLOB children RELATIVE ${CMAKE_CURRENT_LIST_DIR} "*")
set(directories "")
foreach(child ${children})
  if(IS_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/${child}")
    if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/${child}/CMakeLists.txt")
      list(APPEND directories ${child})
    endif(EXISTS "${CMAKE_CURRENT_LIST_DIR}/${child}/CMakeLists.txt")
  endif(IS_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/${child}")
endforeach(child)

# List of all the subdirectories to include
foreach(EXAMPLE_DIR ${directories})
  add_subdirectory(${EXAMPLE_DIR})
endforeach(EXAMPLE_DIR)
