# A list of applications
set_property(DIRECTORY PROPERTY FOLDER Apps)

# Automatically identify all directories in the apps folder
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
foreach(APPDIR ${directories})
  add_subdirectory(${APPDIR})
  dart_add_example(${APPDIR})
endforeach(APPDIR)
