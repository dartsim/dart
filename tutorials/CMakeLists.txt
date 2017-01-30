# A list of tutorials
set_property(DIRECTORY PROPERTY FOLDER Tutorials)

# Automatically identify all directories in the tutorials folder
file(GLOB children RELATIVE ${CMAKE_CURRENT_LIST_DIR} "*")
set(directories "")
foreach(child ${children})
  if(IS_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/${child}")

      file(GLOB srcs "${CMAKE_CURRENT_LIST_DIR}/${child}/*.cpp")

      if(srcs)
        add_executable(${child} ${srcs})
        target_link_libraries(${child} dart dart-utils-urdf dart-gui)
        dart_add_tutorial(${child})
      else()
        add_subdirectory(${child})
      endif()

  endif()
endforeach()
