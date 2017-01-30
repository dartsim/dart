# This directory depends on OpenSceneGraph
if(HAVE_OPENSCENEGRAPH)

  # Automatically identify all directories in the examples folder
  file(GLOB children RELATIVE ${CMAKE_CURRENT_LIST_DIR} "*")
  set(directories "")
  foreach(child ${children})
    if(IS_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/${child}")

      file(GLOB srcs "${CMAKE_CURRENT_LIST_DIR}/${child}/*.cpp")

      if(srcs)
        add_executable(${child} ${srcs})
        target_link_libraries(${child} dart dart-utils-urdf dart-gui-osg)
        dart_add_example(${child})
      else()
        add_subdirectory(${child})
      endif()

    endif()
  endforeach()

endif(HAVE_OPENSCENEGRAPH)
