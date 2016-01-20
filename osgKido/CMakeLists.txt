# Check if OpenSceneGraph is available on the build machine
find_package(OpenSceneGraph 3.0 QUIET
  COMPONENTS osg osgViewer osgManipulator osgGA osgDB)

if(${OPENSCENEGRAPH_FOUND})

  message(STATUS "Found OpenSceneGraph ${OPENSCENEGRAPH_VERSION} -- Building osgKido")

  include_directories(SYSTEM ${OPENSCENEGRAPH_INCLUDE_DIRS})

  file(GLOB srcs "*.cpp" "render/*.cpp" "render/*.hpp")
  file(GLOB hdrs "*.hpp")

  kido_add_library(kido-gui-osg ${srcs} ${hdrs})
  target_link_libraries(kido-gui-osg kido kido-utils ${OPENSCENEGRAPH_LIBRARIES})

  kido_get_filename_components(header_names "gui-osg headers" ${hdrs})
  list(APPEND header_names "render/render.hpp")
  kido_generate_include_header_list(
    gui_osg_headers
    "osgKido/"
    "gui-osg headers"
    ${header_names}
  )
  configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/osgKido.hpp.in
    ${CMAKE_CURRENT_BINARY_DIR}/osgKido.hpp
  )

  add_subdirectory(render)

  if(${KIDO_BUILD_EXAMPLES})
    add_subdirectory(examples)
  endif(${KIDO_BUILD_EXAMPLES})

  install(
    FILES ${osgKido_hdrs} ${CMAKE_CURRENT_BINARY_DIR}/osgKido.hpp
    DESTINATION include/osgKido
    COMPONENT headers
  )

  install(TARGETS kido-gui-osg EXPORT KIDOTargets DESTINATION lib)

else(${OPENSCENEGRAPH_FOUND})

  # osgKido requires both OSG and OpenThreads. This section attempts to identify
  # which of those are missing from the building machine and offer advice to the
  # user for getting osgKido to build.

  find_package(OpenThreads QUIET)
  if(${OPENTHREADS_FOUND})

    set(warning_msg "Could NOT find OpenSceneGraph")

  else(${OPENTHREADS_FOUND})

    if(OSG_LIBRARY)
      set(warning_msg "Could NOT find OpenThreads")
    else(OSG_LIBRARY)
      set(warning_msg "Could NOT find OpenSceneGraph nor OpenThreads")
    endif(OSG_LIBRARY)

  endif(${OPENTHREADS_FOUND})

  message(WARNING "${warning_msg} -- we will skip osgKido\n"
          "If you believe you do have both OSG and OpenThreads installed, try setting OSG_DIR")

endif(${OPENSCENEGRAPH_FOUND})

