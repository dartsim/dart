# Check if OpenSceneGraph is available on the build machine
find_package(OpenSceneGraph 3.0 QUIET
  COMPONENTS osg osgViewer osgManipulator osgGA osgDB)

if(${OPENSCENEGRAPH_FOUND})

  message(STATUS "Found OpenSceneGraph -- Building osgDart")

  include_directories(${OpenSceneGraph_INCLUDE_DIRS})

  file(GLOB osgDart_srcs "*.cpp" "render/*.cpp" "render/*.h")
  file(GLOB osgDart_hdrs "*.h")

  kido_add_library(osgDart ${osgDart_srcs} ${osgDart_hdrs})
  target_link_libraries(osgDart
    kido-core
    ${OPENSCENEGRAPH_LIBRARIES}
  )

  kido_get_filename_components(header_names "osgDart headers" ${osgDart_hdrs})
  list(APPEND header_names "render/render.h")
  kido_generate_include_header_list(
    osgDart_headers
    "osgDart/"
    "osgDart headers"
    ${header_names}
  )
  configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/osgDart.h.in
    ${CMAKE_CURRENT_BINARY_DIR}/osgDart.h
  )

  add_subdirectory(render)

  if(${KIDO_BUILD_EXAMPLES})
    add_subdirectory(examples)
  endif(${KIDO_BUILD_EXAMPLES})

  install(
    FILES ${osgDart_hdrs} ${CMAKE_CURRENT_BINARY_DIR}/osgDart.h
    DESTINATION include/osgDart
    COMPONENT headers
  )

  install(TARGETS osgDart EXPORT osgDartTargets DESTINATION lib)

else(${OPENSCENEGRAPH_FOUND})

  # osgDart requires both OSG and OpenThreads. This section attempts to identify
  # which of those are missing from the building machine and offer advice to the
  # user for getting osgDart to build.

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

  message(WARNING "${warning_msg} -- we will skip osgDart\n"
          "If you believe you do have both OSG and OpenThreads installed, try setting OSG_DIR")

endif(${OPENSCENEGRAPH_FOUND})

