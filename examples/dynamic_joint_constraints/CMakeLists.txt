cmake_minimum_required(VERSION 3.22.1)

get_filename_component(example_name ${CMAKE_CURRENT_LIST_DIR} NAME)

project(${example_name})

set(required_components gui-osg)
set(required_libraries dart dart-gui-osg)

if(DART_IN_SOURCE_BUILD AND NOT TARGET dart-gui-osg)
  return()
endif()

find_package(OpenSceneGraph 3.0.0 REQUIRED COMPONENTS osgText)
set(osg_text_library ${OSGTEXT_LIBRARY})
if(NOT osg_text_library)
  set(osg_text_library ${OSGTEXT_LIBRARY_RELEASE})
endif()
if(NOT osg_text_library)
  message(FATAL_ERROR "Could NOT find osgText")
endif()
if(NOT TARGET osgText::osgText)
  add_library(osgText::osgText UNKNOWN IMPORTED)
  set_target_properties(osgText::osgText
    PROPERTIES
      IMPORTED_LOCATION "${osg_text_library}"
      INTERFACE_INCLUDE_DIRECTORIES "${OSGTEXT_INCLUDE_DIR}"
  )
endif()
list(APPEND required_libraries osgText::osgText)

if(DART_IN_SOURCE_BUILD)
  dart_build_example_in_source(${example_name} LINK_LIBRARIES ${required_libraries})
  return()
endif()

find_package(DART 6.19 REQUIRED COMPONENTS ${required_components} CONFIG)

file(GLOB srcs "*.cpp" "*.hpp")
add_executable(${example_name} ${srcs})
target_link_libraries(${example_name} PUBLIC ${required_libraries})
