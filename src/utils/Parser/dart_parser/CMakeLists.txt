# DART Parser
project( parser )
message( STATUS "Creating Makefile for DART Parser" )

include_directories( . 
		../urdfdom_headers 
		..)

# Search all header and source files
file( GLOB parser_srcs DartLoader.cpp LoaderUtils.cpp ToDartUtils.cpp )
file( GLOB parser_hdrs DartLoader.h )

# Add Library
add_library( parser ${parser_srcs} ${parser_hdrs} )
target_link_libraries( parser  ${DART_LIBRARIES} )
target_link_libraries( parser urdfdom_model urdfdom_model_state urdfdom_sensor tinyxmlstatic robotics )

 set_target_properties (parser PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib")
 set_target_properties (parser PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib")
IF(MSVC)
  SET_TARGET_PROPERTIES(parser PROPERTIES STATIC_LIBRARY_FLAGS_RELEASE "/LTCG")
ENDIF()
  install (FILES ${parser_hdrs} DESTINATION include/dart/utils/Parser/dartParser )
  install (TARGETS parser ARCHIVE DESTINATION lib)

