# urdf_parser
project( urdf_parser )

message( STATUS "Creating urdf parser" )

# Boost
find_package(Boost REQUIRED system)
include_directories(${Boost_INCLUDE_DIR})

include_directories ( . 
		../urdfdom_headers 
		.. )

# Search all header and source files
file( GLOB urdfdom_model_srcs pose.cpp model.cpp link.cpp joint.cpp world.cpp )
file( GLOB urdfdom_model_hdrs urdf_parser.h )
file( GLOB urdfdom_sensor_srcs urdf_sensor.cpp )
file( GLOB urdf_model_state_srcs urdf_model_state.cpp twist.cpp )

# Add Libraries

#-- urdfdom_model
add_library(urdfdom_model SHARED ${urdfdom_model_srcs} ${urdfdom_model_hdrs} )
target_link_libraries(urdfdom_model tinyxml ${Boost_LIBRARIES} )

set_target_properties (urdfdom_model PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib")
set_target_properties (urdfdom_model PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib")
  install (FILES ${urdfdom_model_hdrs} DESTINATION include/dart/Parser/urdf_parser )
  install (TARGETS urdfdom_model  DESTINATION lib)

#-- urdfdom_sensor
add_library(urdfdom_sensor SHARED ${urdfdom_sensor_srcs})
target_link_libraries(urdfdom_sensor ${Boost_LIBRARIES})

set_target_properties (urdfdom_sensor  PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib")
set_target_properties (urdfdom_sensor PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib")
  install (TARGETS urdfdom_sensor  DESTINATION lib)

#-- urdfdom_model_state
add_library(urdfdom_model_state SHARED ${urdf_model_state_srcs} )
target_link_libraries(urdfdom_model_state  ${Boost_LIBRARIES})

set_target_properties (urdfdom_model_state PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib")
set_target_properties (urdfdom_model_state PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib")
  install (FILES ${urdf_model_state_srcs} DESTINATION include/dart/Parser/urdf_parser )
  install (TARGETS urdfdom_model_state  DESTINATION lib)

