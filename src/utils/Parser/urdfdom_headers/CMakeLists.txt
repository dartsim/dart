# urdfdom_headers
project( urdfdom_headers )
message( STATUS "Creating Makefile for urdfdom_headers" )

# Search all header and source files
file( GLOB urdf_exception_hdrs urdf_exception/exception.h  )
file( GLOB urdf_model_hdrs  urdf_model/color.h urdf_model/joint.h urdf_model/link.h urdf_model/model.h urdf_model/pose.h urdf_model/twist.h )
file( GLOB urdf_model_state_hdrs  urdf_model_state/model_state.h )
file( GLOB urdf_sensor_hdrs urdf_sensor/sensor.h )
file( GLOB urdf_world_hdrs urdf_world/world.h )

# Add Headers
  install (FILES ${urdf_exception_hdrs} DESTINATION include/dart/utils/Parser/urdfdom_headers/urdf_exception )
 
  install (FILES ${urdf_model_hdrs} DESTINATION include/dart/utils/Parser/urdfdom_headers/urdf_model )

  install (FILES ${urdf_model_state_hdrs} DESTINATION include/dart/utils/Parser/urdfdom_headers/urdf_model_state )

  install (FILES ${urdf_sensor_hdrs} DESTINATION include/dart/utils/Parser/urdfdom_headers/urdf_sensor )

  install (FILES ${urdf_world_hdrs} DESTINATION include/dart/utils/Parser/urdfdom_headers/urdf_world )
