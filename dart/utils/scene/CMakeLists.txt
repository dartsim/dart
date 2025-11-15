set(scene_headers "${CMAKE_CURRENT_SOURCE_DIR}/SceneParser.hpp")
set(scene_sources "${CMAKE_CURRENT_SOURCE_DIR}/SceneParser.cpp")

dart_add_utils_headers(${scene_headers})

set(target_name ${PROJECT_NAME}-utils-scene)
set(component_name utils-scene)

dart_add_library(${target_name} ${scene_headers} ${scene_sources})
target_link_libraries(${target_name} PUBLIC dart-utils)

if(TARGET ${PROJECT_NAME}-utils-urdf)
  target_link_libraries(${target_name} PUBLIC ${PROJECT_NAME}-utils-urdf)
  target_compile_definitions(${target_name} PUBLIC DART_SCENEPARSER_HAS_URDF=1)
endif()

add_component(${PROJECT_NAME} ${component_name})
add_component_targets(${PROJECT_NAME} ${component_name} ${target_name})

install(
  FILES ${scene_headers}
  DESTINATION include/dart/utils/scene
  COMPONENT headers
)

dart_format_add(${scene_headers} ${scene_sources})
