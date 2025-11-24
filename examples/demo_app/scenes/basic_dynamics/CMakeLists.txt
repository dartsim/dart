cmake_minimum_required(VERSION 3.22.1)

set(scene_target demo_scene_basic_dynamics)

add_library(${scene_target} STATIC
  BasicDynamicsScene.cpp
  BasicDynamicsScene.hpp
)

target_link_libraries(${scene_target} PUBLIC demo_app_core)
dart_format_add(BasicDynamicsScene.cpp BasicDynamicsScene.hpp)

set(DEMO_APP_SCENES ${DEMO_APP_SCENES} ${scene_target} PARENT_SCOPE)
