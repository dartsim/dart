cmake_minimum_required(VERSION 3.22.1)

set(scene_target demo_scene_controlled_pendulum)

add_library(${scene_target} STATIC
  ControlledPendulumScene.cpp
  ControlledPendulumScene.hpp
)

target_link_libraries(${scene_target} PUBLIC demo_app_core)
dart_format_add(ControlledPendulumScene.cpp ControlledPendulumScene.hpp)

set(DEMO_APP_SCENES ${DEMO_APP_SCENES} ${scene_target} PARENT_SCOPE)
