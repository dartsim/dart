cmake_minimum_required(VERSION 3.22.1)

project(dart-demos)

include("${CMAKE_CURRENT_LIST_DIR}/../gui_scene_example.cmake")

# Absolute source paths so the formatter and add_executable resolve regardless
# of how dart_build_gui_example resolves relative paths.
set(dart_demos_sources
  "${CMAKE_CURRENT_LIST_DIR}/app/main.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/registry.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/hello_world.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/empty.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/shapes.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/simple_frames.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/drag_and_drop.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/polyhedron_visual.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/boxes.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/rigid_cubes.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/add_delete_skels.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/capsule_ground_contact.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/hardcoded_design.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/box_stacking.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/hybrid_dynamics.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/vehicle.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/imgui.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/simulation_event_handler.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/mixed_chain.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/coupler_constraint.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/mimic_pendulums.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/rigid_chain.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/rigid_loop.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/free_joint_cases.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/joint_constraints.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/soft_bodies.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/rigid_shapes.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/heightmap.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/lcp_physics.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/biped_stand.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/tinkertoy.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/operational_space_control.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/point_cloud.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/fetch.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/atlas_puppet.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/hubo_puppet.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/g1_puppet.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/human_joint_limits.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/scenes/experimental_rigid_body_gui.cpp"
)

dart_build_gui_example(dart-demos ${dart_demos_sources})

if(TARGET dart-demos)
  target_include_directories(dart-demos PRIVATE "${CMAKE_CURRENT_LIST_DIR}")
  # Scenes load worlds/skeletons via dart::io (URDF/SDF/SKEL). Collision
  # backends (ODE/Bullet) are selected through runtime factories, so they need
  # no extra link here.
  target_link_libraries(dart-demos PRIVATE dart-io)
  # The experimental rigid-body scene drives the experimental World.
  if(TARGET dart-simulation-experimental)
    target_link_libraries(dart-demos PRIVATE dart-simulation-experimental)
  endif()
endif()
