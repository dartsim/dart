cmake_minimum_required(VERSION 3.22.1)

project(dart-examples)

# Getting started examples
add_subdirectory(00_getting_started/hello_world)
add_subdirectory(00_getting_started/simple_frames)

# Rigid bodies and frames examples
add_subdirectory(01_rigid_bodies_and_frames/boxes)
add_subdirectory(01_rigid_bodies_and_frames/rigid_cubes)

# None GUI examples
add_subdirectory(speed_test)

# OSG renderer examples
add_subdirectory(add_delete_skels)
add_subdirectory(atlas_puppet)
add_subdirectory(atlas_simbicon)
add_subdirectory(biped_stand)
add_subdirectory(box_stacking)
add_subdirectory(coupler_constraint)
add_subdirectory(drag_and_drop)
add_subdirectory(empty)
add_subdirectory(mimic_pendulums)
add_subdirectory(hardcoded_design)
add_subdirectory(heightmap)
add_subdirectory(human_joint_limits)
add_subdirectory(hybrid_dynamics)
add_subdirectory(hubo_puppet)
add_subdirectory(imgui)
add_subdirectory(free_joint_cases)
add_subdirectory(joint_constraints)
add_subdirectory(mixed_chain)
add_subdirectory(operational_space_control)
add_subdirectory(polyhedron_visual)
add_subdirectory(capsule_ground_contact)
if(DART_HAVE_OCTOMAP)
  add_subdirectory(point_cloud)
endif()
add_subdirectory(rigid_chain)
add_subdirectory(rigid_loop)
add_subdirectory(rigid_shapes)
add_subdirectory(simulation_event_handler)
add_subdirectory(soft_bodies)
add_subdirectory(g1_puppet)
add_subdirectory(tinkertoy)
add_subdirectory(vehicle)
add_subdirectory(wam_ikfast)

# MJCF model based examples
add_subdirectory(fetch)
