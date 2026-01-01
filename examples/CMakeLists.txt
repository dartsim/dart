cmake_minimum_required(VERSION 3.22.1)

project(dart-examples)

# Getting started examples
add_subdirectory(00_getting_started/hello_world)
add_subdirectory(00_getting_started/simple_frames)

# Rigid bodies and frames examples
add_subdirectory(01_rigid_bodies_and_frames/boxes)
add_subdirectory(01_rigid_bodies_and_frames/rigid_cubes)

# Joints and constraints examples
add_subdirectory(02_joints_and_constraints/coupler_constraint)
add_subdirectory(02_joints_and_constraints/free_joint_cases)
add_subdirectory(02_joints_and_constraints/human_joint_limits)
add_subdirectory(02_joints_and_constraints/joint_constraints)
add_subdirectory(02_joints_and_constraints/mimic_pendulums)
add_subdirectory(02_joints_and_constraints/rigid_chain)
add_subdirectory(02_joints_and_constraints/rigid_loop)

# Collisions and contacts examples
add_subdirectory(03_collisions_and_contacts/box_stacking)
add_subdirectory(03_collisions_and_contacts/capsule_ground_contact)
add_subdirectory(03_collisions_and_contacts/heightmap)
add_subdirectory(03_collisions_and_contacts/rigid_shapes)

# Control and IK examples
add_subdirectory(04_control_and_ik/atlas_puppet)
add_subdirectory(04_control_and_ik/atlas_simbicon)
add_subdirectory(04_control_and_ik/biped_stand)
add_subdirectory(04_control_and_ik/hubo_puppet)
add_subdirectory(04_control_and_ik/operational_space_control)
add_subdirectory(04_control_and_ik/wam_ikfast)

# None GUI examples
add_subdirectory(speed_test)

# OSG renderer examples
add_subdirectory(add_delete_skels)
add_subdirectory(drag_and_drop)
add_subdirectory(empty)
add_subdirectory(hardcoded_design)
add_subdirectory(hybrid_dynamics)
add_subdirectory(imgui)
add_subdirectory(mixed_chain)
add_subdirectory(polyhedron_visual)
if(DART_HAVE_OCTOMAP)
  add_subdirectory(point_cloud)
endif()
add_subdirectory(simulation_event_handler)
add_subdirectory(soft_bodies)
add_subdirectory(g1_puppet)
add_subdirectory(tinkertoy)
add_subdirectory(vehicle)

# MJCF model based examples
add_subdirectory(fetch)
