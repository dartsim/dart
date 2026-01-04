cmake_minimum_required(VERSION 3.22.1)

project(dart-examples)

# Getting started examples
add_subdirectory(hello_world)
add_subdirectory(frame_hierarchy)

# Rigid bodies and frames examples
add_subdirectory(rigid_boxes)
add_subdirectory(rigid_hardcoded_design)
add_subdirectory(rigid_cubes)

# Joints and constraints examples
add_subdirectory(joint_coupler)
add_subdirectory(joint_free_cases)
add_subdirectory(joint_human_limits)
add_subdirectory(joint_constraints)
add_subdirectory(joint_mimic_pendulums)
add_subdirectory(joint_chain)
add_subdirectory(joint_loop)
add_subdirectory(joint_lcp_solvers)

# Collisions and contacts examples
add_subdirectory(collision_box_stacking)
add_subdirectory(collision_capsule_ground_contact)
add_subdirectory(collision_heightmap)
add_subdirectory(collision_rigid_shapes)

# Control and IK examples
add_subdirectory(ik_humanoid)
add_subdirectory(control_walking_humanoid)
add_subdirectory(control_balance_biped)
add_subdirectory(control_operational_space)
add_subdirectory(control_vehicle)
add_subdirectory(ik_analytic_wam)

# IO and models examples
add_subdirectory(io_unified_loading)
add_subdirectory(model_fetch)

# Soft and hybrid examples
add_subdirectory(hybrid_dynamics)
add_subdirectory(mixed_chain)
add_subdirectory(soft_bodies)

# Visualization and interaction examples
add_subdirectory(add_delete_skels)
add_subdirectory(drag_and_drop)
add_subdirectory(empty)
add_subdirectory(imgui)
add_subdirectory(polyhedron_visual)
add_subdirectory(raylib)
add_subdirectory(simulation_event_handler)
add_subdirectory(tinkertoy)

# Performance and scaling examples
add_subdirectory(headless_simulation)

# Integration and tools examples
if(DART_HAVE_OCTOMAP)
  add_subdirectory(point_cloud)
endif()
add_subdirectory(csv_logger)
