cmake_minimum_required(VERSION 3.22.1)

project(dart-examples)

# Getting started examples
add_subdirectory(hello_world)
add_subdirectory(simple_frames)

# Rigid bodies and frames examples
add_subdirectory(boxes)
add_subdirectory(hardcoded_design)
add_subdirectory(rigid_cubes)

# Joints and constraints examples
add_subdirectory(coupler_constraint)
add_subdirectory(free_joint_cases)
add_subdirectory(human_joint_limits)
add_subdirectory(joint_constraints)
add_subdirectory(mimic_pendulums)
add_subdirectory(rigid_chain)
add_subdirectory(rigid_loop)
add_subdirectory(lcp_solvers)

# Collisions and contacts examples
add_subdirectory(box_stacking)
add_subdirectory(capsule_ground_contact)
add_subdirectory(heightmap)
add_subdirectory(rigid_shapes)

# Control and IK examples
add_subdirectory(atlas_puppet)
add_subdirectory(atlas_simbicon)
add_subdirectory(biped_stand)
add_subdirectory(hubo_puppet)
add_subdirectory(operational_space_control)
add_subdirectory(vehicle)
add_subdirectory(wam_ikfast)

# IO and models examples
add_subdirectory(unified_loading)
add_subdirectory(fetch)
add_subdirectory(g1_puppet)

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
# raylib example is added conditionally from root CMakeLists.txt
# when DART_BUILD_GUI_RAYLIB=ON to avoid duplicate binary directory errors
if(NOT DART_BUILD_GUI_RAYLIB)
  add_subdirectory(raylib)
  add_subdirectory(raylib_drag_and_drop)
  add_subdirectory(raylib_imgui)
endif()
add_subdirectory(simulation_event_handler)
add_subdirectory(tinkertoy)

# Performance and scaling examples
add_subdirectory(headless_simulation)
add_subdirectory(speed_test)

# Integration and tools examples
if(DART_HAVE_OCTOMAP)
  add_subdirectory(point_cloud)
endif()
add_subdirectory(csv_logger)
add_subdirectory(rerun)
