cmake_minimum_required(VERSION 3.22.1)

project(dart-examples)

# None GUI examples
add_subdirectory(hello_world)
add_subdirectory(speed_test)

# OSG renderer examples
add_subdirectory(atlas_puppet)
add_subdirectory(atlas_simbicon)
add_subdirectory(biped_stand)
add_subdirectory(box_stacking)
add_subdirectory(drag_and_drop)
add_subdirectory(empty)
add_subdirectory(heightmap)
add_subdirectory(hubo_puppet)
add_subdirectory(imgui)
add_subdirectory(operational_space_control)
if(OCTOMAP_FOUND)
  add_subdirectory(point_cloud)
endif()
add_subdirectory(rigid_shapes)
add_subdirectory(soft_bodies)
add_subdirectory(tinkertoy)
add_subdirectory(wam_ikfast)

# MJCF model based examples
add_subdirectory(fetch)

# Deprecated examples
add_subdirectory(deprecated_examples)
