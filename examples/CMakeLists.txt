# Copyright (c) 2011-2021, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

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
if(DART_HAVE_OCTOMAP)
  add_subdirectory(point_cloud)
endif()
add_subdirectory(soft_bodies)
add_subdirectory(tinkertoy)
add_subdirectory(wam_ikfast)

# MJCF model based examples
add_subdirectory(fetch)

# Deprecated examples
add_subdirectory(deprecated_examples)
