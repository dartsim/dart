# Copyright (c) 2011, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the following "BSD-style" License:
#   Redistribution and use in source and binary forms, with or
#   without modification, are permitted provided that the following
#   conditions are met:
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
#   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
#   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
#   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
#   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
#   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
#   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#   POSSIBILITY OF SUCH DAMAGE.

set(target_name ${PROJECT_NAME}-collision-experimental)
set(component_name collision-experimental)

# Source files - start minimal, add as we implement
set(collision_experimental_sources
  aabb.cpp
  broad_phase/aabb_tree.cpp
  broad_phase/brute_force.cpp
  broad_phase/spatial_hash.cpp
  broad_phase/sweep_and_prune.cpp
  collision_filter.cpp
  collision_object.cpp
  collision_world.cpp
  contact_manifold.cpp
  narrow_phase/box_box.cpp
  narrow_phase/capsule_box.cpp
  narrow_phase/capsule_capsule.cpp
  narrow_phase/capsule_sphere.cpp
  narrow_phase/ccd.cpp
  narrow_phase/convex_convex.cpp
  narrow_phase/cylinder_collision.cpp
  narrow_phase/distance.cpp
  narrow_phase/gjk.cpp
  narrow_phase/mpr.cpp
  narrow_phase/narrow_phase.cpp
  narrow_phase/plane_sphere.cpp
  narrow_phase/raycast.cpp
  narrow_phase/sphere_box.cpp
  narrow_phase/sphere_sphere.cpp
  sdf/dense_esdf_field.cpp
  sdf/dense_sdf_field.cpp
  sdf/dense_tsdf_field.cpp
  shapes/shape.cpp
  types.cpp
)

add_library(${target_name} ${collision_experimental_sources})

include(dart_defs)
include(dart_experimental_dependencies)

target_include_directories(${target_name}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(${target_name} PUBLIC cxx_std_20)

target_link_libraries(${target_name}
  PUBLIC
    Eigen3::Eigen
    EnTT::EnTT
)

target_compile_definitions(${target_name}
  PRIVATE
    DART_BUILDING_COLLISION_EXPERIMENTAL
)

# Benchmarks (optional)
dart_experimental_add_benchmark(
  bm_collision_experimental_sdf_compare
  ${CMAKE_CURRENT_SOURCE_DIR}/benchmarks/bm_sdf_compare.cpp
  LIBRARY ${target_name}
  LABEL collision-experimental
)

if(TARGET bm_collision_experimental_sdf_compare)
  set(DART_EXPERIMENTAL_VOXBLOX_ROOT "" CACHE PATH "Path to Voxblox for experimental collision benchmarks")
  if(DART_EXPERIMENTAL_VOXBLOX_ROOT)
    find_path(VOXBLOX_INCLUDE_DIR
      NAMES voxblox/core/esdf_map.h
      PATHS
        ${DART_EXPERIMENTAL_VOXBLOX_ROOT}
        ${DART_EXPERIMENTAL_VOXBLOX_ROOT}/voxblox/include
        ${DART_EXPERIMENTAL_VOXBLOX_ROOT}/include
    )
    find_library(VOXBLOX_LIBRARY
      NAMES voxblox
      PATHS
        ${DART_EXPERIMENTAL_VOXBLOX_ROOT}/lib
        ${DART_EXPERIMENTAL_VOXBLOX_ROOT}/build
        ${DART_EXPERIMENTAL_VOXBLOX_ROOT}/devel/lib
    )
  endif()

  if(VOXBLOX_INCLUDE_DIR AND VOXBLOX_LIBRARY)
    target_include_directories(bm_collision_experimental_sdf_compare PRIVATE ${VOXBLOX_INCLUDE_DIR})
    target_link_libraries(bm_collision_experimental_sdf_compare PRIVATE ${VOXBLOX_LIBRARY})
    target_compile_definitions(bm_collision_experimental_sdf_compare PRIVATE DART_EXPERIMENTAL_HAVE_VOXBLOX=1)
  endif()
endif()

# Component
add_component(${PROJECT_NAME} ${component_name})
add_component_targets(${PROJECT_NAME} ${component_name} ${target_name})
add_component_dependency_packages(${PROJECT_NAME} ${component_name} EnTT)

install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
  DESTINATION include/dart/collision/experimental
  COMPONENT headers
  FILES_MATCHING PATTERN "*.hpp"
)
