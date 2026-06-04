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

set(target_name ${PROJECT_NAME}-simulation-experimental)
set(component_name simulation-experimental)

include(dart_defs)
include(dart_experimental_dependencies)

add_library(
  ${target_name}
  body/collision_body.cpp
  body/deformable_body.cpp
  body/rigid_body.cpp
  common/diagnostics.cpp
  common/exceptions.cpp
  compute/compute_graph.cpp
  compute/compute_graph_visualization.cpp
  compute/compute_node.cpp
  compute/compute_stage_metadata.cpp
  compute/deformable_psd_backend.cpp
  compute/execution_profile.cpp
  compute/multibody_constraint.cpp
  compute/multibody_dynamics.cpp
  compute/parallel_executor.cpp
  compute/rigid_body_constraint.cpp
  compute/rigid_body_state_batch.cpp
  compute/unified_constraint.cpp
  compute/variational_integration.cpp
  compute/sequential_executor.cpp
  compute/world_batch.cpp
  compute/world_kinematics_graph.cpp
  compute/world_step_profile.cpp
  compute/world_step_stage.cpp
  constraint/loop_closure.cpp
  comps/collision_geometry.cpp
  comps/contact_material.cpp
  comps/deformable_body.cpp
  comps/dynamics.cpp
  comps/frame_types.cpp
  comps/joint.cpp
  comps/link.cpp
  comps/loop_closure.cpp
  comps/multibody.cpp
  comps/name.cpp
  comps/rigid_body.cpp
  detail/affine_body_dynamics.cpp
  detail/boxed_lcp_contact.cpp
  detail/deformable_contact/continuous_collision_step.cpp
  detail/rigid_ipc_barrier.cpp
  detail/rigid_ipc_ccd.cpp
  frame/fixed_frame.cpp
  frame/frame.cpp
  frame/free_frame.cpp
  io/binary_io.cpp
  io/codim_mesh.cpp
  io/deformable_scene_io.cpp
  io/detail/rigid_ipc_fixture.cpp
  io/gmsh_tet_mesh.cpp
  io/obj_triangle_mesh.cpp
  io/serializer.cpp
  io/skeleton_loader.cpp
  io/skeleton_to_multibody.cpp
  multibody/joint.cpp
  multibody/link.cpp
  multibody/multibody.cpp
  space/state_space.cpp
  space/vector_mapper.cpp
  world.cpp
)
set_target_properties(
  ${target_name}
  PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${DART_BINARY_DIR}/lib"
    LIBRARY_OUTPUT_DIRECTORY "${DART_BINARY_DIR}/lib"
    RUNTIME_OUTPUT_DIRECTORY "${DART_BINARY_DIR}/bin"
)
dart_apply_strict_symbol_visibility(${target_name})
dart_maybe_apply_coverage_config(${target_name})

target_include_directories(
  ${target_name}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(${target_name} PUBLIC cxx_std_20)

target_link_libraries(
  ${target_name}
  PUBLIC dart Eigen3::Eigen EnTT::EnTT spdlog::spdlog
  PRIVATE ${PROJECT_NAME}-collision-native ${PROJECT_NAME}-io
)

if(TARGET Taskflow::Taskflow)
  target_link_libraries(${target_name} PRIVATE Taskflow::Taskflow)
elseif(TARGET Taskflow)
  target_link_libraries(${target_name} PRIVATE Taskflow)
endif()

target_compile_definitions(
  ${target_name}
  PRIVATE DART_BUILDING_DART_EXPERIMENTAL
)

# Opt-in differentiable simulation (PLAN-110). The diff sources are pure
# C++/Eigen and compile into the experimental target only when DART_BUILD_DIFF is
# ON. DART_HAS_DIFF is PUBLIC so in-tree consumers (the diff tests) inherit it
# and can compile the detail-only differentiable code path. Differentiability
# remains a runtime opt-in via WorldOptions::differentiable.
if(DART_BUILD_DIFF)
  target_sources(
    ${target_name}
    PRIVATE
      detail/contact_jacobians.cpp
      detail/smooth_jacobians.cpp
      diff/rollout.cpp
  )
  target_compile_definitions(${target_name} PUBLIC DART_HAS_DIFF)
endif()

if(DART_ENABLE_EXPERIMENTAL_CUDA)
  find_package(CUDAToolkit REQUIRED)

  set(cuda_target_name ${target_name}-cuda)
  add_library(
    ${cuda_target_name}
    STATIC
    compute/cuda/cuda_runtime.cpp
    compute/cuda/rigid_body_state_batch_cuda.cpp
    compute/cuda/rigid_body_state_batch_cuda.cu
    compute/cuda/deformable_psd_projection_cuda.cpp
    compute/cuda/deformable_psd_projection_cuda.cu
    compute/cuda/vbd_block_descent_cuda.cu
  )
  dart_maybe_apply_coverage_config(${cuda_target_name})

  target_include_directories(
    ${cuda_target_name}
    PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
  )

  target_compile_features(${cuda_target_name} PUBLIC cxx_std_20)

  target_link_libraries(
    ${cuda_target_name}
    PUBLIC ${target_name}
    PRIVATE CUDA::cudart
  )

  target_compile_options(
    ${cuda_target_name}
    PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:--expt-relaxed-constexpr>
  )

  set_target_properties(
    ${cuda_target_name}
    PROPERTIES
      POSITION_INDEPENDENT_CODE ON
      CUDA_STANDARD 17
      CUDA_STANDARD_REQUIRED ON
      FOLDER "${component_name}/cuda"
  )

  if(DART_CUDA_ARCHITECTURES)
    set_target_properties(
      ${cuda_target_name}
      PROPERTIES CUDA_ARCHITECTURES "${DART_CUDA_ARCHITECTURES}"
    )
  endif()
endif()

# Component
add_component(${PROJECT_NAME} ${component_name})
add_component_targets(${PROJECT_NAME} ${component_name} ${target_name})
add_component_dependencies(${PROJECT_NAME} ${component_name} dart io)
add_component_dependency_packages(${PROJECT_NAME} ${component_name} EnTT Taskflow)

install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
  DESTINATION include/dart/simulation/experimental
  COMPONENT headers
  FILES_MATCHING
  PATTERN "*.hpp"
)
