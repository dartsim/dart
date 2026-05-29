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
  common/profiling.cpp
  compute/compute_graph.cpp
  compute/compute_graph_visualization.cpp
  compute/compute_node.cpp
  compute/compute_stage_metadata.cpp
  compute/execution_profile.cpp
  compute/multibody_dynamics.cpp
  compute/parallel_executor.cpp
  compute/rigid_body_state_batch.cpp
  compute/sequential_executor.cpp
  compute/world_batch.cpp
  compute/world_kinematics_graph.cpp
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
  detail/deformable_contact/continuous_collision_step.cpp
  frame/fixed_frame.cpp
  frame/frame.cpp
  frame/free_frame.cpp
  io/binary_io.cpp
  io/deformable_scene_io.cpp
  io/serializer.cpp
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
  PRIVATE ${PROJECT_NAME}-collision-native
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

if(DART_ENABLE_EXPERIMENTAL_CUDA)
  find_package(CUDAToolkit REQUIRED)

  set(cuda_target_name ${target_name}-cuda)
  add_library(
    ${cuda_target_name}
    STATIC
    compute/cuda/rigid_body_state_batch_cuda.cpp
    compute/cuda/rigid_body_state_batch_cuda.cu
    compute/cuda/deformable_psd_projection_cuda.cpp
    compute/cuda/deformable_psd_projection_cuda.cu
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
add_component_dependencies(${PROJECT_NAME} ${component_name} dart)
add_component_dependency_packages(${PROJECT_NAME} ${component_name} EnTT Taskflow)

install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
  DESTINATION include/dart/simulation/experimental
  COMPONENT headers
  FILES_MATCHING
  PATTERN "*.hpp"
)
