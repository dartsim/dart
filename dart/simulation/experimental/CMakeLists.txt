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

add_library(${target_name}
  body/collision_body.cpp
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
  compute/sequential_executor.cpp
  compute/world_kinematics_graph.cpp
  compute/world_step_stage.cpp
  constraint/loop_closure.cpp
  comps/collision_geometry.cpp
  comps/contact_material.cpp
  comps/dynamics.cpp
  comps/frame_types.cpp
  comps/joint.cpp
  comps/link.cpp
  comps/loop_closure.cpp
  comps/multibody.cpp
  comps/name.cpp
  comps/rigid_body.cpp
  frame/fixed_frame.cpp
  frame/frame.cpp
  frame/free_frame.cpp
  io/binary_io.cpp
  io/serializer.cpp
  multibody/joint.cpp
  multibody/link.cpp
  multibody/multibody.cpp
  space/state_space.cpp
  space/vector_mapper.cpp
  world.cpp
)
dart_apply_strict_symbol_visibility(${target_name})
dart_maybe_apply_coverage_config(${target_name})

target_include_directories(${target_name}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(${target_name} PUBLIC cxx_std_20)

target_link_libraries(${target_name}
  PUBLIC
    dart
    Eigen3::Eigen
    EnTT::EnTT
    spdlog::spdlog
  PRIVATE
    ${PROJECT_NAME}-collision-native
)

if(TARGET Taskflow::Taskflow)
  target_link_libraries(${target_name} PRIVATE Taskflow::Taskflow)
elseif(TARGET Taskflow)
  target_link_libraries(${target_name} PRIVATE Taskflow)
endif()

target_compile_definitions(${target_name}
  PRIVATE
    DART_BUILDING_DART_EXPERIMENTAL
)

# Component
add_component(${PROJECT_NAME} ${component_name})
add_component_targets(${PROJECT_NAME} ${component_name} ${target_name})
add_component_dependencies(${PROJECT_NAME} ${component_name} dart)
add_component_dependency_packages(${PROJECT_NAME} ${component_name} EnTT Taskflow)

install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
  DESTINATION include/dart/simulation/experimental
  COMPONENT headers
  FILES_MATCHING PATTERN "*.hpp"
)
