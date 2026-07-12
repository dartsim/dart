# Copyright (c) 2011, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the following "BSD-style" License.

set(target_name ${PROJECT_NAME}-simulation)
set(component_name simulation)

include(dart_defs)
include(dart_simulation_dependencies)

file(GLOB_RECURSE dart_simulation_headers CONFIGURE_DEPENDS "*.hpp" "*.cuh")
file(GLOB_RECURSE dart_simulation_sources CONFIGURE_DEPENDS "*.cpp")

list(FILTER dart_simulation_sources EXCLUDE REGEX "/compute/cuda/")
list(
  REMOVE_ITEM dart_simulation_sources
  "${CMAKE_CURRENT_SOURCE_DIR}/detail/contact_jacobians.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/detail/smooth_jacobians.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/diff/rollout.cpp"
)

add_library(
  ${target_name}
  ${dart_simulation_headers}
  ${dart_simulation_sources}
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
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(${target_name} PUBLIC cxx_std_23)
target_compile_definitions(
  ${target_name}
  PRIVATE
    DART_BUILDING_DART_SIMULATION
    DART_SIMULATION_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
)

target_link_libraries(
  ${target_name}
  PUBLIC dart Eigen3::Eigen
  PRIVATE
    EnTT::EnTT
    spdlog::spdlog
    ${PROJECT_NAME}-collision-native
    ${PROJECT_NAME}-io
)

if(TARGET Taskflow::Taskflow)
  target_link_libraries(${target_name} PRIVATE Taskflow::Taskflow)
elseif(TARGET Taskflow)
  target_link_libraries(${target_name} PRIVATE Taskflow)
endif()

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
    compute/cuda/barrier_friction_kernel_cuda.cpp
    compute/cuda/barrier_friction_kernel_cuda.cu
    compute/cuda/ccd_line_search_cuda.cpp
    compute/cuda/ccd_line_search_cuda.cu
    compute/cuda/cuda_runtime.cpp
    compute/cuda/contact_candidate_filter_cuda.cpp
    compute/cuda/contact_candidate_filter_cuda.cu
    compute/cuda/lcp_jacobi_batch_cuda.cpp
    compute/cuda/lcp_jacobi_batch_cuda.cu
    compute/cuda/newton_assembly_solve_cuda.cpp
    compute/cuda/newton_assembly_solve_cuda.cu
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

add_component(${PROJECT_NAME} ${component_name})
add_component_targets(${PROJECT_NAME} ${component_name} ${target_name})
add_component_dependencies(
  ${PROJECT_NAME}
  ${component_name}
  dart
  collision-native
  io
)
if(NOT BUILD_SHARED_LIBS)
  add_component_dependency_packages(
    ${PROJECT_NAME}
    ${component_name}
    EnTT
    Taskflow
    spdlog
  )
endif()

set(
  dart_simulation_public_headers_toplevel
  All.hpp
  world.hpp
  world_options.hpp
  world_sync_stage.hpp
  entity.hpp
  fwd.hpp
  export.hpp
  version.hpp
)
set(
  dart_simulation_public_headers_body
  body/collision_body.hpp
  body/collision_shape.hpp
  body/contact.hpp
  body/contact_force.hpp
  body/deformable_body.hpp
  body/deformable_body_options.hpp
  body/rigid_body.hpp
  body/rigid_body_options.hpp
)
set(
  dart_simulation_public_headers_multibody
  multibody/joint.hpp
  multibody/joint_type.hpp
  multibody/link.hpp
  multibody/multibody.hpp
  multibody/multibody_options.hpp
)
set(
  dart_simulation_public_headers_frame
  frame/fixed_frame.hpp
  frame/frame.hpp
  frame/free_frame.hpp
)
set(
  dart_simulation_public_headers_constraint
  constraint/loop_closure.hpp
  constraint/loop_closure_family.hpp
  constraint/loop_closure_residual.hpp
  constraint/loop_closure_runtime_policy.hpp
  constraint/loop_closure_spec.hpp
)
set(
  dart_simulation_public_headers_diff
  diff/step_derivatives.hpp
  diff/step_gradient.hpp
)
set(
  dart_simulation_public_headers_compute
  # Custom-stage extension contract only. Built-in family stages and their
  # internal stats live under compute/detail and are intentionally not installed.
  compute/compute_executor.hpp
  compute/compute_stage_metadata.hpp
  compute/execution_profile.hpp
  compute/parallel_executor.hpp
  compute/sequential_executor.hpp
  compute/world_step_profile.hpp
  compute/world_step_stage.hpp
)
set(dart_simulation_public_headers_diff_optin diff/rollout.hpp)
set(
  dart_simulation_public_headers
  ${dart_simulation_public_headers_toplevel}
  ${dart_simulation_public_headers_body}
  ${dart_simulation_public_headers_multibody}
  ${dart_simulation_public_headers_frame}
  ${dart_simulation_public_headers_constraint}
  ${dart_simulation_public_headers_compute}
  ${dart_simulation_public_headers_diff}
)
if(DART_BUILD_DIFF)
  list(
    APPEND dart_simulation_public_headers
    ${dart_simulation_public_headers_diff_optin}
  )
endif()
set(
  DART_SIMULATION_PUBLIC_HEADERS
  ${dart_simulation_public_headers}
  CACHE INTERNAL
  "Promoted public headers of dart::simulation"
)
set(dart_simulation_public_source_headers)
foreach(_dart_public_header IN LISTS dart_simulation_public_headers)
  list(
    APPEND dart_simulation_public_source_headers
    "${CMAKE_CURRENT_SOURCE_DIR}/${_dart_public_header}"
  )
endforeach()

install(
  FILES ${dart_simulation_public_headers_toplevel}
  DESTINATION include/dart/simulation
  COMPONENT headers
)
install(
  FILES ${dart_simulation_public_headers_body}
  DESTINATION include/dart/simulation/body
  COMPONENT headers
)
install(
  FILES ${dart_simulation_public_headers_multibody}
  DESTINATION include/dart/simulation/multibody
  COMPONENT headers
)
install(
  FILES ${dart_simulation_public_headers_frame}
  DESTINATION include/dart/simulation/frame
  COMPONENT headers
)
install(
  FILES ${dart_simulation_public_headers_constraint}
  DESTINATION include/dart/simulation/constraint
  COMPONENT headers
)
install(
  FILES ${dart_simulation_public_headers_compute}
  DESTINATION include/dart/simulation/compute
  COMPONENT headers
)
install(
  FILES ${dart_simulation_public_headers_diff}
  DESTINATION include/dart/simulation/diff
  COMPONENT headers
)
if(DART_BUILD_DIFF)
  install(
    FILES ${dart_simulation_public_headers_diff_optin}
    DESTINATION include/dart/simulation/diff
    COMPONENT headers
  )
endif()

set(dart_simulation_generated_umbrella_inputs)
foreach(_dart_public_header IN LISTS dart_simulation_public_headers_toplevel)
  if(NOT _dart_public_header STREQUAL "All.hpp")
    list(
      APPEND dart_simulation_generated_umbrella_inputs
      "${_dart_public_header}"
    )
  endif()
endforeach()
dart_get_filename_components(
  dart_simulation_header_names
  "simulation headers"
  ${dart_simulation_generated_umbrella_inputs}
)
foreach(_dart_public_header IN LISTS dart_simulation_public_headers)
  if(_dart_public_header MATCHES "/")
    list(APPEND dart_simulation_header_names "${_dart_public_header}")
  endif()
endforeach()
dart_generate_component_headers(
  COMPONENT_NAME simulation
  TARGET_DIR "dart/simulation/"
  OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}"
  HEADERS ${dart_simulation_header_names}
  SOURCE_HEADERS ${dart_simulation_public_source_headers}
)
install(
  FILES ${CMAKE_CURRENT_BINARY_DIR}/simulation.hpp
  DESTINATION include/dart/simulation
  COMPONENT headers
)
dart_install_compat_headers(
  COMPAT_HEADERS ${DART_GENERATED_COMPAT_HEADERS}
  DESTINATION_PREFIX include/dart/simulation
)

dart_format_add(${dart_simulation_headers} ${dart_simulation_sources})
