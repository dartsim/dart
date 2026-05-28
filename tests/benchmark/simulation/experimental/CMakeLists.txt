# Simulation-experimental Benchmarks (dart::simulation::experimental)
#
# Uses the dart_experimental_* CMake helpers.

# Include DART utility functions
include(dart_defs)

# Include dependency finding (makes benchmark available)
include(dart_experimental_dependencies)

if(DART_EXPERIMENTAL_BUILD_BENCHMARKS)
  dart_experimental_add_benchmark(
    bm_compute_graph
    ${CMAKE_CURRENT_SOURCE_DIR}/bm_compute_graph.cpp
  )

  dart_experimental_add_benchmark(
    bm_deformable_body
    ${CMAKE_CURRENT_SOURCE_DIR}/bm_deformable_body.cpp
  )

  dart_experimental_add_benchmark(
    bm_ipc_distance_kernels
    ${CMAKE_CURRENT_SOURCE_DIR}/bm_ipc_distance_kernels.cpp
  )

  dart_experimental_add_benchmark(
    bm_ipc_candidate_set
    ${CMAKE_CURRENT_SOURCE_DIR}/bm_ipc_candidate_set.cpp
  )

  dart_experimental_add_benchmark(
    bm_ipc_continuous_collision_step
    ${CMAKE_CURRENT_SOURCE_DIR}/bm_ipc_continuous_collision_step.cpp
  )

  dart_experimental_add_benchmark(
    bm_ipc_barrier_kernel
    ${CMAKE_CURRENT_SOURCE_DIR}/bm_ipc_barrier_kernel.cpp
  )

  dart_experimental_add_benchmark(
    bm_ipc_tangent_stencil
    ${CMAKE_CURRENT_SOURCE_DIR}/bm_ipc_tangent_stencil.cpp
  )

  dart_experimental_add_benchmark(
    bm_ecs_safety
    ${CMAKE_CURRENT_SOURCE_DIR}/bm_ecs_safety.cpp
  )

  if(TARGET ${PROJECT_NAME}-simulation-experimental-cuda)
    dart_experimental_add_benchmark(
      bm_cuda_rigid_body_state_batch
      ${CMAKE_CURRENT_SOURCE_DIR}/bm_cuda_rigid_body_state_batch.cpp
      LIBRARY ${PROJECT_NAME}-simulation-experimental-cuda
      LABEL "simulation-experimental-cuda"
    )
  endif()
endif()
