# Simulation Benchmarks (dart::simulation)
#
# Uses the DART simulation CMake helpers.

# Include DART utility functions
include(dart_defs)

# Include dependency finding (makes benchmark available)
include(dart_simulation_dependencies)

if(DART_SIMULATION_BUILD_BENCHMARKS)
  dart_add_simulation_benchmark(
    bm_compute_graph
    ${CMAKE_CURRENT_SOURCE_DIR}/bm_compute_graph.cpp
  )

  dart_add_simulation_benchmark(
    bm_variational_integration
    ${CMAKE_CURRENT_SOURCE_DIR}/bm_variational_integration.cpp
  )

  dart_add_simulation_benchmark(
    bm_deformable_body
    ${CMAKE_CURRENT_SOURCE_DIR}/bm_deformable_body.cpp
  )

  dart_add_simulation_benchmark(
    bm_ipc_distance_kernels
    ${CMAKE_CURRENT_SOURCE_DIR}/bm_ipc_distance_kernels.cpp
  )

  dart_add_simulation_benchmark(
    bm_ipc_candidate_set
    ${CMAKE_CURRENT_SOURCE_DIR}/bm_ipc_candidate_set.cpp
  )

  dart_add_simulation_benchmark(
    bm_ipc_motion_aware_candidate_set
    ${CMAKE_CURRENT_SOURCE_DIR}/bm_ipc_motion_aware_candidate_set.cpp
  )

  dart_add_simulation_benchmark(
    bm_ipc_continuous_collision_step
    ${CMAKE_CURRENT_SOURCE_DIR}/bm_ipc_continuous_collision_step.cpp
  )

  dart_add_simulation_benchmark(
    bm_ipc_barrier_kernel
    ${CMAKE_CURRENT_SOURCE_DIR}/bm_ipc_barrier_kernel.cpp
  )

  dart_add_simulation_benchmark(
    bm_ipc_tangent_stencil
    ${CMAKE_CURRENT_SOURCE_DIR}/bm_ipc_tangent_stencil.cpp
  )

  dart_add_simulation_benchmark(
    bm_rigid_ipc_solver
    ${CMAKE_CURRENT_SOURCE_DIR}/bm_rigid_ipc_solver.cpp
  )

  dart_add_simulation_benchmark(
    bm_affine_body_dynamics
    ${CMAKE_CURRENT_SOURCE_DIR}/bm_affine_body_dynamics.cpp
  )

  dart_add_simulation_benchmark(
    bm_plan083_cpu_scene_corpus
    ${CMAKE_CURRENT_SOURCE_DIR}/bm_plan083_cpu_scene_corpus.cpp
  )

  dart_add_simulation_benchmark(
    bm_vbd_vertex_block_kernel
    ${CMAKE_CURRENT_SOURCE_DIR}/bm_vbd_vertex_block_kernel.cpp
  )

  dart_add_simulation_benchmark(
    bm_vbd_block_descent
    ${CMAKE_CURRENT_SOURCE_DIR}/bm_vbd_block_descent.cpp
  )

  dart_add_simulation_benchmark(
    bm_vbd_neo_hookean
    ${CMAKE_CURRENT_SOURCE_DIR}/bm_vbd_neo_hookean.cpp
  )

  dart_add_simulation_benchmark(
    bm_vbd_world_solver
    ${CMAKE_CURRENT_SOURCE_DIR}/bm_vbd_world_solver.cpp
  )

  dart_add_simulation_benchmark(
    bm_avbd_rigid_fixed_joint
    ${CMAKE_CURRENT_SOURCE_DIR}/bm_avbd_rigid_fixed_joint.cpp
  )

  dart_add_simulation_benchmark(
    bm_ecs_safety
    ${CMAKE_CURRENT_SOURCE_DIR}/bm_ecs_safety.cpp
  )

  # Differentiable simulation gradient benchmark (PLAN-110): analytic step
  # Jacobian vs finite-differencing, and diff-ON vs diff-OFF step overhead.
  # Exercises the detail-only differentiable code path, so it is registered
  # only when DART_BUILD_DIFF is ON.
  if(DART_BUILD_DIFF)
    dart_add_simulation_benchmark(
      bm_diff_gradient
      ${CMAKE_CURRENT_SOURCE_DIR}/bm_diff_gradient.cpp
    )
  endif()

  # These benchmarks include <entt/entt.hpp> directly to exercise ECS internals.
  # The simulation library now keeps EnTT PRIVATE, so it no longer re-exports
  # the entt include path; link it explicitly for the benchmarks that need it.
  foreach(_entt_bm bm_ecs_safety bm_variational_integration bm_vbd_world_solver)
    if(TARGET ${_entt_bm})
      target_link_libraries(${_entt_bm} PRIVATE EnTT::EnTT)
    endif()
  endforeach()

  if(TARGET ${PROJECT_NAME}-simulation-cuda)
    dart_add_simulation_benchmark(
      bm_cuda_rigid_body_state_batch
      ${CMAKE_CURRENT_SOURCE_DIR}/bm_cuda_rigid_body_state_batch.cpp
      LIBRARY ${PROJECT_NAME}-simulation-cuda
      LABEL "simulation-cuda"
    )
    dart_add_simulation_benchmark(
      bm_plan083_gpu_psd_projection
      ${CMAKE_CURRENT_SOURCE_DIR}/bm_plan083_gpu_psd_projection.cpp
      LIBRARY ${PROJECT_NAME}-simulation-cuda
      LABEL "simulation-cuda"
    )
    dart_add_simulation_benchmark(
      bm_plan083_gpu_contact_candidates
      ${CMAKE_CURRENT_SOURCE_DIR}/bm_plan083_gpu_contact_candidates.cpp
      LIBRARY ${PROJECT_NAME}-simulation-cuda
      LABEL "simulation-cuda"
    )
    dart_add_simulation_benchmark(
      bm_plan083_gpu_ccd_line_search
      ${CMAKE_CURRENT_SOURCE_DIR}/bm_plan083_gpu_ccd_line_search.cpp
      LIBRARY ${PROJECT_NAME}-simulation-cuda
      LABEL "simulation-cuda"
    )
    dart_add_simulation_benchmark(
      bm_plan083_gpu_barrier_friction
      ${CMAKE_CURRENT_SOURCE_DIR}/bm_plan083_gpu_barrier_friction.cpp
      LIBRARY ${PROJECT_NAME}-simulation-cuda
      LABEL "simulation-cuda"
    )
    dart_add_simulation_benchmark(
      bm_newton_assembly_solve_cuda
      ${CMAKE_CURRENT_SOURCE_DIR}/bm_newton_assembly_solve_cuda.cpp
      LIBRARY ${PROJECT_NAME}-simulation-cuda
      LABEL "simulation-cuda"
    )
    dart_add_simulation_benchmark(
      bm_plan083_gpu_scene_parity
      ${CMAKE_CURRENT_SOURCE_DIR}/bm_plan083_gpu_scene_parity.cpp
      LIBRARY ${PROJECT_NAME}-simulation-cuda
      LABEL "simulation-cuda"
    )
    dart_add_simulation_benchmark(
      bm_vbd_cuda
      ${CMAKE_CURRENT_SOURCE_DIR}/bm_vbd_cuda.cpp
      LIBRARY ${PROJECT_NAME}-simulation-cuda
      LABEL "simulation-cuda"
    )
  endif()
endif()
