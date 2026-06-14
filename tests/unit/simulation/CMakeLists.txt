# Copyright (c) 2011, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

# Simulation Tests (dart::simulation)
#
# This file only includes test subdirectories. Each subdirectory defines its own tests.
# Uses the DART simulation CMake helpers.

# Include DART utility functions
include(dart_defs)

# Include dependency finding (makes GTest available)
include(dart_simulation_dependencies)

# Enable CTest for test registration
enable_testing()

# Note: Tests will be added to DART_ALL_TESTS via dart_add_simulation_test().
# We collect them after registration to create the meta target.

#==============================================================================
# Unit Tests
#==============================================================================
# To add a new test module:
#   1. Create a directory under this folder (e.g., common/)
#   2. Add test_*.cpp files to the directory
#   3. Call dart_add_unit_test_dir() below

message(STATUS "simulation: Configuring unit tests...")

if(DART_SIMULATION_BUILD_TESTS)
  # Reset DART_ALL_TESTS to avoid accumulating tests across reconfigurations.
  # This cache variable is populated by dart_add_simulation_test() and persists
  # between CMake runs, which would cause the meta target to have stale entries.
  set(DART_ALL_TESTS "" CACHE INTERNAL "List of all tests" FORCE)

  # Register all unit test directories
  dart_add_unit_test_dir(common ${CMAKE_CURRENT_SOURCE_DIR}/common)
  dart_add_unit_test_dir(compute ${CMAKE_CURRENT_SOURCE_DIR}/compute)
  dart_add_unit_test_dir(contact ${CMAKE_CURRENT_SOURCE_DIR}/contact)
  dart_add_unit_test_dir(
    elasticity ${CMAKE_CURRENT_SOURCE_DIR}/elasticity
  )
  if(TARGET test_continuous_collision_step)
    target_link_libraries(
      test_continuous_collision_step
      PRIVATE ${PROJECT_NAME}-collision-native
    )
  endif()
  dart_add_unit_test_dir(io ${CMAKE_CURRENT_SOURCE_DIR}/io)
  if(TARGET test_rigid_ipc_fixture)
    target_link_libraries(
      test_rigid_ipc_fixture
      PRIVATE ${PROJECT_NAME}-collision-native
    )
  endif()
  dart_add_unit_test_dir(deformable_vbd ${CMAKE_CURRENT_SOURCE_DIR}/deformable_vbd)
  dart_add_unit_test_dir(world ${CMAKE_CURRENT_SOURCE_DIR}/world)
  if(TARGET test_skeleton_loader)
    target_link_libraries(test_skeleton_loader PRIVATE ${PROJECT_NAME}-io)
  endif()
  dart_add_unit_test_dir(frame ${CMAKE_CURRENT_SOURCE_DIR}/frame)
  dart_add_unit_test_dir(multibody ${CMAKE_CURRENT_SOURCE_DIR}/multibody)
  dart_add_unit_test_dir(space ${CMAKE_CURRENT_SOURCE_DIR}/space)

  # test_variational_integration runs a long linear-time variational-integrator
  # convergence sweep. It is comfortably fast in Release, but under coverage
  # (gcov) instrumentation it -- like its integration-test siblings test_world
  # and test_deformable_body (~1000-1600s) -- exceeds the default 1500s ctest
  # timeout and fails the Coverage (Debug) job. Give it the same headroom as
  # UNIT_simulation_World (TIMEOUT 3600); the guard keeps this a no-op in build
  # configurations where the test is not registered.
  if(TEST test_variational_integration)
    set_tests_properties(test_variational_integration PROPERTIES TIMEOUT 3600)
    if(DART_CODECOV)
      target_compile_definitions(
        test_variational_integration
        PRIVATE DART_CODECOV=1
      )
    endif()
  endif()

  # These rigid IPC and deformable gates remain active in normal CI, but
  # coverage instrumentation can make their long runtime corpora exceed the
  # default ctest timeout. Keep compiling the targets under Codecov, then skip
  # the heavyweight paper-sized target and give long-running targets enough
  # coverage-specific headroom.
  if(DART_CODECOV)
    if(TEST test_deformable_body)
      set_tests_properties(test_deformable_body PROPERTIES TIMEOUT 3600)
    endif()
    if(TEST test_rigid_ipc_fixture)
      set_tests_properties(test_rigid_ipc_fixture PROPERTIES TIMEOUT 3600)
    endif()
    if(TEST test_rigid_ipc_paper_experiments)
      set_tests_properties(
        test_rigid_ipc_paper_experiments
        PROPERTIES DISABLED TRUE
      )
    endif()
    if(TEST test_world)
      set_tests_properties(test_world PROPERTIES TIMEOUT 3600)
      target_compile_definitions(test_world PRIVATE DART_CODECOV=1)
    endif()
    if(TARGET test_boxed_lcp_contact)
      target_compile_definitions(test_boxed_lcp_contact PRIVATE DART_CODECOV=1)
    endif()
  endif()

  # Differentiable simulation (PLAN-110 WS1). The zero-cost parity test uses only
  # the always-compiled WorldOptions core, so it runs in both build configs. The
  # FD-vs-analytic Jacobian test exercises the detail-only differentiable code
  # path and is registered only when DART_BUILD_DIFF is ON.
  dart_add_simulation_test(
    test_diff_zero_cost_parity
    ${CMAKE_CURRENT_SOURCE_DIR}/diff/test_diff_zero_cost_parity.cpp
  )
  if(DART_BUILD_DIFF)
    dart_add_simulation_test(
      test_diff_smooth_jacobian
      ${CMAKE_CURRENT_SOURCE_DIR}/diff/test_diff_smooth_jacobian.cpp
    )
    dart_add_simulation_test(
      test_diff_contact_jacobian
      ${CMAKE_CURRENT_SOURCE_DIR}/diff/test_diff_contact_jacobian.cpp
    )
    dart_add_simulation_test(
      test_diff_public_contact_jacobian
      ${CMAKE_CURRENT_SOURCE_DIR}/diff/test_diff_public_contact_jacobian.cpp
    )
    dart_add_simulation_test(
      test_diff_contact_gradient_modes
      ${CMAKE_CURRENT_SOURCE_DIR}/diff/test_diff_contact_gradient_modes.cpp
    )
    dart_add_simulation_test(
      test_diff_apply_step_vjp
      ${CMAKE_CURRENT_SOURCE_DIR}/diff/test_diff_apply_step_vjp.cpp
    )
    dart_add_simulation_test(
      test_diff_parameter_jacobian
      ${CMAKE_CURRENT_SOURCE_DIR}/diff/test_diff_parameter_jacobian.cpp
    )
    dart_add_simulation_test(
      test_diff_rollout
      ${CMAKE_CURRENT_SOURCE_DIR}/diff/test_diff_rollout.cpp
    )
    dart_add_simulation_test(
      test_diff_optimization
      ${CMAKE_CURRENT_SOURCE_DIR}/diff/test_diff_optimization.cpp
    )
    dart_add_simulation_test(
      test_diff_paper_experiments
      ${CMAKE_CURRENT_SOURCE_DIR}/diff/test_diff_paper_experiments.cpp
    )
  endif()

  if(TARGET ${PROJECT_NAME}-simulation-cuda)
    dart_add_simulation_test(
      test_rigid_body_state_batch_cuda
      ${CMAKE_CURRENT_SOURCE_DIR}/cuda/test_rigid_body_state_batch_cuda.cpp
      LIBRARY ${PROJECT_NAME}-simulation-cuda
      LABEL simulation-cuda
    )
    dart_add_simulation_test(
      test_deformable_psd_projection_cuda
      ${CMAKE_CURRENT_SOURCE_DIR}/cuda/test_deformable_psd_projection_cuda.cpp
      LIBRARY ${PROJECT_NAME}-simulation-cuda
      LABEL simulation-cuda
    )
    dart_add_simulation_test(
      test_vbd_block_descent_cuda
      ${CMAKE_CURRENT_SOURCE_DIR}/cuda/test_vbd_block_descent_cuda.cpp
      LIBRARY ${PROJECT_NAME}-simulation-cuda
      LABEL simulation-cuda
    )
    dart_add_simulation_test(
      test_lcp_jacobi_batch_cuda
      ${CMAKE_CURRENT_SOURCE_DIR}/cuda/test_lcp_jacobi_batch_cuda.cpp
      LIBRARY ${PROJECT_NAME}-simulation-cuda
      LABEL simulation-cuda
    )
    dart_add_simulation_test(
      test_newton_assembly_solve_cuda
      ${CMAKE_CURRENT_SOURCE_DIR}/cuda/test_newton_assembly_solve_cuda.cpp
      LIBRARY ${PROJECT_NAME}-simulation-cuda
      LABEL simulation-cuda
    )
    dart_add_simulation_test(
      test_contact_candidate_filter_cuda
      ${CMAKE_CURRENT_SOURCE_DIR}/cuda/test_contact_candidate_filter_cuda.cpp
      LIBRARY ${PROJECT_NAME}-simulation-cuda
      LABEL simulation-cuda
    )
    dart_add_simulation_test(
      test_ccd_line_search_cuda
      ${CMAKE_CURRENT_SOURCE_DIR}/cuda/test_ccd_line_search_cuda.cpp
      LIBRARY ${PROJECT_NAME}-simulation-cuda
      LABEL simulation-cuda
    )
    dart_add_simulation_test(
      test_barrier_friction_kernel_cuda
      ${CMAKE_CURRENT_SOURCE_DIR}/cuda/test_barrier_friction_kernel_cuda.cpp
      LIBRARY ${PROJECT_NAME}-simulation-cuda
      LABEL simulation-cuda
    )
    set_tests_properties(test_lcp_jacobi_batch_cuda PROPERTIES TIMEOUT 3600)
  endif()

  # In-tree tests that poke ECS internals include <entt/entt.hpp> directly.
  # EnTT is a PRIVATE dependency of the simulation library (the public surface
  # is entt-free), so test targets do not inherit it. Rather than maintain a
  # per-target list here -- which silently rots as new ECS-internal tests are
  # added and only breaks in clean/FetchContent builds -- dart_add_simulation_test
  # links EnTT::EnTT into every test target it creates. The public-header poison
  # smoke test below is built directly (not via that helper) and stays EnTT-free
  # on purpose.

  # Negative public-header self-containment smoke test.
  #
  # Generate a TU that includes EVERY promoted public header (the exact install
  # allowlist, shared via the DART_SIMULATION_PUBLIC_HEADERS cache variable so
  # it cannot drift) and instantiates a World. It is compiled by a target that
  # links the simulation library but NOT EnTT::EnTT/Taskflow. Because those
  # are PRIVATE dependencies of the library, their include directories are
  # absent here, so any promoted header that (transitively) pulled in
  # <entt/...> or <taskflow/...> would fail to compile with "file not found".
  if(DART_SIMULATION_PUBLIC_HEADERS)
    set(_smoke_includes "")
    foreach(_hdr IN LISTS DART_SIMULATION_PUBLIC_HEADERS)
      string(APPEND _smoke_includes "#include <dart/simulation/${_hdr}>\n")
    endforeach()
    set(DART_SIMULATION_PUBLIC_HEADER_INCLUDES "${_smoke_includes}")
    set(_smoke_src "${CMAKE_CURRENT_BINARY_DIR}/public_header_smoke.cpp")
    configure_file(
      ${CMAKE_CURRENT_SOURCE_DIR}/public_headers/public_header_smoke.cpp.in
      ${_smoke_src}
      @ONLY
    )
    add_executable(test_simulation_public_header_smoke ${_smoke_src})
    # Link ONLY the simulation library. EnTT/Taskflow are PRIVATE to it, so
    # this target does NOT inherit their link interface.
    target_link_libraries(
      test_simulation_public_header_smoke
      PRIVATE dart-simulation
    )
    # EnTT and Taskflow are header-only and live in $CONDA_PREFIX/include, the
    # same directory Eigen/spdlog ship in, so dropping the link interface is not
    # enough to hide their headers from the include path. Add a poison directory
    # with the HIGHEST priority (BEFORE): any <entt/...>/<taskflow/...> reachable
    # from a promoted header then resolves to a header that #errors, failing the
    # build before the real headers are ever found. Eigen/spdlog are unaffected.
    target_include_directories(
      test_simulation_public_header_smoke
      BEFORE
      PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/public_headers/poison
    )
    set_target_properties(
      test_simulation_public_header_smoke
      PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${DART_BINARY_DIR}/bin"
        FOLDER "simulation/tests"
    )
    add_test(
      NAME test_simulation_public_header_smoke
      COMMAND $<TARGET_FILE:test_simulation_public_header_smoke>
    )
    set_tests_properties(
      test_simulation_public_header_smoke
      PROPERTIES LABELS "simulation"
    )
    set(
      DART_ALL_TESTS
      ${DART_ALL_TESTS}
      test_simulation_public_header_smoke
      CACHE INTERNAL
      "List of all tests"
    )
  endif()

  # Create meta target for all unit tests
  # DART_ALL_TESTS now contains only the tests added by this module
  add_custom_target(
    dart_simulation_tests
    DEPENDS ${DART_ALL_TESTS}
    COMMENT "Building all simulation unit tests"
  )

  list(LENGTH DART_ALL_TESTS num_tests)
  message(STATUS "simulation: Total ${num_tests} unit test(s)")
else()
  message(STATUS "simulation: Unit tests disabled (GTest not found)")
endif()
