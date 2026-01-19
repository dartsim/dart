# Simulation-experimental Benchmarks (dart::simulation::experimental)
#
# Uses the dart_experimental_* CMake helpers.

# Include DART utility functions
include(dart_defs)

# Include dependency finding (makes benchmark available)
include(dart_experimental_dependencies)

if(DART_EXPERIMENTAL_BUILD_BENCHMARKS)
  dart_experimental_add_benchmark(
    bm_ecs_safety
    ${CMAKE_CURRENT_SOURCE_DIR}/bm_ecs_safety.cpp
  )
  dart_experimental_add_benchmark(
    bm_multi_body
    ${CMAKE_CURRENT_SOURCE_DIR}/bm_multi_body.cpp
  )
endif()
