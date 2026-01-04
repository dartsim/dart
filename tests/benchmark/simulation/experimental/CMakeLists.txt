# Simulation-experimental Benchmarks (dart::simulation::experimental)
#
# Uses the dart_experimental_* CMake helpers.

# Include dart_experimental utility functions (must come first to define dart_experimental_find_package)
include(dart_experimental_defs)

# Include dependency finding (makes benchmark available)
include(dart_experimental_dependencies)

if(DART_EXPERIMENTAL_BUILD_BENCHMARKS)
  dart_experimental_add_benchmark(
    bm_ecs_safety
    ${CMAKE_CURRENT_SOURCE_DIR}/bm_ecs_safety.cpp
  )
endif()
