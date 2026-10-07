cmake_minimum_required(VERSION 3.22)
project(dart_perf LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(
  DART
  REQUIRED
  COMPONENTS utils-urdf collision-ode collision-bullet
  CONFIG
)
include(CheckCXXSourceCompiles)
set(CMAKE_REQUIRED_LIBRARIES dart)
check_cxx_source_compiles(
  "#include <dart/simulation/World.hpp>
int main() { dart::simulation::World w; w.setNumSimulationThreads(1); }"
  PERF_HAS_THREADS
)
check_cxx_source_compiles(
  "#include <dart/collision/CollisionOption.hpp>
int main() { dart::collision::CollisionOption o; o.maxNumContactsPerPair = 4; }"
  PERF_HAS_PER_PAIR
)
check_cxx_source_compiles(
  "#include <dart/simulation/World.hpp>
int main() { dart::simulation::World w; auto o = w.getDeactivationOptions();
  o.mEnabled = false; w.setDeactivationOptions(o); }"
  PERF_HAS_DEACTIVATION
)

add_executable(portable_step_bench portable_step_bench.cpp)
target_link_libraries(
  portable_step_bench
  PRIVATE
    dart
    dart-utils
    dart-utils-urdf
    dart-collision-ode
    dart-collision-bullet
)
foreach(feature THREADS PER_PAIR DEACTIVATION)
  target_compile_definitions(
    portable_step_bench
    PRIVATE PERF_HAS_${feature}=$<BOOL:${PERF_HAS_${feature}}>
  )
endforeach()
foreach(shim allocshim heappad)
  add_library(${shim} SHARED ${shim}.c)
  target_link_libraries(${shim} PRIVATE ${CMAKE_DL_LIBS})
endforeach()
