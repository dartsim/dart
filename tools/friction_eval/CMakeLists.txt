# Friction-solver evaluation harness (see README.md). A standalone
# find_package(DART) build, so it runs against an installed DART 6.19.4 as well
# as the 6.20 line and candidate builds. It is not part of the DART build or
# CI; its scenes run in CI through tests/integration/test_FrictionAnalytic.cpp.
cmake_minimum_required(VERSION 3.22.1)
# C: the 6.20 line's ODE find module runs check_c_source_compiles().
project(dart_friction_eval LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(DART REQUIRED COMPONENTS collision-bullet collision-ode CONFIG)

add_executable(friction_eval friction_eval.cpp)
target_link_libraries(
  friction_eval
  PRIVATE dart dart-collision-bullet dart-collision-ode
)
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  # DT_RPATH (unlike DT_RUNPATH) also resolves the DART libraries' own
  # dependencies, so the binary runs in place against an uninstalled env.
  target_link_options(friction_eval PRIVATE "LINKER:--disable-new-dtags")
endif()
