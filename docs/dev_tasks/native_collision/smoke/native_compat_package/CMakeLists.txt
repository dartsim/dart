cmake_minimum_required(VERSION 3.22)
project(native_collision_compat_package_smoke LANGUAGES CXX)

find_package(Tracy CONFIG REQUIRED)
find_package(DART REQUIRED COMPONENTS collision-fcl collision-bullet collision-ode)

add_executable(native_collision_compat_package_smoke main.cpp)
target_link_libraries(
  native_collision_compat_package_smoke
  PRIVATE dart-collision-fcl dart-collision-bullet dart-collision-ode)
