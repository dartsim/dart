# Gazebo benchmark and compatibility drivers for DART. They link one Gazebo
# release's gz-physics, gz-sim and DART, so tools/gazebo/compat/lane.sh builds
# them per lane (`lane.sh <lane> bench`); they are not part of the DART build.
cmake_minimum_required(VERSION 3.22.1)
project(dart_gazebo_bench LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(GZ_RELEASE "ionic" CACHE STRING "Gazebo release: harmonic, ionic or jetty")
set(
  GZ_PHYSICS_SOURCE_DIR
  ""
  CACHE PATH
  "gz-physics source tree, for the dartsim plugin's RetrieveWorld feature"
)

if(GZ_RELEASE STREQUAL "harmonic")
  set(gz_physics gz-physics7)
  set(gz_plugin gz-plugin2)
  set(gz_sim gz-sim8)
  set(gz_common gz-common5)
  set(sdformat sdformat14)
elseif(GZ_RELEASE STREQUAL "ionic")
  set(gz_physics gz-physics8)
  set(gz_plugin gz-plugin3)
  set(gz_sim gz-sim9)
  set(gz_common gz-common6)
  set(sdformat sdformat15)
elseif(GZ_RELEASE STREQUAL "jetty")
  # Jetty packages drop the major version from their names.
  set(gz_physics gz-physics)
  set(gz_plugin gz-plugin)
  set(gz_sim gz-sim)
  set(gz_common gz-common)
  set(sdformat sdformat)
else()
  message(FATAL_ERROR "Unknown GZ_RELEASE '${GZ_RELEASE}'")
endif()
if(NOT EXISTS "${GZ_PHYSICS_SOURCE_DIR}/dartsim/src/World.hh")
  message(FATAL_ERROR "Set GZ_PHYSICS_SOURCE_DIR to a gz-physics source tree")
endif()

find_package(${gz_physics} REQUIRED COMPONENTS sdf)
find_package(${gz_plugin} REQUIRED COMPONENTS loader)
find_package(${gz_sim} REQUIRED)
find_package(${gz_common} REQUIRED)
find_package(${sdformat} REQUIRED)
find_package(DART REQUIRED CONFIG)

set(
  gz_physics_libraries
  ${gz_physics}::${gz_physics}
  ${gz_physics}::${gz_physics}-sdf
  ${gz_plugin}::loader
  ${sdformat}::${sdformat}
)

add_executable(gz_physics_step_bench gz_physics_step_bench.cc)
target_link_libraries(gz_physics_step_bench PRIVATE ${gz_physics_libraries})

add_executable(gz_sim_server_bench gz_sim_server_bench.cc)
target_link_libraries(
  gz_sim_server_bench
  PRIVATE ${gz_sim}::${gz_sim} ${gz_common}::${gz_common}
)

add_executable(gz_sleep_oracle gz_sleep_oracle.cc)
target_include_directories(
  gz_sleep_oracle
  PRIVATE "${GZ_PHYSICS_SOURCE_DIR}/dartsim/src"
)
target_link_libraries(gz_sleep_oracle PRIVATE ${gz_physics_libraries} dart)

# Batched rays arrived in gz-physics 9 (Jetty), whose dartsim plugin casts
# rays over DART's ODE and Bullet collision groups itself.
if(GZ_RELEASE STREQUAL "jetty")
  add_executable(gz_raycast_probe gz_raycast_probe.cc)
  target_link_libraries(gz_raycast_probe PRIVATE ${gz_physics_libraries})
endif()
