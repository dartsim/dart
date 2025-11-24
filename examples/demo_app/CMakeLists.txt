cmake_minimum_required(VERSION 3.22.1)

project(demo_app)

set(required_components gui)
set(required_libraries dart dart-gui)

if(NOT DART_IN_SOURCE_BUILD)
  find_package(DART 6.16.0 REQUIRED COMPONENTS ${required_components} CONFIG)
endif()

set(demo_app_include_dir ${CMAKE_CURRENT_LIST_DIR})

set(core_sources
  app/DemoApp.cpp
  app/DemoApp.hpp
  core/Scene.hpp
  core/SceneRegistry.cpp
  core/SceneRegistry.hpp
  core/WorldNodeHooks.hpp
)

add_library(demo_app_core STATIC ${core_sources})
target_include_directories(demo_app_core PUBLIC ${demo_app_include_dir})
target_link_libraries(demo_app_core PUBLIC ${required_libraries})
dart_format_add(${core_sources})

set(DEMO_APP_SCENES "")

add_subdirectory(scenes/basic_dynamics)
add_subdirectory(scenes/controlled_pendulum)

add_executable(demo_app main.cpp)
target_link_libraries(demo_app PRIVATE demo_app_core ${DEMO_APP_SCENES})
set_target_properties(demo_app
  PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${DART_BINARY_DIR}/bin"
)
dart_add_example(demo_app)
dart_format_add(main.cpp)
