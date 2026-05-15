cmake_minimum_required(VERSION 3.22.1)

get_filename_component(example_name ${CMAKE_CURRENT_LIST_DIR} NAME)

project(${example_name})

set(example_target dart_${example_name})

if(DART_IN_SOURCE_BUILD AND NOT DART_BUILD_GUI_FILAMENT)
  if(DART_VERBOSE OR NOT DART_IN_SOURCE_BUILD)
    message(STATUS "Skipping ${example_name} example (requires DART_BUILD_GUI_FILAMENT=ON)")
  endif()
  return()
endif()

if(NOT DART_IN_SOURCE_BUILD)
  list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../../cmake")
  find_package(DART 7.0.0 QUIET COMPONENTS io utils-urdf gui-experimental imgui CONFIG)
  if(NOT DART_FOUND)
    find_package(DART 7.0.0 REQUIRED COMPONENTS io utils-urdf gui-experimental CONFIG)
  endif()
  find_package(Filament REQUIRED MODULE)
  find_package(glfw3 CONFIG REQUIRED)
  if(NOT TARGET dart-imgui-lib)
    find_package(imgui CONFIG QUIET)
  endif()
endif()

find_package(JPEG REQUIRED)
find_package(PNG REQUIRED)

if(TARGET imgui::imgui)
  set(imgui_target imgui::imgui)
elseif(TARGET dart-imgui-lib)
  set(imgui_target dart-imgui-lib)
else()
  message(FATAL_ERROR "No ImGui target is available for ${example_name}.")
endif()

if(NOT TARGET dart-gui-experimental)
  message(FATAL_ERROR "${example_name} requires the dart-gui-experimental target.")
endif()

include("${CMAKE_CURRENT_LIST_DIR}/../../dart/gui/experimental/detail/filament/filament_sources.cmake")

set(generated_dir "${CMAKE_CURRENT_BINARY_DIR}/generated")
dart_filament_gui_generate_material_headers(
  material_headers
  GENERATED_DIR "${generated_dir}"
)

file(GLOB srcs "*.cpp" "*.hpp")
add_executable(
  ${example_target}
  ${srcs}
  ${DART_FILAMENT_GUI_BACKEND_SRCS}
  ${DART_FILAMENT_GUI_BACKEND_HDRS}
  ${material_headers}
)
set_target_properties(${example_target} PROPERTIES OUTPUT_NAME ${example_name})
target_include_directories(${example_target} PRIVATE "${generated_dir}")
target_compile_definitions(
  ${example_target}
  PRIVATE DART_FILAMENT_GUI_REPOSITORY_ROOT="${DART_FILAMENT_GUI_SOURCE_DIR}"
)
target_link_libraries(
  ${example_target}
  PRIVATE
    dart
    dart-io
    dart-utils-urdf
    dart-gui-experimental
    Filament::filament
    JPEG::JPEG
    PNG::PNG
    glfw
    ${imgui_target}
)
target_compile_features(${example_target} PRIVATE cxx_std_20)

if(DART_IN_SOURCE_BUILD)
  set_target_properties(${example_target} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${DART_BINARY_DIR}/bin")
  dart_add_example(${example_target})
  dart_format_add(
    ${srcs}
    ${DART_FILAMENT_GUI_BACKEND_SRCS}
    ${DART_FILAMENT_GUI_BACKEND_HDRS}
  )

  if(BUILD_TESTING AND DART_BUILD_TESTS AND DART_ENABLE_FILAMENT_GUI_SMOKE_TESTS)
    dart_filament_gui_add_smoke_tests(
      ${example_target}
      BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}"
    )
  endif()
endif()
