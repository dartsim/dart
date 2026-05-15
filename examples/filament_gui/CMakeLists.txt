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

set(generated_dir "${CMAKE_CURRENT_BINARY_DIR}/generated")
set(material_bin "${generated_dir}/default_lit.filamat")
set(material_header "${generated_dir}/default_lit_material.hpp")
set(textured_material_bin "${generated_dir}/textured_lit.filamat")
set(textured_material_header "${generated_dir}/textured_lit_material.hpp")
set(transparent_material_bin "${generated_dir}/transparent_lit.filamat")
set(transparent_material_header "${generated_dir}/transparent_lit_material.hpp")
set(transparent_textured_material_bin "${generated_dir}/transparent_textured_lit.filamat")
set(transparent_textured_material_header "${generated_dir}/transparent_textured_lit_material.hpp")
set(debug_material_bin "${generated_dir}/debug_color.filamat")
set(debug_material_header "${generated_dir}/debug_color_material.hpp")
set(imgui_material_bin "${generated_dir}/imgui.filamat")
set(imgui_material_header "${generated_dir}/imgui_material.hpp")

add_custom_command(
  OUTPUT "${material_bin}"
  COMMAND "${CMAKE_COMMAND}" -E make_directory "${generated_dir}"
  COMMAND Filament::matc -a opengl -p desktop -o "${material_bin}"
          "${CMAKE_CURRENT_LIST_DIR}/materials/default_lit.mat"
  DEPENDS "${CMAKE_CURRENT_LIST_DIR}/materials/default_lit.mat" Filament::matc
  COMMENT "Compiling Filament material for ${example_name}"
  VERBATIM
)

add_custom_command(
  OUTPUT "${material_header}"
  COMMAND "${CMAKE_COMMAND}"
          "-DINPUT=${material_bin}"
          "-DOUTPUT=${material_header}"
          -DSYMBOL=kDefaultLitMaterial
          -DNAMESPACE=dart::gui::experimental::filament
          -P "${CMAKE_CURRENT_LIST_DIR}/../../cmake/dart_binary_to_header.cmake"
  DEPENDS "${material_bin}" "${CMAKE_CURRENT_LIST_DIR}/../../cmake/dart_binary_to_header.cmake"
  COMMENT "Embedding Filament material for ${example_name}"
  VERBATIM
)

add_custom_command(
  OUTPUT "${textured_material_bin}"
  COMMAND "${CMAKE_COMMAND}" -E make_directory "${generated_dir}"
  COMMAND Filament::matc -a opengl -p desktop -o "${textured_material_bin}"
          "${CMAKE_CURRENT_LIST_DIR}/materials/textured_lit.mat"
  DEPENDS "${CMAKE_CURRENT_LIST_DIR}/materials/textured_lit.mat" Filament::matc
  COMMENT "Compiling textured Filament material for ${example_name}"
  VERBATIM
)

add_custom_command(
  OUTPUT "${textured_material_header}"
  COMMAND "${CMAKE_COMMAND}"
          "-DINPUT=${textured_material_bin}"
          "-DOUTPUT=${textured_material_header}"
          -DSYMBOL=kTexturedLitMaterial
          -DNAMESPACE=dart::gui::experimental::filament
          -P "${CMAKE_CURRENT_LIST_DIR}/../../cmake/dart_binary_to_header.cmake"
  DEPENDS "${textured_material_bin}" "${CMAKE_CURRENT_LIST_DIR}/../../cmake/dart_binary_to_header.cmake"
  COMMENT "Embedding textured Filament material for ${example_name}"
  VERBATIM
)

add_custom_command(
  OUTPUT "${transparent_material_bin}"
  COMMAND "${CMAKE_COMMAND}" -E make_directory "${generated_dir}"
  COMMAND Filament::matc -a opengl -p desktop -o "${transparent_material_bin}"
          "${CMAKE_CURRENT_LIST_DIR}/materials/transparent_lit.mat"
  DEPENDS "${CMAKE_CURRENT_LIST_DIR}/materials/transparent_lit.mat" Filament::matc
  COMMENT "Compiling transparent Filament material for ${example_name}"
  VERBATIM
)

add_custom_command(
  OUTPUT "${transparent_material_header}"
  COMMAND "${CMAKE_COMMAND}"
          "-DINPUT=${transparent_material_bin}"
          "-DOUTPUT=${transparent_material_header}"
          -DSYMBOL=kTransparentLitMaterial
          -DNAMESPACE=dart::gui::experimental::filament
          -P "${CMAKE_CURRENT_LIST_DIR}/../../cmake/dart_binary_to_header.cmake"
  DEPENDS "${transparent_material_bin}" "${CMAKE_CURRENT_LIST_DIR}/../../cmake/dart_binary_to_header.cmake"
  COMMENT "Embedding transparent Filament material for ${example_name}"
  VERBATIM
)

add_custom_command(
  OUTPUT "${transparent_textured_material_bin}"
  COMMAND "${CMAKE_COMMAND}" -E make_directory "${generated_dir}"
  COMMAND Filament::matc -a opengl -p desktop -o "${transparent_textured_material_bin}"
          "${CMAKE_CURRENT_LIST_DIR}/materials/transparent_textured_lit.mat"
  DEPENDS "${CMAKE_CURRENT_LIST_DIR}/materials/transparent_textured_lit.mat" Filament::matc
  COMMENT "Compiling transparent textured Filament material for ${example_name}"
  VERBATIM
)

add_custom_command(
  OUTPUT "${transparent_textured_material_header}"
  COMMAND "${CMAKE_COMMAND}"
          "-DINPUT=${transparent_textured_material_bin}"
          "-DOUTPUT=${transparent_textured_material_header}"
          -DSYMBOL=kTransparentTexturedLitMaterial
          -DNAMESPACE=dart::gui::experimental::filament
          -P "${CMAKE_CURRENT_LIST_DIR}/../../cmake/dart_binary_to_header.cmake"
  DEPENDS "${transparent_textured_material_bin}" "${CMAKE_CURRENT_LIST_DIR}/../../cmake/dart_binary_to_header.cmake"
  COMMENT "Embedding transparent textured Filament material for ${example_name}"
  VERBATIM
)

add_custom_command(
  OUTPUT "${imgui_material_bin}"
  COMMAND "${CMAKE_COMMAND}" -E make_directory "${generated_dir}"
  COMMAND Filament::matc -a opengl -p desktop -o "${imgui_material_bin}"
          "${CMAKE_CURRENT_LIST_DIR}/materials/imgui.mat"
  DEPENDS "${CMAKE_CURRENT_LIST_DIR}/materials/imgui.mat" Filament::matc
  COMMENT "Compiling Filament ImGui material for ${example_name}"
  VERBATIM
)

add_custom_command(
  OUTPUT "${debug_material_bin}"
  COMMAND "${CMAKE_COMMAND}" -E make_directory "${generated_dir}"
  COMMAND Filament::matc -a opengl -p desktop -o "${debug_material_bin}"
          "${CMAKE_CURRENT_LIST_DIR}/materials/debug_color.mat"
  DEPENDS "${CMAKE_CURRENT_LIST_DIR}/materials/debug_color.mat" Filament::matc
  COMMENT "Compiling debug Filament material for ${example_name}"
  VERBATIM
)

add_custom_command(
  OUTPUT "${debug_material_header}"
  COMMAND "${CMAKE_COMMAND}"
          "-DINPUT=${debug_material_bin}"
          "-DOUTPUT=${debug_material_header}"
          -DSYMBOL=kDebugColorMaterial
          -DNAMESPACE=dart::gui::experimental::filament
          -P "${CMAKE_CURRENT_LIST_DIR}/../../cmake/dart_binary_to_header.cmake"
  DEPENDS "${debug_material_bin}" "${CMAKE_CURRENT_LIST_DIR}/../../cmake/dart_binary_to_header.cmake"
  COMMENT "Embedding debug Filament material for ${example_name}"
  VERBATIM
)

add_custom_command(
  OUTPUT "${imgui_material_header}"
  COMMAND "${CMAKE_COMMAND}"
          "-DINPUT=${imgui_material_bin}"
          "-DOUTPUT=${imgui_material_header}"
          -DSYMBOL=kImGuiMaterial
          -DNAMESPACE=dart::gui::experimental::filament
          -P "${CMAKE_CURRENT_LIST_DIR}/../../cmake/dart_binary_to_header.cmake"
  DEPENDS "${imgui_material_bin}" "${CMAKE_CURRENT_LIST_DIR}/../../cmake/dart_binary_to_header.cmake"
  COMMENT "Embedding Filament ImGui material for ${example_name}"
  VERBATIM
)

file(GLOB srcs "*.cpp" "*.hpp")
set(filament_gui_backend_srcs
  "${CMAKE_CURRENT_LIST_DIR}/../../dart/gui/experimental/detail/filament/application.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/../../dart/gui/experimental/detail/filament/debug_overlay.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/../../dart/gui/experimental/detail/filament/imgui_overlay.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/../../dart/gui/experimental/detail/filament/input.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/../../dart/gui/experimental/detail/filament/native_window.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/../../dart/gui/experimental/detail/filament/panel.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/../../dart/gui/experimental/detail/filament/renderable_factory.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/../../dart/gui/experimental/detail/filament/renderable_resources.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/../../dart/gui/experimental/detail/filament/renderable_sync.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/../../dart/gui/experimental/detail/filament/render_context.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/../../dart/gui/experimental/detail/filament/render_environment.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/../../dart/gui/experimental/detail/filament/scenes.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/../../dart/gui/experimental/detail/filament/scene_requirements.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/../../dart/gui/experimental/detail/filament/screenshot.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/../../dart/gui/experimental/detail/filament/selection.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/../../dart/gui/experimental/detail/filament/textures.cpp"
)
set(filament_gui_backend_hdrs
  "${CMAKE_CURRENT_LIST_DIR}/../../dart/gui/experimental/detail/filament/application.hpp"
  "${CMAKE_CURRENT_LIST_DIR}/../../dart/gui/experimental/detail/filament/debug_overlay.hpp"
  "${CMAKE_CURRENT_LIST_DIR}/../../dart/gui/experimental/detail/filament/imgui_overlay.hpp"
  "${CMAKE_CURRENT_LIST_DIR}/../../dart/gui/experimental/detail/filament/input.hpp"
  "${CMAKE_CURRENT_LIST_DIR}/../../dart/gui/experimental/detail/filament/native_window.hpp"
  "${CMAKE_CURRENT_LIST_DIR}/../../dart/gui/experimental/detail/filament/panel.hpp"
  "${CMAKE_CURRENT_LIST_DIR}/../../dart/gui/experimental/detail/filament/renderable_factory.hpp"
  "${CMAKE_CURRENT_LIST_DIR}/../../dart/gui/experimental/detail/filament/renderable_resources.hpp"
  "${CMAKE_CURRENT_LIST_DIR}/../../dart/gui/experimental/detail/filament/renderable_sync.hpp"
  "${CMAKE_CURRENT_LIST_DIR}/../../dart/gui/experimental/detail/filament/render_context.hpp"
  "${CMAKE_CURRENT_LIST_DIR}/../../dart/gui/experimental/detail/filament/render_environment.hpp"
  "${CMAKE_CURRENT_LIST_DIR}/../../dart/gui/experimental/detail/filament/scenes.hpp"
  "${CMAKE_CURRENT_LIST_DIR}/../../dart/gui/experimental/detail/filament/scene_requirements.hpp"
  "${CMAKE_CURRENT_LIST_DIR}/../../dart/gui/experimental/detail/filament/screenshot.hpp"
  "${CMAKE_CURRENT_LIST_DIR}/../../dart/gui/experimental/detail/filament/selection.hpp"
  "${CMAKE_CURRENT_LIST_DIR}/../../dart/gui/experimental/detail/filament/textures.hpp"
)
add_executable(
  ${example_target}
  ${srcs}
  ${filament_gui_backend_srcs}
  ${filament_gui_backend_hdrs}
  "${material_header}"
  "${textured_material_header}"
  "${transparent_material_header}"
  "${transparent_textured_material_header}"
  "${debug_material_header}"
  "${imgui_material_header}"
)
set_target_properties(${example_target} PROPERTIES OUTPUT_NAME ${example_name})
target_include_directories(${example_target} PRIVATE "${generated_dir}")
target_compile_definitions(
  ${example_target}
  PRIVATE DART_FILAMENT_GUI_REPOSITORY_ROOT="${CMAKE_CURRENT_LIST_DIR}/../.."
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
    ${filament_gui_backend_srcs}
    ${filament_gui_backend_hdrs}
  )

  if(BUILD_TESTING AND DART_BUILD_TESTS AND DART_ENABLE_FILAMENT_GUI_SMOKE_TESTS)
    find_package(Python3 COMPONENTS Interpreter REQUIRED)
    set(test_name EXAMPLE_filament_gui_headless_smoke)
    add_test(
      NAME ${test_name}
      COMMAND
        "${CMAKE_COMMAND}"
        "-DDART_FILAMENT_GUI_EXECUTABLE=$<TARGET_FILE:${example_target}>"
        "-DDART_FILAMENT_GUI_SCREENSHOT=${CMAKE_CURRENT_BINARY_DIR}/filament_gui_headless_smoke.ppm"
        -DDART_FILAMENT_GUI_WIDTH=640
        -DDART_FILAMENT_GUI_HEIGHT=480
        -DDART_FILAMENT_GUI_FRAMES=10
        "-DDART_FILAMENT_GUI_PYTHON=${Python3_EXECUTABLE}"
        "-DDART_FILAMENT_GUI_ANALYZER=${CMAKE_CURRENT_LIST_DIR}/analyze_headless_smoke.py"
        -P "${CMAKE_CURRENT_LIST_DIR}/run_headless_smoke.cmake"
    )
    set_tests_properties(
      ${test_name}
      PROPERTIES LABELS "filament;gui;smoke;graphics"
    )
    if(UNIX AND NOT APPLE)
      set_tests_properties(
        ${test_name}
        PROPERTIES ENVIRONMENT "LIBGL_ALWAYS_SOFTWARE=1;MESA_LOADER_DRIVER_OVERRIDE=llvmpipe"
      )
    endif()

    set(drag_test_name EXAMPLE_filament_gui_drag_and_drop_headless_smoke)
    add_test(
      NAME ${drag_test_name}
      COMMAND
        "${CMAKE_COMMAND}"
        "-DDART_FILAMENT_GUI_EXECUTABLE=$<TARGET_FILE:${example_target}>"
        "-DDART_FILAMENT_GUI_SCREENSHOT=${CMAKE_CURRENT_BINARY_DIR}/filament_gui_drag_and_drop_headless_smoke.ppm"
        -DDART_FILAMENT_GUI_SCENE=drag-and-drop
        -DDART_FILAMENT_GUI_WIDTH=640
        -DDART_FILAMENT_GUI_HEIGHT=480
        -DDART_FILAMENT_GUI_FRAMES=4
        -P "${CMAKE_CURRENT_LIST_DIR}/run_headless_smoke.cmake"
    )
    set_tests_properties(
      ${drag_test_name}
      PROPERTIES LABELS "filament;gui;smoke;graphics"
    )
    if(UNIX AND NOT APPLE)
      set_tests_properties(
        ${drag_test_name}
        PROPERTIES ENVIRONMENT "LIBGL_ALWAYS_SOFTWARE=1;MESA_LOADER_DRIVER_OVERRIDE=llvmpipe"
      )
    endif()
  endif()
endif()
