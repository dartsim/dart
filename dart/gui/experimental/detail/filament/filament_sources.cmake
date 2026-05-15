# Copyright (c) 2011, The DART development contributors

set(DART_FILAMENT_GUI_BACKEND_DIR "${CMAKE_CURRENT_LIST_DIR}")
set(DART_FILAMENT_GUI_TESTING_DIR "${DART_FILAMENT_GUI_BACKEND_DIR}/testing")
get_filename_component(
  DART_FILAMENT_GUI_SOURCE_DIR
  "${DART_FILAMENT_GUI_BACKEND_DIR}/../../../../.."
  ABSOLUTE
)
set(DART_FILAMENT_GUI_BINARY_TO_HEADER
  "${DART_FILAMENT_GUI_SOURCE_DIR}/cmake/dart_binary_to_header.cmake")

set(DART_FILAMENT_GUI_DEFAULT_LIT_MATERIAL
  "${DART_FILAMENT_GUI_BACKEND_DIR}/materials/default_lit.mat")
set(DART_FILAMENT_GUI_TEXTURED_LIT_MATERIAL
  "${DART_FILAMENT_GUI_BACKEND_DIR}/materials/textured_lit.mat")
set(DART_FILAMENT_GUI_TRANSPARENT_LIT_MATERIAL
  "${DART_FILAMENT_GUI_BACKEND_DIR}/materials/transparent_lit.mat")
set(DART_FILAMENT_GUI_TRANSPARENT_TEXTURED_LIT_MATERIAL
  "${DART_FILAMENT_GUI_BACKEND_DIR}/materials/transparent_textured_lit.mat")
set(DART_FILAMENT_GUI_DEBUG_COLOR_MATERIAL
  "${DART_FILAMENT_GUI_BACKEND_DIR}/materials/debug_color.mat")
set(DART_FILAMENT_GUI_IMGUI_MATERIAL
  "${DART_FILAMENT_GUI_BACKEND_DIR}/materials/imgui.mat")

set(DART_FILAMENT_GUI_BACKEND_SRCS
  "${DART_FILAMENT_GUI_BACKEND_DIR}/application.cpp"
  "${DART_FILAMENT_GUI_BACKEND_DIR}/application_teardown.cpp"
  "${DART_FILAMENT_GUI_BACKEND_DIR}/debug_overlay.cpp"
  "${DART_FILAMENT_GUI_BACKEND_DIR}/frame_renderer.cpp"
  "${DART_FILAMENT_GUI_BACKEND_DIR}/frame_viewport.cpp"
  "${DART_FILAMENT_GUI_BACKEND_DIR}/imgui_overlay.cpp"
  "${DART_FILAMENT_GUI_BACKEND_DIR}/input.cpp"
  "${DART_FILAMENT_GUI_BACKEND_DIR}/native_window.cpp"
  "${DART_FILAMENT_GUI_BACKEND_DIR}/panel.cpp"
  "${DART_FILAMENT_GUI_BACKEND_DIR}/render_context.cpp"
  "${DART_FILAMENT_GUI_BACKEND_DIR}/render_environment.cpp"
  "${DART_FILAMENT_GUI_BACKEND_DIR}/renderable_factory.cpp"
  "${DART_FILAMENT_GUI_BACKEND_DIR}/renderable_resources.cpp"
  "${DART_FILAMENT_GUI_BACKEND_DIR}/renderable_sync.cpp"
  "${DART_FILAMENT_GUI_BACKEND_DIR}/scene_frame.cpp"
  "${DART_FILAMENT_GUI_BACKEND_DIR}/scene_requirements.cpp"
  "${DART_FILAMENT_GUI_BACKEND_DIR}/scene_startup.cpp"
  "${DART_FILAMENT_GUI_BACKEND_DIR}/scenes.cpp"
  "${DART_FILAMENT_GUI_BACKEND_DIR}/screenshot.cpp"
  "${DART_FILAMENT_GUI_BACKEND_DIR}/selection.cpp"
  "${DART_FILAMENT_GUI_BACKEND_DIR}/simulation_stepper.cpp"
  "${DART_FILAMENT_GUI_BACKEND_DIR}/textures.cpp"
  "${DART_FILAMENT_GUI_BACKEND_DIR}/ui_frame.cpp")

set(DART_FILAMENT_GUI_BACKEND_HDRS
  "${DART_FILAMENT_GUI_BACKEND_DIR}/application.hpp"
  "${DART_FILAMENT_GUI_BACKEND_DIR}/application_teardown.hpp"
  "${DART_FILAMENT_GUI_BACKEND_DIR}/debug_overlay.hpp"
  "${DART_FILAMENT_GUI_BACKEND_DIR}/frame_renderer.hpp"
  "${DART_FILAMENT_GUI_BACKEND_DIR}/frame_viewport.hpp"
  "${DART_FILAMENT_GUI_BACKEND_DIR}/imgui_overlay.hpp"
  "${DART_FILAMENT_GUI_BACKEND_DIR}/input.hpp"
  "${DART_FILAMENT_GUI_BACKEND_DIR}/native_window.hpp"
  "${DART_FILAMENT_GUI_BACKEND_DIR}/panel.hpp"
  "${DART_FILAMENT_GUI_BACKEND_DIR}/render_context.hpp"
  "${DART_FILAMENT_GUI_BACKEND_DIR}/render_environment.hpp"
  "${DART_FILAMENT_GUI_BACKEND_DIR}/renderable_factory.hpp"
  "${DART_FILAMENT_GUI_BACKEND_DIR}/renderable_resources.hpp"
  "${DART_FILAMENT_GUI_BACKEND_DIR}/renderable_sync.hpp"
  "${DART_FILAMENT_GUI_BACKEND_DIR}/scene_frame.hpp"
  "${DART_FILAMENT_GUI_BACKEND_DIR}/scene_requirements.hpp"
  "${DART_FILAMENT_GUI_BACKEND_DIR}/scene_startup.hpp"
  "${DART_FILAMENT_GUI_BACKEND_DIR}/scenes.hpp"
  "${DART_FILAMENT_GUI_BACKEND_DIR}/screenshot.hpp"
  "${DART_FILAMENT_GUI_BACKEND_DIR}/selection.hpp"
  "${DART_FILAMENT_GUI_BACKEND_DIR}/simulation_stepper.hpp"
  "${DART_FILAMENT_GUI_BACKEND_DIR}/textures.hpp"
  "${DART_FILAMENT_GUI_BACKEND_DIR}/ui_frame.hpp")

function(
  _dart_filament_gui_add_material_header
  out_headers
  generated_dir
  material_name
  material_source
  symbol
  material_label
)
  set(_material_bin "${generated_dir}/${material_name}.filamat")
  set(_material_header "${generated_dir}/${material_name}_material.hpp")

  add_custom_command(
    OUTPUT "${_material_bin}"
    COMMAND "${CMAKE_COMMAND}" -E make_directory "${generated_dir}"
    COMMAND Filament::matc -a opengl -p desktop -o "${_material_bin}"
            "${material_source}"
    DEPENDS "${material_source}" Filament::matc
    COMMENT "Compiling ${material_label} Filament material"
    VERBATIM
  )

  add_custom_command(
    OUTPUT "${_material_header}"
    COMMAND "${CMAKE_COMMAND}"
            "-DINPUT=${_material_bin}"
            "-DOUTPUT=${_material_header}"
            "-DSYMBOL=${symbol}"
            -DNAMESPACE=dart::gui::experimental::filament
            -P "${DART_FILAMENT_GUI_BINARY_TO_HEADER}"
    DEPENDS "${_material_bin}" "${DART_FILAMENT_GUI_BINARY_TO_HEADER}"
    COMMENT "Embedding ${material_label} Filament material"
    VERBATIM
  )

  set(${out_headers} ${${out_headers}} "${_material_header}" PARENT_SCOPE)
endfunction()

function(dart_filament_gui_generate_material_headers out_headers)
  cmake_parse_arguments(DART_FILAMENT_GUI "" "GENERATED_DIR" "" ${ARGN})
  if(NOT DART_FILAMENT_GUI_GENERATED_DIR)
    message(
      FATAL_ERROR
      "dart_filament_gui_generate_material_headers requires GENERATED_DIR"
    )
  endif()

  set(_headers)
  _dart_filament_gui_add_material_header(
    _headers
    "${DART_FILAMENT_GUI_GENERATED_DIR}"
    default_lit
    "${DART_FILAMENT_GUI_DEFAULT_LIT_MATERIAL}"
    kDefaultLitMaterial
    "default lit"
  )
  _dart_filament_gui_add_material_header(
    _headers
    "${DART_FILAMENT_GUI_GENERATED_DIR}"
    textured_lit
    "${DART_FILAMENT_GUI_TEXTURED_LIT_MATERIAL}"
    kTexturedLitMaterial
    "textured lit"
  )
  _dart_filament_gui_add_material_header(
    _headers
    "${DART_FILAMENT_GUI_GENERATED_DIR}"
    transparent_lit
    "${DART_FILAMENT_GUI_TRANSPARENT_LIT_MATERIAL}"
    kTransparentLitMaterial
    "transparent lit"
  )
  _dart_filament_gui_add_material_header(
    _headers
    "${DART_FILAMENT_GUI_GENERATED_DIR}"
    transparent_textured_lit
    "${DART_FILAMENT_GUI_TRANSPARENT_TEXTURED_LIT_MATERIAL}"
    kTransparentTexturedLitMaterial
    "transparent textured lit"
  )
  _dart_filament_gui_add_material_header(
    _headers
    "${DART_FILAMENT_GUI_GENERATED_DIR}"
    debug_color
    "${DART_FILAMENT_GUI_DEBUG_COLOR_MATERIAL}"
    kDebugColorMaterial
    "debug color"
  )
  _dart_filament_gui_add_material_header(
    _headers
    "${DART_FILAMENT_GUI_GENERATED_DIR}"
    imgui
    "${DART_FILAMENT_GUI_IMGUI_MATERIAL}"
    kImGuiMaterial
    "ImGui"
  )

  set(${out_headers} ${_headers} PARENT_SCOPE)
endfunction()

function(_dart_filament_gui_apply_smoke_test_properties test_name)
  set_tests_properties(
    ${test_name}
    PROPERTIES LABELS "filament;gui;smoke;graphics"
  )
  if(UNIX AND NOT APPLE)
    set_tests_properties(
      ${test_name}
      PROPERTIES ENVIRONMENT
                 "LIBGL_ALWAYS_SOFTWARE=1;MESA_LOADER_DRIVER_OVERRIDE=llvmpipe"
    )
  endif()
endfunction()

function(dart_filament_gui_add_smoke_tests example_target)
  cmake_parse_arguments(DART_FILAMENT_GUI "" "BINARY_DIR" "" ${ARGN})
  if(NOT DART_FILAMENT_GUI_BINARY_DIR)
    set(DART_FILAMENT_GUI_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}")
  endif()

  find_package(Python3 COMPONENTS Interpreter REQUIRED)
  set(test_name EXAMPLE_filament_gui_headless_smoke)
  add_test(
    NAME ${test_name}
    COMMAND
      "${CMAKE_COMMAND}"
      "-DDART_FILAMENT_GUI_EXECUTABLE=$<TARGET_FILE:${example_target}>"
      "-DDART_FILAMENT_GUI_SCREENSHOT=${DART_FILAMENT_GUI_BINARY_DIR}/filament_gui_headless_smoke.ppm"
      -DDART_FILAMENT_GUI_WIDTH=640
      -DDART_FILAMENT_GUI_HEIGHT=480
      -DDART_FILAMENT_GUI_FRAMES=10
      "-DDART_FILAMENT_GUI_PYTHON=${Python3_EXECUTABLE}"
      "-DDART_FILAMENT_GUI_ANALYZER=${DART_FILAMENT_GUI_TESTING_DIR}/analyze_headless_smoke.py"
      -P "${DART_FILAMENT_GUI_TESTING_DIR}/run_headless_smoke.cmake"
  )
  _dart_filament_gui_apply_smoke_test_properties(${test_name})

  set(drag_test_name EXAMPLE_filament_gui_drag_and_drop_headless_smoke)
  add_test(
    NAME ${drag_test_name}
    COMMAND
      "${CMAKE_COMMAND}"
      "-DDART_FILAMENT_GUI_EXECUTABLE=$<TARGET_FILE:${example_target}>"
      "-DDART_FILAMENT_GUI_SCREENSHOT=${DART_FILAMENT_GUI_BINARY_DIR}/filament_gui_drag_and_drop_headless_smoke.ppm"
      -DDART_FILAMENT_GUI_SCENE=drag-and-drop
      -DDART_FILAMENT_GUI_WIDTH=640
      -DDART_FILAMENT_GUI_HEIGHT=480
      -DDART_FILAMENT_GUI_FRAMES=4
      -P "${DART_FILAMENT_GUI_TESTING_DIR}/run_headless_smoke.cmake"
  )
  _dart_filament_gui_apply_smoke_test_properties(${drag_test_name})
endfunction()
