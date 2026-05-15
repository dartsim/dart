# Copyright (c) 2011, The DART development contributors

set(DART_FILAMENT_GUI_BACKEND_DIR "${CMAKE_CURRENT_LIST_DIR}")
set(DART_FILAMENT_GUI_DETAIL_DIR "${DART_FILAMENT_GUI_BACKEND_DIR}/..")
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
  "${DART_FILAMENT_GUI_BACKEND_DIR}/scene_fixtures.cpp"
  "${DART_FILAMENT_GUI_BACKEND_DIR}/scene_requirements.cpp"
  "${DART_FILAMENT_GUI_BACKEND_DIR}/scene_startup.cpp"
  "${DART_FILAMENT_GUI_BACKEND_DIR}/scenes.cpp"
  "${DART_FILAMENT_GUI_BACKEND_DIR}/screenshot.cpp"
  "${DART_FILAMENT_GUI_BACKEND_DIR}/selection.cpp"
  "${DART_FILAMENT_GUI_BACKEND_DIR}/simulation_stepper.cpp"
  "${DART_FILAMENT_GUI_BACKEND_DIR}/textures.cpp"
  "${DART_FILAMENT_GUI_BACKEND_DIR}/ui_frame.cpp")

set(DART_FILAMENT_GUI_BACKEND_HDRS
  "${DART_FILAMENT_GUI_DETAIL_DIR}/application.hpp"
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
  "${DART_FILAMENT_GUI_BACKEND_DIR}/scene_fixtures.hpp"
  "${DART_FILAMENT_GUI_BACKEND_DIR}/scene_requirements.hpp"
  "${DART_FILAMENT_GUI_BACKEND_DIR}/scene_startup.hpp"
  "${DART_FILAMENT_GUI_BACKEND_DIR}/scenes.hpp"
  "${DART_FILAMENT_GUI_BACKEND_DIR}/screenshot.hpp"
  "${DART_FILAMENT_GUI_BACKEND_DIR}/selection.hpp"
  "${DART_FILAMENT_GUI_BACKEND_DIR}/simulation_stepper.hpp"
  "${DART_FILAMENT_GUI_BACKEND_DIR}/textures.hpp"
  "${DART_FILAMENT_GUI_BACKEND_DIR}/ui_frame.hpp")

function(_dart_filament_gui_find_dependencies out_imgui_target)
  if(NOT TARGET Filament::filament OR NOT TARGET Filament::matc)
    find_package(Filament REQUIRED MODULE)
  endif()
  if(NOT TARGET Filament::filament OR NOT TARGET Filament::matc)
    message(FATAL_ERROR "Filament::filament and Filament::matc are required")
  endif()

  if(NOT TARGET glfw)
    find_package(glfw3 CONFIG REQUIRED)
  endif()
  if(NOT TARGET glfw)
    message(FATAL_ERROR "glfw target is required")
  endif()

  find_package(JPEG REQUIRED)
  find_package(PNG REQUIRED)

  if(NOT TARGET dart-imgui-lib AND NOT TARGET imgui::imgui)
    find_package(imgui CONFIG QUIET)
  endif()

  if(TARGET imgui::imgui)
    set(_imgui_target imgui::imgui)
  elseif(TARGET dart-imgui-lib)
    set(_imgui_target dart-imgui-lib)
  else()
    message(FATAL_ERROR "No ImGui target is available for filament_gui")
  endif()

  set(${out_imgui_target} "${_imgui_target}" PARENT_SCOPE)
endfunction()

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

  set(hello_world_test_name EXAMPLE_filament_gui_hello_world_headless_smoke)
  add_test(
    NAME ${hello_world_test_name}
    COMMAND
      "${CMAKE_COMMAND}"
      "-DDART_FILAMENT_GUI_EXECUTABLE=$<TARGET_FILE:${example_target}>"
      "-DDART_FILAMENT_GUI_SCREENSHOT=${DART_FILAMENT_GUI_BINARY_DIR}/filament_gui_hello_world_headless_smoke.ppm"
      -DDART_FILAMENT_GUI_SCENE=hello-world
      -DDART_FILAMENT_GUI_WIDTH=640
      -DDART_FILAMENT_GUI_HEIGHT=480
      -DDART_FILAMENT_GUI_FRAMES=4
      -P "${DART_FILAMENT_GUI_TESTING_DIR}/run_headless_smoke.cmake"
  )
  _dart_filament_gui_apply_smoke_test_properties(${hello_world_test_name})

  set(boxes_test_name EXAMPLE_filament_gui_boxes_headless_smoke)
  add_test(
    NAME ${boxes_test_name}
    COMMAND
      "${CMAKE_COMMAND}"
      "-DDART_FILAMENT_GUI_EXECUTABLE=$<TARGET_FILE:${example_target}>"
      "-DDART_FILAMENT_GUI_SCREENSHOT=${DART_FILAMENT_GUI_BINARY_DIR}/filament_gui_boxes_headless_smoke.ppm"
      -DDART_FILAMENT_GUI_SCENE=boxes
      -DDART_FILAMENT_GUI_WIDTH=640
      -DDART_FILAMENT_GUI_HEIGHT=480
      -DDART_FILAMENT_GUI_FRAMES=4
      -P "${DART_FILAMENT_GUI_TESTING_DIR}/run_headless_smoke.cmake"
  )
  _dart_filament_gui_apply_smoke_test_properties(${boxes_test_name})

  set(hardcoded_design_test_name
      EXAMPLE_filament_gui_hardcoded_design_headless_smoke)
  add_test(
    NAME ${hardcoded_design_test_name}
    COMMAND
      "${CMAKE_COMMAND}"
      "-DDART_FILAMENT_GUI_EXECUTABLE=$<TARGET_FILE:${example_target}>"
      "-DDART_FILAMENT_GUI_SCREENSHOT=${DART_FILAMENT_GUI_BINARY_DIR}/filament_gui_hardcoded_design_headless_smoke.ppm"
      -DDART_FILAMENT_GUI_SCENE=hardcoded-design
      -DDART_FILAMENT_GUI_WIDTH=640
      -DDART_FILAMENT_GUI_HEIGHT=480
      -DDART_FILAMENT_GUI_FRAMES=4
      -P "${DART_FILAMENT_GUI_TESTING_DIR}/run_headless_smoke.cmake"
  )
  _dart_filament_gui_apply_smoke_test_properties(
    ${hardcoded_design_test_name})

  set(rigid_chain_test_name EXAMPLE_filament_gui_rigid_chain_headless_smoke)
  add_test(
    NAME ${rigid_chain_test_name}
    COMMAND
      "${CMAKE_COMMAND}"
      "-DDART_FILAMENT_GUI_EXECUTABLE=$<TARGET_FILE:${example_target}>"
      "-DDART_FILAMENT_GUI_SCREENSHOT=${DART_FILAMENT_GUI_BINARY_DIR}/filament_gui_rigid_chain_headless_smoke.ppm"
      -DDART_FILAMENT_GUI_SCENE=rigid-chain
      -DDART_FILAMENT_GUI_WIDTH=640
      -DDART_FILAMENT_GUI_HEIGHT=480
      -DDART_FILAMENT_GUI_FRAMES=4
      -P "${DART_FILAMENT_GUI_TESTING_DIR}/run_headless_smoke.cmake"
  )
  _dart_filament_gui_apply_smoke_test_properties(${rigid_chain_test_name})

  set(rigid_loop_test_name EXAMPLE_filament_gui_rigid_loop_headless_smoke)
  add_test(
    NAME ${rigid_loop_test_name}
    COMMAND
      "${CMAKE_COMMAND}"
      "-DDART_FILAMENT_GUI_EXECUTABLE=$<TARGET_FILE:${example_target}>"
      "-DDART_FILAMENT_GUI_SCREENSHOT=${DART_FILAMENT_GUI_BINARY_DIR}/filament_gui_rigid_loop_headless_smoke.ppm"
      -DDART_FILAMENT_GUI_SCENE=rigid-loop
      -DDART_FILAMENT_GUI_WIDTH=640
      -DDART_FILAMENT_GUI_HEIGHT=480
      -DDART_FILAMENT_GUI_FRAMES=4
      -P "${DART_FILAMENT_GUI_TESTING_DIR}/run_headless_smoke.cmake"
  )
  _dart_filament_gui_apply_smoke_test_properties(${rigid_loop_test_name})

  set(mixed_chain_test_name EXAMPLE_filament_gui_mixed_chain_headless_smoke)
  add_test(
    NAME ${mixed_chain_test_name}
    COMMAND
      "${CMAKE_COMMAND}"
      "-DDART_FILAMENT_GUI_EXECUTABLE=$<TARGET_FILE:${example_target}>"
      "-DDART_FILAMENT_GUI_SCREENSHOT=${DART_FILAMENT_GUI_BINARY_DIR}/filament_gui_mixed_chain_headless_smoke.ppm"
      -DDART_FILAMENT_GUI_SCENE=mixed-chain
      -DDART_FILAMENT_GUI_WIDTH=640
      -DDART_FILAMENT_GUI_HEIGHT=480
      -DDART_FILAMENT_GUI_FRAMES=4
      -P "${DART_FILAMENT_GUI_TESTING_DIR}/run_headless_smoke.cmake"
  )
  _dart_filament_gui_apply_smoke_test_properties(${mixed_chain_test_name})

  set(coupler_constraint_test_name
      EXAMPLE_filament_gui_coupler_constraint_headless_smoke)
  add_test(
    NAME ${coupler_constraint_test_name}
    COMMAND
      "${CMAKE_COMMAND}"
      "-DDART_FILAMENT_GUI_EXECUTABLE=$<TARGET_FILE:${example_target}>"
      "-DDART_FILAMENT_GUI_SCREENSHOT=${DART_FILAMENT_GUI_BINARY_DIR}/filament_gui_coupler_constraint_headless_smoke.ppm"
      -DDART_FILAMENT_GUI_SCENE=coupler-constraint
      -DDART_FILAMENT_GUI_WIDTH=640
      -DDART_FILAMENT_GUI_HEIGHT=480
      -DDART_FILAMENT_GUI_FRAMES=4
      -P "${DART_FILAMENT_GUI_TESTING_DIR}/run_headless_smoke.cmake"
  )
  _dart_filament_gui_apply_smoke_test_properties(${coupler_constraint_test_name})

  set(add_delete_skels_test_name
      EXAMPLE_filament_gui_add_delete_skels_headless_smoke)
  add_test(
    NAME ${add_delete_skels_test_name}
    COMMAND
      "${CMAKE_COMMAND}"
      "-DDART_FILAMENT_GUI_EXECUTABLE=$<TARGET_FILE:${example_target}>"
      "-DDART_FILAMENT_GUI_SCREENSHOT=${DART_FILAMENT_GUI_BINARY_DIR}/filament_gui_add_delete_skels_headless_smoke.ppm"
      -DDART_FILAMENT_GUI_SCENE=add-delete-skels
      -DDART_FILAMENT_GUI_WIDTH=640
      -DDART_FILAMENT_GUI_HEIGHT=480
      -DDART_FILAMENT_GUI_FRAMES=4
      -P "${DART_FILAMENT_GUI_TESTING_DIR}/run_headless_smoke.cmake"
  )
  _dart_filament_gui_apply_smoke_test_properties(${add_delete_skels_test_name})

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

  set(simple_frames_test_name EXAMPLE_filament_gui_simple_frames_headless_smoke)
  add_test(
    NAME ${simple_frames_test_name}
    COMMAND
      "${CMAKE_COMMAND}"
      "-DDART_FILAMENT_GUI_EXECUTABLE=$<TARGET_FILE:${example_target}>"
      "-DDART_FILAMENT_GUI_SCREENSHOT=${DART_FILAMENT_GUI_BINARY_DIR}/filament_gui_simple_frames_headless_smoke.ppm"
      -DDART_FILAMENT_GUI_SCENE=simple-frames
      -DDART_FILAMENT_GUI_WIDTH=640
      -DDART_FILAMENT_GUI_HEIGHT=480
      -DDART_FILAMENT_GUI_FRAMES=4
      -P "${DART_FILAMENT_GUI_TESTING_DIR}/run_headless_smoke.cmake"
  )
  _dart_filament_gui_apply_smoke_test_properties(${simple_frames_test_name})

  set(soft_bodies_test_name EXAMPLE_filament_gui_soft_bodies_headless_smoke)
  add_test(
    NAME ${soft_bodies_test_name}
    COMMAND
      "${CMAKE_COMMAND}"
      "-DDART_FILAMENT_GUI_EXECUTABLE=$<TARGET_FILE:${example_target}>"
      "-DDART_FILAMENT_GUI_SCREENSHOT=${DART_FILAMENT_GUI_BINARY_DIR}/filament_gui_soft_bodies_headless_smoke.ppm"
      -DDART_FILAMENT_GUI_SCENE=soft-bodies
      -DDART_FILAMENT_GUI_WIDTH=640
      -DDART_FILAMENT_GUI_HEIGHT=480
      -DDART_FILAMENT_GUI_FRAMES=4
      -P "${DART_FILAMENT_GUI_TESTING_DIR}/run_headless_smoke.cmake"
  )
  _dart_filament_gui_apply_smoke_test_properties(${soft_bodies_test_name})

  set(point_cloud_test_name EXAMPLE_filament_gui_point_cloud_headless_smoke)
  add_test(
    NAME ${point_cloud_test_name}
    COMMAND
      "${CMAKE_COMMAND}"
      "-DDART_FILAMENT_GUI_EXECUTABLE=$<TARGET_FILE:${example_target}>"
      "-DDART_FILAMENT_GUI_SCREENSHOT=${DART_FILAMENT_GUI_BINARY_DIR}/filament_gui_point_cloud_headless_smoke.ppm"
      -DDART_FILAMENT_GUI_SCENE=point-cloud
      -DDART_FILAMENT_GUI_WIDTH=640
      -DDART_FILAMENT_GUI_HEIGHT=480
      -DDART_FILAMENT_GUI_FRAMES=4
      -P "${DART_FILAMENT_GUI_TESTING_DIR}/run_headless_smoke.cmake"
  )
  _dart_filament_gui_apply_smoke_test_properties(${point_cloud_test_name})

  set(capsule_ground_contact_test_name
      EXAMPLE_filament_gui_capsule_ground_contact_headless_smoke)
  add_test(
    NAME ${capsule_ground_contact_test_name}
    COMMAND
      "${CMAKE_COMMAND}"
      "-DDART_FILAMENT_GUI_EXECUTABLE=$<TARGET_FILE:${example_target}>"
      "-DDART_FILAMENT_GUI_SCREENSHOT=${DART_FILAMENT_GUI_BINARY_DIR}/filament_gui_capsule_ground_contact_headless_smoke.ppm"
      -DDART_FILAMENT_GUI_SCENE=capsule-ground-contact
      -DDART_FILAMENT_GUI_WIDTH=640
      -DDART_FILAMENT_GUI_HEIGHT=480
      -DDART_FILAMENT_GUI_FRAMES=4
      -P "${DART_FILAMENT_GUI_TESTING_DIR}/run_headless_smoke.cmake"
  )
  _dart_filament_gui_apply_smoke_test_properties(
    ${capsule_ground_contact_test_name})

  set(simulation_event_handler_test_name
      EXAMPLE_filament_gui_simulation_event_handler_headless_smoke)
  add_test(
    NAME ${simulation_event_handler_test_name}
    COMMAND
      "${CMAKE_COMMAND}"
      "-DDART_FILAMENT_GUI_EXECUTABLE=$<TARGET_FILE:${example_target}>"
      "-DDART_FILAMENT_GUI_SCREENSHOT=${DART_FILAMENT_GUI_BINARY_DIR}/filament_gui_simulation_event_handler_headless_smoke.ppm"
      -DDART_FILAMENT_GUI_SCENE=simulation-event-handler
      -DDART_FILAMENT_GUI_WIDTH=640
      -DDART_FILAMENT_GUI_HEIGHT=480
      -DDART_FILAMENT_GUI_FRAMES=4
      -P "${DART_FILAMENT_GUI_TESTING_DIR}/run_headless_smoke.cmake"
  )
  _dart_filament_gui_apply_smoke_test_properties(
    ${simulation_event_handler_test_name})

  set(polyhedron_test_name EXAMPLE_filament_gui_polyhedron_headless_smoke)
  add_test(
    NAME ${polyhedron_test_name}
    COMMAND
      "${CMAKE_COMMAND}"
      "-DDART_FILAMENT_GUI_EXECUTABLE=$<TARGET_FILE:${example_target}>"
      "-DDART_FILAMENT_GUI_SCREENSHOT=${DART_FILAMENT_GUI_BINARY_DIR}/filament_gui_polyhedron_headless_smoke.ppm"
      -DDART_FILAMENT_GUI_SCENE=polyhedron
      -DDART_FILAMENT_GUI_WIDTH=640
      -DDART_FILAMENT_GUI_HEIGHT=480
      -DDART_FILAMENT_GUI_FRAMES=4
      -P "${DART_FILAMENT_GUI_TESTING_DIR}/run_headless_smoke.cmake"
  )
  _dart_filament_gui_apply_smoke_test_properties(${polyhedron_test_name})

  set(heightmap_test_name EXAMPLE_filament_gui_heightmap_headless_smoke)
  add_test(
    NAME ${heightmap_test_name}
    COMMAND
      "${CMAKE_COMMAND}"
      "-DDART_FILAMENT_GUI_EXECUTABLE=$<TARGET_FILE:${example_target}>"
      "-DDART_FILAMENT_GUI_SCREENSHOT=${DART_FILAMENT_GUI_BINARY_DIR}/filament_gui_heightmap_headless_smoke.ppm"
      -DDART_FILAMENT_GUI_SCENE=heightmap
      -DDART_FILAMENT_GUI_WIDTH=640
      -DDART_FILAMENT_GUI_HEIGHT=480
      -DDART_FILAMENT_GUI_FRAMES=4
      -P "${DART_FILAMENT_GUI_TESTING_DIR}/run_headless_smoke.cmake"
  )
  _dart_filament_gui_apply_smoke_test_properties(${heightmap_test_name})
endfunction()

function(dart_filament_gui_add_example example_target)
  cmake_parse_arguments(
    DART_FILAMENT_GUI_EXAMPLE
    ""
    "GENERATED_DIR;OUTPUT_NAME;SOURCE_DIR"
    ""
    ${ARGN}
  )
  if(NOT DART_FILAMENT_GUI_EXAMPLE_GENERATED_DIR)
    message(FATAL_ERROR "dart_filament_gui_add_example requires GENERATED_DIR")
  endif()
  if(NOT DART_FILAMENT_GUI_EXAMPLE_OUTPUT_NAME)
    message(FATAL_ERROR "dart_filament_gui_add_example requires OUTPUT_NAME")
  endif()
  if(NOT DART_FILAMENT_GUI_EXAMPLE_SOURCE_DIR)
    message(FATAL_ERROR "dart_filament_gui_add_example requires SOURCE_DIR")
  endif()
  if(NOT TARGET dart-gui-experimental)
    message(FATAL_ERROR "filament_gui requires the dart-gui-experimental target")
  endif()

  _dart_filament_gui_find_dependencies(_imgui_target)
  dart_filament_gui_generate_material_headers(
    _material_headers
    GENERATED_DIR "${DART_FILAMENT_GUI_EXAMPLE_GENERATED_DIR}"
  )

  file(
    GLOB
    _example_sources
    "${DART_FILAMENT_GUI_EXAMPLE_SOURCE_DIR}/*.cpp"
    "${DART_FILAMENT_GUI_EXAMPLE_SOURCE_DIR}/*.hpp"
  )

  add_executable(
    ${example_target}
    ${_example_sources}
    ${DART_FILAMENT_GUI_BACKEND_SRCS}
    ${DART_FILAMENT_GUI_BACKEND_HDRS}
    ${_material_headers}
  )
  set_target_properties(
    ${example_target}
    PROPERTIES OUTPUT_NAME "${DART_FILAMENT_GUI_EXAMPLE_OUTPUT_NAME}"
  )
  target_include_directories(
    ${example_target}
    PRIVATE "${DART_FILAMENT_GUI_EXAMPLE_GENERATED_DIR}"
  )
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
      ${_imgui_target}
  )
  target_compile_features(${example_target} PRIVATE cxx_std_20)

  if(DART_IN_SOURCE_BUILD)
    set_target_properties(
      ${example_target}
      PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${DART_BINARY_DIR}/bin"
    )
    dart_add_example(${example_target})
    dart_format_add(
      ${_example_sources}
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
endfunction()
