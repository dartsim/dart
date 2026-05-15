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

set(DART_FILAMENT_GUI_SMOKE_SCENE_PAIRS
    hello_world hello-world
    boxes boxes
    hardcoded_design hardcoded-design
    rigid_chain rigid-chain
    rigid_loop rigid-loop
    mixed_chain mixed-chain
    coupler_constraint coupler-constraint
    add_delete_skels add-delete-skels
    vehicle vehicle
    hybrid_dynamics hybrid-dynamics
    joint_constraints joint-constraints
    free_joint_cases free-joint-cases
    human_joint_limits human-joint-limits
    lcp_physics lcp-physics
    mimic_pendulums mimic-pendulums
    atlas_puppet atlas-puppet
    hubo_puppet hubo-puppet
    atlas_simbicon atlas-simbicon
    operational_space_control operational-space-control
    wam_ikfast wam-ikfast
    fetch fetch
    tinkertoy tinkertoy
    drag_and_drop drag-and-drop
    simple_frames simple-frames
    soft_bodies soft-bodies
    point_cloud point-cloud
    capsule_ground_contact capsule-ground-contact
    simulation_event_handler simulation-event-handler
    polyhedron polyhedron
    heightmap heightmap
)

function(_dart_filament_gui_smoke_test_name out_name scene_suffix)
  set(
    ${out_name}
    "EXAMPLE_filament_gui_${scene_suffix}_headless_smoke"
    PARENT_SCOPE
  )
endfunction()

function(_dart_filament_gui_add_headless_smoke_test test_name example_target)
  cmake_parse_arguments(
    DART_FILAMENT_GUI_SMOKE
    "ANALYZE"
    "ANALYSIS_MODE;BINARY_DIR;FRAMES;SCENE"
    ""
    ${ARGN}
  )
  if(NOT DART_FILAMENT_GUI_SMOKE_BINARY_DIR)
    set(DART_FILAMENT_GUI_SMOKE_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}")
  endif()
  if(NOT DART_FILAMENT_GUI_SMOKE_FRAMES)
    set(DART_FILAMENT_GUI_SMOKE_FRAMES 4)
  endif()

  string(REGEX REPLACE "^EXAMPLE_" "" _screenshot_stem "${test_name}")
  set(
    _command
    "${CMAKE_COMMAND}"
    "-DDART_FILAMENT_GUI_EXECUTABLE=$<TARGET_FILE:${example_target}>"
    "-DDART_FILAMENT_GUI_SCREENSHOT=${DART_FILAMENT_GUI_SMOKE_BINARY_DIR}/${_screenshot_stem}.ppm"
    -DDART_FILAMENT_GUI_WIDTH=640
    -DDART_FILAMENT_GUI_HEIGHT=480
    "-DDART_FILAMENT_GUI_FRAMES=${DART_FILAMENT_GUI_SMOKE_FRAMES}"
  )

  if(DART_FILAMENT_GUI_SMOKE_SCENE)
    list(
      APPEND
      _command
      "-DDART_FILAMENT_GUI_SCENE=${DART_FILAMENT_GUI_SMOKE_SCENE}"
    )
  endif()
  if(DART_FILAMENT_GUI_SMOKE_ANALYZE)
    if(NOT DART_FILAMENT_GUI_SMOKE_ANALYSIS_MODE)
      set(DART_FILAMENT_GUI_SMOKE_ANALYSIS_MODE contrast)
    endif()
    if(NOT DART_FILAMENT_GUI_SMOKE_ANALYSIS_MODE MATCHES "^(basic|contrast)$")
      message(
        FATAL_ERROR
        "ANALYSIS_MODE must be 'basic' or 'contrast' for ${test_name}"
      )
    endif()
    list(
      APPEND
      _command
      "-DDART_FILAMENT_GUI_PYTHON=${Python3_EXECUTABLE}"
      "-DDART_FILAMENT_GUI_ANALYZER=${DART_FILAMENT_GUI_TESTING_DIR}/analyze_headless_smoke.py"
      "-DDART_FILAMENT_GUI_ANALYSIS_MODE=${DART_FILAMENT_GUI_SMOKE_ANALYSIS_MODE}"
    )
  endif()

  list(
    APPEND
    _command
    -P
    "${DART_FILAMENT_GUI_TESTING_DIR}/run_headless_smoke.cmake"
  )
  add_test(NAME ${test_name} COMMAND ${_command})
  _dart_filament_gui_apply_smoke_test_properties(${test_name})
endfunction()

function(dart_filament_gui_add_smoke_tests example_target)
  cmake_parse_arguments(DART_FILAMENT_GUI "" "BINARY_DIR" "" ${ARGN})
  if(NOT DART_FILAMENT_GUI_BINARY_DIR)
    set(DART_FILAMENT_GUI_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}")
  endif()

  find_package(Python3 COMPONENTS Interpreter REQUIRED)
  _dart_filament_gui_add_headless_smoke_test(
    EXAMPLE_filament_gui_headless_smoke
    ${example_target}
    BINARY_DIR "${DART_FILAMENT_GUI_BINARY_DIR}"
    FRAMES 10
    ANALYZE
    ANALYSIS_MODE contrast
  )

  list(LENGTH DART_FILAMENT_GUI_SMOKE_SCENE_PAIRS _smoke_pair_count)
  math(EXPR _smoke_last_index "${_smoke_pair_count} - 1")
  foreach(_suffix_index RANGE 0 ${_smoke_last_index} 2)
    math(EXPR _scene_index "${_suffix_index} + 1")
    list(
      GET
      DART_FILAMENT_GUI_SMOKE_SCENE_PAIRS
      ${_suffix_index}
      _scene_suffix
    )
    list(GET DART_FILAMENT_GUI_SMOKE_SCENE_PAIRS ${_scene_index} _scene_name)
    _dart_filament_gui_smoke_test_name(_test_name "${_scene_suffix}")
    _dart_filament_gui_add_headless_smoke_test(
      ${_test_name}
      ${example_target}
      BINARY_DIR "${DART_FILAMENT_GUI_BINARY_DIR}"
      SCENE "${_scene_name}"
      ANALYZE
      ANALYSIS_MODE basic
    )
  endforeach()
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

  set(_example_main "${DART_FILAMENT_GUI_EXAMPLE_SOURCE_DIR}/main.cpp")
  if(NOT EXISTS "${_example_main}")
    message(FATAL_ERROR "filament_gui requires ${_example_main}")
  endif()
  file(READ "${_example_main}" _example_main_contents)
  string(
    FIND
    "${_example_main_contents}"
    "#include <filament/"
    _filament_angle_include
  )
  string(
    FIND
    "${_example_main_contents}"
    "#include \"filament/"
    _filament_quote_include
  )
  if(
    NOT _filament_angle_include EQUAL -1
    OR NOT _filament_quote_include EQUAL -1
  )
    message(
      FATAL_ERROR
      "filament_gui entry point must not include Filament headers directly: "
      "${_example_main}"
    )
  endif()

  file(
    GLOB
    _example_local_sources
    CONFIGURE_DEPENDS
    "${DART_FILAMENT_GUI_EXAMPLE_SOURCE_DIR}/*.cpp"
    "${DART_FILAMENT_GUI_EXAMPLE_SOURCE_DIR}/*.hpp"
  )
  set(_unexpected_example_local_sources ${_example_local_sources})
  list(REMOVE_ITEM _unexpected_example_local_sources "${_example_main}")
  if(_unexpected_example_local_sources)
    list(
      JOIN
      _unexpected_example_local_sources
      "\n  "
      _unexpected_example_local_sources_message
    )
    message(
      FATAL_ERROR
      "filament_gui keeps reusable renderer code under dart/gui; unexpected "
      "example-local C++ source/header files:\n  "
      "${_unexpected_example_local_sources_message}"
    )
  endif()

  file(
    GLOB_RECURSE
    _example_tree_files
    LIST_DIRECTORIES false
    CONFIGURE_DEPENDS
    "${DART_FILAMENT_GUI_EXAMPLE_SOURCE_DIR}/*"
  )
  set(_unexpected_example_tree_files ${_example_tree_files})
  list(
    REMOVE_ITEM
    _unexpected_example_tree_files
    "${DART_FILAMENT_GUI_EXAMPLE_SOURCE_DIR}/CMakeLists.txt"
    "${DART_FILAMENT_GUI_EXAMPLE_SOURCE_DIR}/README.md"
    "${_example_main}"
  )
  if(_unexpected_example_tree_files)
    list(
      JOIN
      _unexpected_example_tree_files
      "\n  "
      _unexpected_example_tree_files_message
    )
    message(
      FATAL_ERROR
      "filament_gui must stay a minimal DART GUI entry point; unexpected "
      "example-tree regular files:\n  "
      "${_unexpected_example_tree_files_message}"
    )
  endif()

  add_executable(
    ${example_target}
    "${_example_main}"
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
      "${_example_main}"
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
