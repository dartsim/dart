# Copyright (c) 2011, The DART development contributors

set(DART_FILAMENT_GUI_BACKEND_DIR "${CMAKE_CURRENT_LIST_DIR}")
set(DART_FILAMENT_GUI_TESTING_DIR "${DART_FILAMENT_GUI_BACKEND_DIR}/testing")

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
