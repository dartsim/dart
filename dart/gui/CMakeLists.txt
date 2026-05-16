if(DART_BUILD_GUI OR DART_BUILD_GUI_FILAMENT)
  add_subdirectory(experimental)
endif()

if(NOT DART_BUILD_GUI)
  message(STATUS "Skipping GUI (DART_BUILD_GUI == ${DART_BUILD_GUI})")
  return()
endif()

set(target_name ${PROJECT_NAME}-gui)
set(component_name gui)

if(NOT TARGET dart-gui-core)
  message(FATAL_ERROR "dart-gui requires the dart-gui-core target")
endif()

set(
  dart_gui_hdrs
  all.hpp
  application.hpp
  debug.hpp
  export.hpp
  fwd.hpp
  geometry.hpp
  interaction.hpp
  panel.hpp
  profile.hpp
  renderable.hpp
  scene.hpp
  viewer.hpp
)
set(dart_gui_srcs gui.cpp)

dart_add_library(${target_name} ${dart_gui_hdrs} ${dart_gui_srcs})
target_link_libraries(${target_name} PUBLIC dart-gui-core)
target_include_directories(
  ${target_name}
  PUBLIC
    $<BUILD_INTERFACE:${DART_SOURCE_DIR}>
    $<BUILD_INTERFACE:${DART_BINARY_DIR}>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(${target_name} PUBLIC cxx_std_20)

add_component(${PROJECT_NAME} ${component_name})
add_component_targets(${PROJECT_NAME} ${component_name} ${target_name})
add_component_dependencies(${PROJECT_NAME} ${component_name} gui-core)

configure_file(
  "${CMAKE_CURRENT_LIST_DIR}/all.hpp"
  "${CMAKE_CURRENT_BINARY_DIR}/gui.hpp"
  COPYONLY
)
install(
  FILES
    all.hpp
    application.hpp
    debug.hpp
    export.hpp
    fwd.hpp
    geometry.hpp
    interaction.hpp
    panel.hpp
    profile.hpp
    renderable.hpp
    scene.hpp
    viewer.hpp
    "${CMAKE_CURRENT_BINARY_DIR}/gui.hpp"
  DESTINATION include/dart/gui
  COMPONENT headers
)

dart_format_add(${dart_gui_hdrs} ${dart_gui_srcs})
