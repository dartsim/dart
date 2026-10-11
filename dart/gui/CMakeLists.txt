# dart-gui combines the Filament GUI and ImGui and depends directly on core.
file(
  GLOB _gui_format_files
  CONFIGURE_DEPENDS
  "*.hpp"
  "*.cpp"
  "detail/*.hpp"
  "detail/*.cpp"
  "detail/*.mm"
)
dart_format_add(${_gui_format_files})

include(DARTFilamentProvider)
include(DARTFilamentMaterials)
include(DARTFilamentPlatform)

set(
  _filament_sources
  Viewer.cpp
  detail/FilamentRenderer.cpp
  detail/render_context.cpp
  detail/render_environment.cpp
  detail/renderable_factory.cpp
  detail/renderable_resources.cpp
  detail/renderable_sync.cpp
  detail/textures.cpp
  detail/image_decoders.cpp
  detail/screenshot.cpp
  detail/imgui_overlay.cpp
  detail/renderable.cpp
  detail/shape_descriptions.cpp
  detail/geometry.cpp
  detail/viewer_types.cpp
  detail/debug.cpp
  detail/interaction.cpp
  detail/overlay_adapter.cpp
)
if(APPLE)
  enable_language(OBJCXX)
  list(APPEND _filament_sources detail/native_window_macos.mm)
else()
  list(APPEND _filament_sources detail/native_window.cpp)
endif()
# Filament's SDK subclass requires matching RTTI settings.
dart_gui_filament_platform_source(_platform_source)
list(APPEND _filament_sources "${_platform_source}")
file(GLOB _filament_headers CONFIGURE_DEPENDS "detail/*.hpp")
dart_gui_filament_generate_material_headers(_material_headers "${CMAKE_CURRENT_BINARY_DIR}/filament_generated")
add_library(
  dart-gui-filament-backend
  OBJECT
  ${_filament_sources}
  ${_filament_headers}
  ${_material_headers}
)
# Backend sources keep same-named file-local helpers; unity batching would
# merge their anonymous namespaces and redefine them.
set_target_properties(
  dart-gui-filament-backend
  PROPERTIES
    CXX_STANDARD 20
    OBJCXX_STANDARD 20
    POSITION_INDEPENDENT_CODE ON
    UNITY_BUILD OFF
)
target_compile_features(dart-gui-filament-backend PRIVATE cxx_std_20)
target_include_directories(
  dart-gui-filament-backend
  PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/filament_generated"
)
# DART dependencies may also expose the environment's general include prefix.
# The selected SDK must take precedence over a different SDK in that prefix.
get_target_property(
  _filament_sdk_includes
  Filament::filament
  INTERFACE_INCLUDE_DIRECTORIES
)
target_include_directories(
  dart-gui-filament-backend
  SYSTEM
  BEFORE
  PRIVATE ${_filament_sdk_includes}
)
target_compile_definitions(
  dart-gui-filament-backend
  PRIVATE DART_BUILDING_DART_GUI
)
target_link_libraries(
  dart-gui-filament-backend
  PRIVATE dart dart-gui-imgui Filament::filament glfw PNG::PNG JPEG::JPEG
)

# Avoid exporting the SDK's language requirement with this C++17 facade.
file(GLOB _gui_headers CONFIGURE_DEPENDS "*.hpp")
file(GLOB _gui_sources CONFIGURE_DEPENDS "*.cpp")
list(REMOVE_ITEM _gui_sources "${CMAKE_CURRENT_SOURCE_DIR}/Viewer.cpp")
dart_add_library(dart-gui ${_gui_headers} ${_gui_sources}
  $<TARGET_OBJECTS:dart-gui-filament-backend>
)
add_component(${PROJECT_NAME} gui)
add_component_targets(${PROJECT_NAME} gui dart-gui)
add_component_dependencies(${PROJECT_NAME} gui dart)
# Add ImGui after the gui component exists; it also defines dart-gui-imgui.
add_subdirectory(imgui)
target_compile_definitions(dart-gui PRIVATE DART_BUILDING_DART_GUI)
if(NOT BUILD_SHARED_LIBS)
  target_compile_definitions(dart-gui PUBLIC DART_GUI_STATIC)
  target_compile_definitions(dart-gui-filament-backend PRIVATE DART_GUI_STATIC)
endif()
target_compile_features(dart-gui PUBLIC cxx_std_17)
target_link_libraries(
  dart-gui
  PUBLIC dart
  PRIVATE DARTFilament::Runtime glfw PNG::PNG JPEG::JPEG
)
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  target_link_options(dart-gui PRIVATE "-Wl,-z,noexecstack")
endif()

# The legacy ImGui name forwards to the complete GUI library for rebuilt
# consumers.
add_library(dart-external-imgui INTERFACE Viewer.hpp)
target_link_libraries(dart-external-imgui INTERFACE dart-gui)
add_dependencies(dart-external-imgui dart-gui)
add_component(${PROJECT_NAME} external-imgui)
add_component_targets(${PROJECT_NAME} external-imgui dart-external-imgui)
add_component_dependencies(${PROJECT_NAME} external-imgui gui)
if(NOT BUILD_SHARED_LIBS)
  add_component_dependency_packages(${PROJECT_NAME} gui Filament glfw3 PNG JPEG)
  if(NOT DART_USE_SYSTEM_FILAMENT)
    message(
      WARNING
      "Static dart-gui links the private Filament SDK, which is not installed; "
      "consumers of this install must set Filament_ROOT=${Filament_ROOT}"
    )
  endif()
endif()
dart_get_filename_components(_gui_header_names "gui headers" ${_gui_headers})
dart_generate_include_header_file("${CMAKE_CURRENT_BINARY_DIR}/gui.hpp"
  "dart/gui/" ${_gui_header_names}
)
install(
  FILES ${_gui_headers} "${CMAKE_CURRENT_BINARY_DIR}/gui.hpp"
  DESTINATION include/dart/gui
  COMPONENT headers
)
install(
  FILES "${DART_FILAMENT_LICENSE_DIRECTORY}/LICENSE"
  DESTINATION "${CMAKE_INSTALL_DATADIR}/licenses/dart/filament"
)
install(
  DIRECTORY "${DART_FILAMENT_LICENSE_DIRECTORY}/notices/"
  DESTINATION "${CMAKE_INSTALL_DATADIR}/licenses/dart/filament/notices"
)
