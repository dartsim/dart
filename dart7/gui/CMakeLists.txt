add_library(dart7-gui STATIC
  renderer.cpp
)

target_include_directories(dart7-gui
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(dart7-gui
  PUBLIC
    dart7::dart7
)

target_compile_definitions(dart7-gui
  PUBLIC
    DART_BUILD_DART7_GUI=1
)

target_compile_features(dart7-gui PUBLIC cxx_std_20)

add_library(dart7::gui ALIAS dart7-gui)
