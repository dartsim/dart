add_subdirectory(convhull_3d)
if(NOT DART_USE_SYSTEM_IMGUI)
  dart_find_package(OpenGL)
  dart_check_optional_package(OPENGL "dart-external-imgui" "OpenGL")

  include(FetchContent)

  set(IMGUI_TARGET_VERSION "v1.91.9")
  message(STATUS "Fetching ImGui ${IMGUI_TARGET_VERSION} from GitHub...")

  FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG ${IMGUI_TARGET_VERSION}
    GIT_SHALLOW TRUE
    SOURCE_DIR "${DART_BINARY_DIR}/_deps/imgui-src"
  )

  FetchContent_GetProperties(imgui)
  if(NOT imgui_POPULATED)
    FetchContent_MakeAvailable(imgui)
  endif()

  set(imgui_core_sources
    "${imgui_SOURCE_DIR}/imgui.cpp"
    "${imgui_SOURCE_DIR}/imgui_draw.cpp"
    "${imgui_SOURCE_DIR}/imgui_tables.cpp"
    "${imgui_SOURCE_DIR}/imgui_widgets.cpp"
    "${imgui_SOURCE_DIR}/backends/imgui_impl_opengl2.cpp"
  )
  set(imgui_headers
    "${imgui_SOURCE_DIR}/imconfig.h"
    "${imgui_SOURCE_DIR}/imgui.h"
    "${imgui_SOURCE_DIR}/imgui_internal.h"
    "${imgui_SOURCE_DIR}/imstb_rectpack.h"
    "${imgui_SOURCE_DIR}/imstb_textedit.h"
    "${imgui_SOURCE_DIR}/imstb_truetype.h"
    "${imgui_SOURCE_DIR}/backends/imgui_impl_opengl2.h"
  )

  set(target_name ${PROJECT_NAME}-external-imgui)
  set(component_name external-imgui)

  dart_add_library(${target_name} ${imgui_core_sources} ${imgui_headers})
  target_include_directories(${target_name}
    PUBLIC
      $<BUILD_INTERFACE:${imgui_SOURCE_DIR}>
      $<BUILD_INTERFACE:${imgui_SOURCE_DIR}/backends>
      $<INSTALL_INTERFACE:include>
      $<INSTALL_INTERFACE:include/dart/external/imgui>
  )
  target_link_libraries(${target_name} PUBLIC OpenGL::GL)
  if(APPLE)
    target_link_libraries(${target_name} PUBLIC "-framework Cocoa")
  endif()

  if(CMAKE_COMPILER_IS_GNUCXX)
    target_compile_options(${target_name} PRIVATE -w)
  endif()
  target_compile_definitions(
    ${target_name} PUBLIC IMGUI_DISABLE_OBSOLETE_FUNCTIONS
  )

  add_component(${PROJECT_NAME} ${component_name})
  add_component_targets(${PROJECT_NAME} ${component_name} ${target_name})

  install(
    FILES ${imgui_headers}
    DESTINATION include/dart/external/imgui
    COMPONENT headers
  )
endif()
add_subdirectory(ikfast)
add_subdirectory(lodepng)
add_subdirectory(odelcpsolver)
