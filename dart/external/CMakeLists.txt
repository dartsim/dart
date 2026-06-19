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

  find_package(Git REQUIRED)
  set(imgui_patch_files
    "${CMAKE_CURRENT_LIST_DIR}/imgui_patches/dart-compat.patch"
  )
  foreach(imgui_patch IN LISTS imgui_patch_files)
    execute_process(
      COMMAND "${GIT_EXECUTABLE}" -C "${imgui_SOURCE_DIR}" apply --check "${imgui_patch}"
      RESULT_VARIABLE imgui_patch_check_result
      OUTPUT_QUIET
      ERROR_QUIET
    )
    if(imgui_patch_check_result EQUAL 0)
      message(STATUS "Applying DART ImGui compatibility patch: ${imgui_patch}")
      execute_process(
        COMMAND "${GIT_EXECUTABLE}" -C "${imgui_SOURCE_DIR}" apply "${imgui_patch}"
        RESULT_VARIABLE imgui_patch_result
        OUTPUT_VARIABLE imgui_patch_output
        ERROR_VARIABLE imgui_patch_error
      )
      if(NOT imgui_patch_result EQUAL 0)
        message(
          FATAL_ERROR
          "Failed to apply ${imgui_patch}:\n${imgui_patch_output}\n${imgui_patch_error}"
        )
      endif()
    else()
      execute_process(
        COMMAND "${GIT_EXECUTABLE}" -C "${imgui_SOURCE_DIR}" apply --reverse --check "${imgui_patch}"
        RESULT_VARIABLE imgui_patch_reverse_check_result
        OUTPUT_QUIET
        ERROR_QUIET
      )
      if(NOT imgui_patch_reverse_check_result EQUAL 0)
        message(FATAL_ERROR "DART ImGui compatibility patch is incompatible: ${imgui_patch}")
      endif()
    endif()
  endforeach()

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
  set(imgui_compat_header_dir "${DART_BINARY_DIR}/dart/external/imgui")
  file(MAKE_DIRECTORY "${imgui_compat_header_dir}")
  foreach(imgui_header IN LISTS imgui_headers)
    get_filename_component(imgui_header_name "${imgui_header}" NAME)
    configure_file(
      "${imgui_header}"
      "${imgui_compat_header_dir}/${imgui_header_name}"
      COPYONLY
    )
  endforeach()

  set(target_name ${PROJECT_NAME}-external-imgui)
  set(component_name external-imgui)

  dart_add_library(${target_name} ${imgui_core_sources} ${imgui_headers})
  target_include_directories(${target_name}
    PUBLIC
      $<BUILD_INTERFACE:${DART_BINARY_DIR}>
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
