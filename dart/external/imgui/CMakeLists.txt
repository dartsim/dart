set(target_name ${PROJECT_NAME}-external-imgui)
set(component_name external-imgui)

set(imgui_header_names
  imconfig.h
  imgui.h
  imgui_internal.h
  imstb_rectpack.h
  imstb_textedit.h
  imstb_truetype.h
  imgui_impl_opengl2.h
)

function(dart_find_imgui_header out_var header_name)
  foreach(search_dir IN LISTS ARGN)
    if(NOT search_dir OR NOT IS_DIRECTORY "${search_dir}")
      continue()
    endif()

    foreach(candidate_dir
        "${search_dir}"
        "${search_dir}/imgui"
        "${search_dir}/imgui/backends"
        "${search_dir}/backends")
      if(EXISTS "${candidate_dir}/${header_name}")
        set(${out_var} "${candidate_dir}/${header_name}" PARENT_SCOPE)
        return()
      endif()
    endforeach()
  endforeach()

  string(MAKE_C_IDENTIFIER "${header_name}" header_cache_name)
  set(resolved_header_var "resolved_${header_cache_name}")
  find_file(${resolved_header_var}
    NAMES "${header_name}"
    PATH_SUFFIXES imgui imgui/backends backends
  )
  if(NOT ${resolved_header_var})
    message(FATAL_ERROR "Could not find ImGui header ${header_name}")
  endif()

  set(${out_var} "${${resolved_header_var}}" PARENT_SCOPE)
endfunction()

function(dart_configure_imgui_compat_headers out_var)
  set(imgui_compat_header_dir "${DART_BINARY_DIR}/dart/external/imgui")
  file(MAKE_DIRECTORY "${imgui_compat_header_dir}")

  set(compat_headers)
  foreach(imgui_header_name IN LISTS imgui_header_names)
    dart_find_imgui_header(imgui_header "${imgui_header_name}" ${ARGN})
    configure_file(
      "${imgui_header}"
      "${imgui_compat_header_dir}/${imgui_header_name}"
      COPYONLY
    )
    list(APPEND compat_headers "${imgui_compat_header_dir}/${imgui_header_name}")
  endforeach()

  set(${out_var} ${compat_headers} PARENT_SCOPE)
endfunction()

if(DART_USE_SYSTEM_IMGUI)
  dart_find_package(imgui)
  dart_check_required_package(imgui "imgui")

  set(imgui_header_search_dirs ${imgui_INCLUDE_DIRS})
  get_target_property(
    imgui_target_include_dirs imgui::imgui INTERFACE_INCLUDE_DIRECTORIES
  )
  if(imgui_target_include_dirs)
    list(APPEND imgui_header_search_dirs ${imgui_target_include_dirs})
  endif()

  dart_configure_imgui_compat_headers(
    imgui_compat_headers ${imgui_header_search_dirs}
  )
  set(imgui_compat_source "${CMAKE_CURRENT_BINARY_DIR}/external_imgui_compat.cpp")
  file(WRITE "${imgui_compat_source}"
    "namespace dart_external_imgui { void keep_component() {} }\n"
  )

  dart_add_library(${target_name} ${imgui_compat_source} ${imgui_compat_headers})
  target_include_directories(${target_name}
    PUBLIC
      $<BUILD_INTERFACE:${DART_BINARY_DIR}>
      $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(${target_name} PUBLIC imgui::imgui)
else()
  dart_find_package(OpenGL)
  if(NOT DEFINED OPENGL_FOUND)
    if(DEFINED OpenGL_FOUND)
      set(OPENGL_FOUND ${OpenGL_FOUND})
    else()
      set(OPENGL_FOUND FALSE)
    endif()
  endif()
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
    "${CMAKE_CURRENT_LIST_DIR}/../imgui_patches/dart-compat.patch"
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
      file(READ "${imgui_SOURCE_DIR}/imgui_draw.cpp" imgui_draw_contents)
      file(READ "${imgui_SOURCE_DIR}/imgui_internal.h" imgui_internal_contents)
      file(READ "${imgui_SOURCE_DIR}/imgui_widgets.cpp" imgui_widgets_contents)
      string(FIND
        "${imgui_draw_contents}"
        "Guard against null/empty input (issue #2516)"
        imgui_draw_ttf_patch_marker
      )
      string(FIND
        "${imgui_draw_contents}"
        "Guard against null/empty input to prevent SEGV in stb_decompress (issue #2516)"
        imgui_draw_compressed_ttf_patch_marker
      )
      string(FIND
        "${imgui_draw_contents}"
        "Guard against null input (issue #2516)"
        imgui_draw_base85_patch_marker
      )
      string(FIND
        "${imgui_internal_contents}"
        "IMGUI_INTERNAL_FORMAT_API"
        imgui_internal_patch_marker
      )
      string(FIND
        "${imgui_widgets_contents}"
        "issue #2668"
        imgui_widgets_patch_marker
      )
      if(imgui_draw_ttf_patch_marker EQUAL -1
          OR imgui_draw_compressed_ttf_patch_marker EQUAL -1
          OR imgui_draw_base85_patch_marker EQUAL -1
          OR imgui_internal_patch_marker EQUAL -1
          OR imgui_widgets_patch_marker EQUAL -1)
        message(FATAL_ERROR "DART ImGui patch is incompatible: ${imgui_patch}")
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
  dart_configure_imgui_compat_headers(
    imgui_compat_headers "${imgui_SOURCE_DIR}" "${imgui_SOURCE_DIR}/backends"
  )

  dart_add_library(${target_name} ${imgui_core_sources} ${imgui_compat_headers})
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
endif()

if(CMAKE_COMPILER_IS_GNUCXX)
  target_compile_options(${target_name} PRIVATE -w)
endif()
target_compile_definitions(${target_name} PUBLIC IMGUI_DISABLE_OBSOLETE_FUNCTIONS)

add_component(${PROJECT_NAME} ${component_name})
add_component_targets(${PROJECT_NAME} ${component_name} ${target_name})
if(DART_USE_SYSTEM_IMGUI)
  add_component_dependency_packages(${PROJECT_NAME} ${component_name} imgui)
endif()

install(
  FILES ${imgui_compat_headers}
  DESTINATION include/dart/external/imgui
  COMPONENT headers
)
