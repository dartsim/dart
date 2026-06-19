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

function(dart_try_find_imgui_header out_var header_name)
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

  set(${out_var} "" PARENT_SCOPE)
endfunction()

function(dart_find_imgui_source out_var source_name)
  foreach(search_dir IN LISTS ARGN)
    if(NOT search_dir OR NOT IS_DIRECTORY "${search_dir}")
      continue()
    endif()

    foreach(candidate_dir
        "${search_dir}"
        "${search_dir}/imgui"
        "${search_dir}/imgui/backends"
        "${search_dir}/backends"
        "${search_dir}/../share/imgui"
        "${search_dir}/../share/imgui/backends"
        "${search_dir}/../share/imgui/examples"
        "${search_dir}/../share/imgui/examples/backends"
        "${search_dir}/../share/doc/imgui/examples"
        "${search_dir}/../share/doc/imgui/examples/backends"
        "${search_dir}/../share/doc/libimgui-dev/examples/backends"
        "${search_dir}/../../share/doc/imgui/examples"
        "${search_dir}/../../share/doc/imgui/examples/backends"
        "${search_dir}/../../share/doc/libimgui-dev/examples/backends")
      if(EXISTS "${candidate_dir}/${source_name}")
        set(${out_var} "${candidate_dir}/${source_name}" PARENT_SCOPE)
        return()
      endif()
    endforeach()
  endforeach()

  string(MAKE_C_IDENTIFIER "${source_name}" source_cache_name)
  set(resolved_source_var "resolved_${source_cache_name}")
  find_file(${resolved_source_var}
    NAMES "${source_name}"
    HINTS ${ARGN}
    PATH_SUFFIXES
      imgui
      imgui/backends
      backends
      share/imgui/examples/backends
      share/doc/imgui/examples
      share/doc/imgui/examples/backends
      share/doc/libimgui-dev/examples/backends
      ../share/imgui
      ../share/imgui/backends
      ../share/imgui/examples
      ../share/imgui/examples/backends
      ../share/doc/imgui/examples
      ../share/doc/imgui/examples/backends
      ../share/doc/libimgui-dev/examples/backends
      ../../share/doc/imgui/examples
      ../../share/doc/imgui/examples/backends
      ../../share/doc/libimgui-dev/examples/backends
  )
  if(NOT ${resolved_source_var})
    message(
      FATAL_ERROR
      "Could not find ImGui source ${source_name}. The system ImGui package "
      "must either provide OpenGL2 backend symbols in imgui::imgui or install "
      "${source_name} so DART can build its compatibility target."
    )
  endif()

  set(${out_var} "${${resolved_source_var}}" PARENT_SCOPE)
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

macro(dart_try_find_system_imgui)
  if(CMAKE_DISABLE_FIND_PACKAGE_imgui)
    set(imgui_FOUND FALSE)
  else()
    find_package(imgui QUIET CONFIG)
  endif()

  if(NOT imgui_FOUND AND NOT CMAKE_DISABLE_FIND_PACKAGE_imgui)
    find_package(PkgConfig QUIET)
    pkg_check_modules(PC_imgui imgui QUIET)

    find_path(_dart_imgui_include_dir
      NAMES imgui.h
      HINTS ${PC_imgui_INCLUDEDIR}
      PATHS
        "${CMAKE_INSTALL_PREFIX}/include"
        "${CMAKE_INSTALL_PREFIX}/include/imgui"
    )
    find_path(_dart_imgui_backends_include_dir
      NAMES imgui_impl_opengl2.h
      HINTS
        ${PC_imgui_INCLUDEDIR}
        ${PC_imgui_INCLUDEDIR}/backends
      PATHS
        "${CMAKE_INSTALL_PREFIX}/include"
        "${CMAKE_INSTALL_PREFIX}/include/imgui/backends"
    )
    find_library(_dart_imgui_library
      NAMES imgui
      HINTS ${PC_imgui_LIBDIR}
    )

    if(_dart_imgui_include_dir
        AND _dart_imgui_backends_include_dir
        AND _dart_imgui_library)
      set(imgui_INCLUDE_DIRS
        ${_dart_imgui_include_dir}
        ${_dart_imgui_backends_include_dir}
      )
      set(imgui_LIBRARIES ${_dart_imgui_library})
      set(imgui_FOUND TRUE)
    endif()
  endif()

  if(imgui_FOUND AND NOT TARGET imgui::imgui)
    add_library(imgui::imgui INTERFACE IMPORTED)
    set_target_properties(imgui::imgui PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${imgui_INCLUDE_DIRS}"
      INTERFACE_LINK_LIBRARIES "${imgui_LIBRARIES}"
    )
  endif()
endmacro()

if(DART_USE_SYSTEM_IMGUI)
  if(DART_BUILD_GUI_OSG)
    dart_find_package(imgui)
    dart_check_required_package(imgui "imgui")
  else()
    dart_try_find_system_imgui()
    if(NOT imgui_FOUND OR NOT TARGET imgui::imgui)
      message(STATUS "Skipping dart-external-imgui: system ImGui was not found")
      return()
    endif()
  endif()

  dart_find_package(OpenGL)
  if((NOT OPENGL_FOUND AND NOT OpenGL_FOUND) OR NOT TARGET OpenGL::GL)
    message(STATUS "Skipping dart-external-imgui: OpenGL was not found")
    return()
  endif()
  if(DART_BUILD_GUI_OSG)
    dart_check_required_package(OpenGL "OpenGL")
  endif()

  set(imgui_header_search_dirs ${imgui_INCLUDE_DIRS})
  get_target_property(
    imgui_target_include_dirs imgui::imgui INTERFACE_INCLUDE_DIRECTORIES
  )
  if(imgui_target_include_dirs)
    list(APPEND imgui_header_search_dirs ${imgui_target_include_dirs})
  endif()

  set(imgui_missing_compat_headers)
  foreach(imgui_header_name IN LISTS imgui_header_names)
    dart_try_find_imgui_header(
      imgui_compat_header
      "${imgui_header_name}"
      ${imgui_header_search_dirs}
    )
    if(NOT imgui_compat_header)
      list(APPEND imgui_missing_compat_headers "${imgui_header_name}")
    endif()
  endforeach()
  if(imgui_missing_compat_headers AND NOT DART_BUILD_GUI_OSG)
    string(REPLACE ";" ", " imgui_missing_compat_headers_text
      "${imgui_missing_compat_headers}"
    )
    message(STATUS
      "Skipping dart-external-imgui: system ImGui compatibility headers were "
      "not found (${imgui_missing_compat_headers_text})"
    )
    return()
  endif()

  dart_configure_imgui_compat_headers(
    imgui_compat_headers ${imgui_header_search_dirs}
  )
  set(imgui_compat_source "${CMAKE_CURRENT_BINARY_DIR}/external_imgui_compat.cpp")
  file(WRITE "${imgui_compat_source}"
    "namespace dart_external_imgui { void keep_component() {} }\n"
  )

  set(imgui_compat_sources ${imgui_compat_source})

  # Some system packages put the OpenGL2 backend in libimgui, while others
  # install the backend as example source that consumers must compile.
  include(CheckCXXSourceCompiles)
  set(_dart_saved_required_includes ${CMAKE_REQUIRED_INCLUDES})
  set(_dart_saved_required_libraries ${CMAKE_REQUIRED_LIBRARIES})
  set(_dart_saved_required_quiet ${CMAKE_REQUIRED_QUIET})
  set(CMAKE_REQUIRED_INCLUDES ${imgui_header_search_dirs})
  set(CMAKE_REQUIRED_LIBRARIES imgui::imgui OpenGL::GL)
  set(CMAKE_REQUIRED_QUIET TRUE)
  check_cxx_source_compiles(
    "
    #include <imgui_impl_opengl2.h>
    int main()
    {
      ImGui_ImplOpenGL2_Init();
      ImGui_ImplOpenGL2_NewFrame();
      ImGui_ImplOpenGL2_RenderDrawData(nullptr);
      return 0;
    }
    "
    DART_SYSTEM_IMGUI_HAS_OPENGL2_BACKEND
  )
  set(CMAKE_REQUIRED_INCLUDES ${_dart_saved_required_includes})
  set(CMAKE_REQUIRED_LIBRARIES ${_dart_saved_required_libraries})
  set(CMAKE_REQUIRED_QUIET ${_dart_saved_required_quiet})
  unset(_dart_saved_required_includes)
  unset(_dart_saved_required_libraries)
  unset(_dart_saved_required_quiet)

  if(NOT DART_SYSTEM_IMGUI_HAS_OPENGL2_BACKEND)
    if(NOT DART_BUILD_GUI_OSG)
      message(STATUS
        "Skipping dart-external-imgui: system ImGui OpenGL2 backend was not found"
      )
      return()
    endif()

    dart_find_imgui_source(
      imgui_opengl2_backend_source
      imgui_impl_opengl2.cpp
      ${imgui_header_search_dirs}
    )
    list(APPEND imgui_compat_sources "${imgui_opengl2_backend_source}")
  endif()

  dart_add_library(${target_name} ${imgui_compat_sources} ${imgui_compat_headers})
  target_include_directories(${target_name}
    PUBLIC
      $<BUILD_INTERFACE:${DART_BINARY_DIR}>
      $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(${target_name} PUBLIC imgui::imgui OpenGL::GL)
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
      file(READ
        "${imgui_SOURCE_DIR}/backends/imgui_impl_opengl2.cpp"
        imgui_opengl2_contents
      )
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
      string(FIND
        "${imgui_opengl2_contents}"
        "glPushClientAttrib(GL_VERTEX_ARRAY | GL_TEXTURE_COORD_ARRAY | GL_COLOR_ARRAY)"
        imgui_opengl2_patch_marker
      )
      if(imgui_draw_ttf_patch_marker EQUAL -1
          OR imgui_draw_compressed_ttf_patch_marker EQUAL -1
          OR imgui_draw_base85_patch_marker EQUAL -1
          OR imgui_internal_patch_marker EQUAL -1
          OR imgui_widgets_patch_marker EQUAL -1
          OR imgui_opengl2_patch_marker EQUAL -1)
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
if(NOT DART_USE_SYSTEM_IMGUI)
  target_compile_definitions(${target_name} PUBLIC IMGUI_DISABLE_OBSOLETE_FUNCTIONS)
endif()

add_component(${PROJECT_NAME} ${component_name})
add_component_targets(${PROJECT_NAME} ${component_name} ${target_name})
if(DART_USE_SYSTEM_IMGUI)
  add_component_dependency_packages(${PROJECT_NAME} ${component_name} imgui OpenGL)
endif()

install(
  FILES ${imgui_compat_headers}
  DESTINATION include/dart/external/imgui
  COMPONENT headers
)
