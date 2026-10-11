# ImGui compiles into dart-gui. The internal dart-gui-imgui interface gives the
# Filament backend objects the same ImGui headers and definitions.
set(target_name ${PROJECT_NAME}-gui)
set(component_dependency_packages)
add_library(dart-gui-imgui INTERFACE)

set(
  imgui_header_names
  imconfig.h
  imgui.h
  imgui_internal.h
  imstb_rectpack.h
  imstb_textedit.h
  imstb_truetype.h
)

function(dart_find_imgui_header out_var header_name)
  foreach(search_dir IN LISTS ARGN)
    if(NOT search_dir OR NOT IS_DIRECTORY "${search_dir}")
      continue()
    endif()

    foreach(candidate_dir "${search_dir}" "${search_dir}/imgui")
      if(EXISTS "${candidate_dir}/${header_name}")
        set(${out_var} "${candidate_dir}/${header_name}" PARENT_SCOPE)
        return()
      endif()
    endforeach()
  endforeach()

  string(MAKE_C_IDENTIFIER "${header_name}" header_cache_name)
  set(resolved_header_var "resolved_${header_cache_name}")
  find_file(${resolved_header_var} NAMES "${header_name}" PATH_SUFFIXES imgui)
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
    list(
      APPEND compat_headers
      "${imgui_compat_header_dir}/${imgui_header_name}"
    )
  endforeach()

  set(${out_var} ${compat_headers} PARENT_SCOPE)
endfunction()

if(DART_USE_SYSTEM_IMGUI)
  dart_find_package(imgui)
  if(NOT imgui_FOUND OR NOT TARGET imgui::imgui)
    message(
      FATAL_ERROR
      "dart-gui requires ImGui >= 1.92.8; install it or set DART_USE_SYSTEM_IMGUI=OFF."
    )
  endif()
  if(imgui_VERSION VERSION_LESS 1.92.8)
    message(
      FATAL_ERROR
      "The Filament GUI requires Dear ImGui >= 1.92.8 (found ${imgui_VERSION}); "
      "upgrade it or use the bundled ImGui with DART_USE_SYSTEM_IMGUI=OFF."
    )
  endif()

  set(imgui_header_search_dirs ${imgui_INCLUDE_DIRS})
  get_target_property(
    imgui_target_include_dirs
    imgui::imgui
    INTERFACE_INCLUDE_DIRECTORIES
  )
  if(imgui_target_include_dirs)
    list(APPEND imgui_header_search_dirs ${imgui_target_include_dirs})
  endif()

  dart_configure_imgui_compat_headers(
    imgui_compat_headers ${imgui_header_search_dirs}
  )

  target_sources(${target_name} PRIVATE ${imgui_compat_headers})
  target_include_directories(
    ${target_name}
    PUBLIC $<BUILD_INTERFACE:${DART_BINARY_DIR}> $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(
    ${target_name}
    PUBLIC
      $<BUILD_INTERFACE:imgui::imgui>
      $<INSTALL_INTERFACE:$<1:imgui::imgui>>
  )
  target_include_directories(dart-gui-imgui INTERFACE "${DART_BINARY_DIR}")
  target_link_libraries(dart-gui-imgui INTERFACE imgui::imgui)
else()
  include(FetchContent)

  # Default to the vanilla tag to preserve the installed ImGui header layout
  # (docking adds public ImGuiIO /
  # ImGuiStyle members and shifts enum values). DART_IMGUI_DOCKING opts a
  # build into the docking-branch release of the same version — identical to
  # v1.92.8 plus the dockspace/DockBuilder API (IMGUI_HAS_DOCK) — which the
  # demos configuration (`pixi run demos` via config-demos) uses for the
  # rearrangeable docked workspace. Non-docking builds keep working through
  # the fixed-layout fallback.
  if(DART_IMGUI_DOCKING)
    set(IMGUI_TARGET_VERSION "v1.92.8-docking")
  else()
    set(IMGUI_TARGET_VERSION "v1.92.8")
  endif()

  # DART applies its compatibility patch directly inside the FetchContent
  # checkout, which leaves that working tree dirty. If the requested tag changed
  # since this build tree was last configured, FetchContent's in-place
  # re-checkout to the new tag fails on the local patch hunks (configure stops
  # until the cached source is deleted by hand). Detect a tag change via a stamp
  # file and drop the cached download so the new tag is fetched from a clean
  # state.
  set(dart_imgui_src_dir "${DART_BINARY_DIR}/_deps/imgui-src")
  set(dart_imgui_tag_stamp "${DART_BINARY_DIR}/_deps/dart_imgui_tag.txt")
  set(dart_imgui_cached_tag "")
  if(EXISTS "${dart_imgui_tag_stamp}")
    file(READ "${dart_imgui_tag_stamp}" dart_imgui_cached_tag)
    string(STRIP "${dart_imgui_cached_tag}" dart_imgui_cached_tag)
  endif()
  if(
    EXISTS "${dart_imgui_src_dir}"
    AND NOT dart_imgui_cached_tag STREQUAL IMGUI_TARGET_VERSION
  )
    message(
      STATUS
      "Bundled ImGui tag changed to ${IMGUI_TARGET_VERSION}; refreshing cached download"
    )
    # The source uses a custom SOURCE_DIR (imgui-src); the FetchContent sub-build
    # and build trees are named after the declared content (dart_imgui-*). Remove
    # all of them so the next populate re-clones the new tag from scratch.
    file(GLOB dart_imgui_fetch_dirs "${DART_BINARY_DIR}/_deps/dart_imgui-*")
    file(REMOVE_RECURSE "${dart_imgui_src_dir}" ${dart_imgui_fetch_dirs})
  endif()

  message(STATUS "Fetching ImGui ${IMGUI_TARGET_VERSION} from GitHub...")

  FetchContent_Declare(
    dart_imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG ${IMGUI_TARGET_VERSION}
    GIT_SHALLOW TRUE
    SOURCE_DIR
    "${DART_BINARY_DIR}/_deps/imgui-src"
  )

  FetchContent_GetProperties(dart_imgui)
  if(NOT dart_imgui_POPULATED)
    FetchContent_MakeAvailable(dart_imgui)
  endif()

  find_package(Git REQUIRED)
  set(imgui_patch_files "${CMAKE_CURRENT_LIST_DIR}/dart-compat.patch")
  foreach(imgui_patch IN LISTS imgui_patch_files)
    execute_process(
      COMMAND
        "${GIT_EXECUTABLE}" -C "${dart_imgui_SOURCE_DIR}" apply --check
        "${imgui_patch}"
      RESULT_VARIABLE imgui_patch_check_result
      OUTPUT_QUIET
      ERROR_QUIET
    )
    if(imgui_patch_check_result EQUAL 0)
      message(STATUS "Applying DART ImGui compatibility patch: ${imgui_patch}")
      execute_process(
        COMMAND
          "${GIT_EXECUTABLE}" -C "${dart_imgui_SOURCE_DIR}" apply
          "${imgui_patch}"
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
      file(READ "${dart_imgui_SOURCE_DIR}/imgui_draw.cpp" imgui_draw_contents)
      file(
        READ "${dart_imgui_SOURCE_DIR}/imgui_internal.h"
        imgui_internal_contents
      )
      file(
        READ "${dart_imgui_SOURCE_DIR}/imgui_widgets.cpp"
        imgui_widgets_contents
      )
      string(
        FIND "${imgui_draw_contents}"
        "Guard against null/empty input (issue #2516)"
        imgui_draw_ttf_patch_marker
      )
      string(
        FIND "${imgui_draw_contents}"
        "Guard against null/empty input to prevent SEGV in stb_decompress (issue #2516)"
        imgui_draw_compressed_ttf_patch_marker
      )
      string(
        FIND "${imgui_draw_contents}"
        "Guard against null input (issue #2516)"
        imgui_draw_base85_patch_marker
      )
      string(
        FIND "${imgui_internal_contents}"
        "IMGUI_INTERNAL_FORMAT_API"
        imgui_internal_patch_marker
      )
      string(
        FIND "${imgui_widgets_contents}"
        "issue #2668"
        imgui_widgets_patch_marker
      )
      if(
        imgui_draw_ttf_patch_marker EQUAL -1
        OR imgui_draw_compressed_ttf_patch_marker EQUAL -1
        OR imgui_draw_base85_patch_marker EQUAL -1
        OR imgui_internal_patch_marker EQUAL -1
        OR imgui_widgets_patch_marker EQUAL -1
      )
        message(FATAL_ERROR "DART ImGui patch is incompatible: ${imgui_patch}")
      endif()
    endif()
  endforeach()

  # Record the tag now that the matching ImGui sources are populated and patched
  # so a later tag change can refresh the cached download (see above).
  file(WRITE "${dart_imgui_tag_stamp}" "${IMGUI_TARGET_VERSION}\n")

  set(
    imgui_core_sources
    "${dart_imgui_SOURCE_DIR}/imgui.cpp"
    "${dart_imgui_SOURCE_DIR}/imgui_draw.cpp"
    "${dart_imgui_SOURCE_DIR}/imgui_tables.cpp"
    "${dart_imgui_SOURCE_DIR}/imgui_widgets.cpp"
  )
  dart_configure_imgui_compat_headers(
    imgui_compat_headers
    "${dart_imgui_SOURCE_DIR}"
  )

  target_sources(
    ${target_name}
    PRIVATE ${imgui_core_sources} ${imgui_compat_headers}
  )
  target_include_directories(
    ${target_name}
    PUBLIC
      $<BUILD_INTERFACE:${DART_BINARY_DIR}>
      $<BUILD_INTERFACE:${dart_imgui_SOURCE_DIR}>
      $<INSTALL_INTERFACE:include>
      $<INSTALL_INTERFACE:include/dart/external/imgui>
  )
  target_compile_definitions(
    ${target_name}
    PUBLIC IMGUI_DISABLE_OBSOLETE_FUNCTIONS
  )
  target_include_directories(
    dart-gui-imgui
    INTERFACE "${DART_BINARY_DIR}" "${dart_imgui_SOURCE_DIR}"
  )
  target_compile_definitions(
    dart-gui-imgui
    INTERFACE IMGUI_DISABLE_OBSOLETE_FUNCTIONS
  )
endif()

if(CMAKE_COMPILER_IS_GNUCXX AND imgui_core_sources)
  set_source_files_properties(
    ${imgui_core_sources}
    TARGET_DIRECTORY ${target_name}
    PROPERTIES COMPILE_OPTIONS -w
  )
endif()

if(DART_USE_SYSTEM_IMGUI)
  list(APPEND component_dependency_packages imgui)
endif()
add_component_dependency_packages(
  ${PROJECT_NAME} gui ${component_dependency_packages}
)

install(
  FILES ${imgui_compat_headers}
  DESTINATION include/dart/external/imgui
  COMPONENT headers
)
