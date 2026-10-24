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

  # The docking release of v1.92.8 is vanilla v1.92.8 plus the dockspace and
  # DockBuilder API (IMGUI_HAS_DOCK) that the demo workspace uses. Docking
  # adds public ImGuiIO and ImGuiStyle members and shifts enum values, so the
  # installed dart/external/imgui headers use the docking layout. A vanilla
  # system ImGui (DART_USE_SYSTEM_IMGUI=ON) still builds dart-gui.
  set(IMGUI_TARGET_VERSION "v1.92.8-docking")
  set(
    IMGUI_ARCHIVE_SHA256
    ca0653454ed371b7a87e9b0bc29a5d15c9be7f7c0fbe778042fc48c71df1d3d8
  )

  # On CMake 3.24 and newer, extract with the current time so a changed
  # archive rebuilds its objects. The hash-specific source path also rebuilds
  # objects on CMake 3.22/3.23, which preserve archive timestamps.
  set(dart_imgui_timestamp)
  if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.24)
    set(dart_imgui_timestamp DOWNLOAD_EXTRACT_TIMESTAMP FALSE)
  endif()

  message(STATUS "Fetching ImGui ${IMGUI_TARGET_VERSION} from GitHub...")

  # Update IMGUI_ARCHIVE_SHA256 whenever IMGUI_TARGET_VERSION changes.
  # A changed archive gets new source paths even when its timestamps are old.
  FetchContent_Declare(
    dart_imgui
    URL
      "https://github.com/ocornut/imgui/archive/refs/tags/${IMGUI_TARGET_VERSION}.tar.gz"
    URL_HASH SHA256=${IMGUI_ARCHIVE_SHA256}
    SOURCE_DIR
    "${DART_BINARY_DIR}/_deps/dart_imgui-${IMGUI_ARCHIVE_SHA256}-src"
    TLS_VERIFY ON
    ${dart_imgui_timestamp}
  )

  FetchContent_MakeAvailable(dart_imgui)
  dart_check_fetchcontent(
    dart_imgui DART_USE_SYSTEM_IMGUI
    "${DART_BINARY_DIR}/_deps/dart_imgui-${IMGUI_ARCHIVE_SHA256}-src"
  )

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
  )
  # An install prefix can also hold another ImGui (for example a Conda
  # environment's vanilla imgui.h), so installed consumers must find the
  # bundled docking headers first when they write <imgui.h>.
  target_include_directories(
    ${target_name}
    BEFORE
    PUBLIC $<INSTALL_INTERFACE:include/dart/external/imgui>
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

# Third-party sources: no warnings on any compiler, and kept out of unity
# batches, where their file-local helpers could clash with dart-gui sources.
if(imgui_core_sources)
  set_source_files_properties(
    ${imgui_core_sources}
    TARGET_DIRECTORY ${target_name}
    PROPERTIES
      COMPILE_OPTIONS "$<IF:$<CXX_COMPILER_ID:MSVC>,/w,-w>"
      SKIP_UNITY_BUILD_INCLUSION ON
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
