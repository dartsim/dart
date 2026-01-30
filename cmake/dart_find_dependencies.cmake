# If you add a dependency, please add the corresponding rosdep key as a
# dependency in package.xml.

#======================================
# Required dependencies for DART core
#======================================
if(DART_VERBOSE)
  message(STATUS "")
  message(STATUS "[ Required dependencies for DART core ]")
endif()

# fmt
dart_find_package(fmt)
dart_check_required_package(fmt "libfmt")

# Eigen
dart_find_package(Eigen3)
dart_check_required_package(EIGEN3 "eigen3")

# FCL
dart_find_package(fcl)
dart_check_required_package(fcl "fcl")

# ASSIMP
dart_find_package(assimp)
dart_check_required_package(assimp "assimp")

#=======================
# Optional dependencies
#=======================

# octomap
dart_find_package(octomap)
if(OCTOMAP_FOUND OR octomap_FOUND)
  if(NOT DEFINED octomap_VERSION)
    set(DART_HAVE_OCTOMAP FALSE CACHE BOOL "Check if octomap found." FORCE)
    message(WARNING "Looking for octomap - octomap_VERSION is not defined, " "please install octomap with version information")
  else()
    set(DART_HAVE_OCTOMAP TRUE CACHE BOOL "Check if octomap found." FORCE)
    if(DART_VERBOSE)
      message(STATUS "Looking for octomap - version ${octomap_VERSION} found")
    endif()
  endif()
else()
  set(DART_HAVE_OCTOMAP FALSE CACHE BOOL "Check if octomap found." FORCE)
  message(WARNING "Looking for octomap - NOT found, to use VoxelGridShape, " "please install octomap")
endif()

if(DART_BUILD_PROFILE AND DART_PROFILE_TRACY)
  if(DART_USE_SYSTEM_TRACY)
    find_package(Tracy CONFIG REQUIRED)
  else()
    include(FetchContent)
    FetchContent_Declare(tracy GIT_REPOSITORY https://github.com/wolfpld/tracy.git GIT_TAG v0.11.1 GIT_SHALLOW TRUE GIT_PROGRESS TRUE)
    FetchContent_MakeAvailable(tracy)
    if(MSVC)
      target_compile_options(TracyClient PRIVATE /W0)
    else()
      target_compile_options(TracyClient PRIVATE -w)
    endif()
  endif()
endif()

find_package(Python3 COMPONENTS Interpreter Development)

option(DART_SKIP_spdlog "If ON, do not use spdlog even if it is found." OFF)
mark_as_advanced(DART_SKIP_spdlog)
dart_find_package(spdlog)

# Only fetch ODE/Bullet if the corresponding collision module is enabled
# and system libraries are not being used.
set(_dart_need_fetch_content OFF)
if(DART_BUILD_COLLISION_ODE AND NOT DART_USE_SYSTEM_ODE)
  set(_dart_need_fetch_content ON)
endif()
if(DART_BUILD_COLLISION_BULLET AND NOT DART_USE_SYSTEM_BULLET)
  set(_dart_need_fetch_content ON)
endif()
if(_dart_need_fetch_content)
  include(FetchContent)
endif()
unset(_dart_need_fetch_content)

if(DART_BUILD_COLLISION_ODE AND NOT DART_USE_SYSTEM_ODE)
  # Match Ubuntu/conda-forge libode settings (libccd enabled, box-cylinder disabled).
  set(_dart_build_shared_libs "${BUILD_SHARED_LIBS}")
  set(BUILD_SHARED_LIBS ON CACHE BOOL "" FORCE)

  set(ODE_WITH_LIBCCD ON CACHE BOOL "" FORCE)
  set(ODE_WITH_LIBCCD_SYSTEM ON CACHE BOOL "" FORCE)
  set(ODE_WITH_LIBCCD_BOX_CYL OFF CACHE BOOL "" FORCE)
  set(ODE_WITH_DEMOS OFF CACHE BOOL "" FORCE)
  set(ODE_WITH_TESTS OFF CACHE BOOL "" FORCE)
  # ODE 0.16.6 uses cmake_minimum_required(VERSION 2.8) which is incompatible
  # with CMake 4.0+. Temporarily set CMAKE_POLICY_VERSION_MINIMUM for ODE only.
  set(_dart_old_policy_min "${CMAKE_POLICY_VERSION_MINIMUM}")
  set(CMAKE_POLICY_VERSION_MINIMUM 3.5 CACHE STRING "" FORCE)
  FetchContent_Declare(
    ode
    URL https://bitbucket.org/odedevs/ode/downloads/ode-0.16.6.tar.gz
    URL_HASH SHA256=c91a28c6ff2650284784a79c726a380d6afec87ecf7a35c32a6be0c5b74513e8
    EXCLUDE_FROM_ALL
    SYSTEM
  )
  FetchContent_MakeAvailable(ode)
  # Restore CMAKE_POLICY_VERSION_MINIMUM
  if(_dart_old_policy_min)
    set(CMAKE_POLICY_VERSION_MINIMUM "${_dart_old_policy_min}" CACHE STRING "" FORCE)
  else()
    unset(CMAKE_POLICY_VERSION_MINIMUM CACHE)
  endif()
  unset(_dart_old_policy_min)
  set(DART_ODE_SOURCE_DIR "${ode_SOURCE_DIR}" CACHE INTERNAL "ODE source dir.")
  set(DART_ODE_BINARY_DIR "${ode_BINARY_DIR}" CACHE INTERNAL "ODE binary dir.")

  # Wire fetched ODE into find_package flow by creating ODE::ODE alias target
  # and setting ODE_FOUND so dart_find_package(ODE) succeeds.
  if(NOT TARGET ODE::ODE AND TARGET ODE)
    add_library(ODE::ODE ALIAS ODE)
  endif()
  set(ODE_FOUND TRUE CACHE BOOL "ODE found via FetchContent" FORCE)
  set(ODE_LIBRARIES ODE::ODE CACHE STRING "ODE libraries" FORCE)
  set(ODE_INCLUDE_DIRS "${ode_SOURCE_DIR}/include;${ode_BINARY_DIR}/include" CACHE STRING "ODE include dirs" FORCE)

  set(BUILD_SHARED_LIBS "${_dart_build_shared_libs}" CACHE BOOL "" FORCE)
  unset(_dart_build_shared_libs)
elseif(DART_BUILD_COLLISION_ODE)
  # Using system ODE - clear any stale FetchContent cache values so dart_find_package runs fresh
  unset(ODE_FOUND CACHE)
  unset(ODE_LIBRARIES CACHE)
  unset(ODE_INCLUDE_DIRS CACHE)
  unset(DART_ODE_SOURCE_DIR CACHE)
  unset(DART_ODE_BINARY_DIR CACHE)
endif()

if(DART_BUILD_COLLISION_BULLET AND NOT DART_USE_SYSTEM_BULLET)
  # Match conda-forge Bullet float64 build flags.
  set(_dart_build_shared_libs "${BUILD_SHARED_LIBS}")
  set(BUILD_SHARED_LIBS ON CACHE BOOL "" FORCE)

  set(USE_DOUBLE_PRECISION ON CACHE BOOL "" FORCE)
  set(BULLET2_MULTITHREADING ON CACHE BOOL "" FORCE)
  set(BUILD_BULLET_ROBOTICS_GUI_EXTRA OFF CACHE BOOL "" FORCE)
  set(BUILD_BULLET_ROBOTICS_EXTRA OFF CACHE BOOL "" FORCE)
  set(BUILD_GIMPACTUTILS_EXTRA OFF CACHE BOOL "" FORCE)
  set(BUILD_CPU_DEMOS OFF CACHE BOOL "" FORCE)
  set(BUILD_BULLET2_DEMOS OFF CACHE BOOL "" FORCE)
  set(BUILD_UNIT_TESTS OFF CACHE BOOL "" FORCE)
  set(BUILD_OPENGL3_DEMOS OFF CACHE BOOL "" FORCE)
  set(BUILD_PYBULLET OFF CACHE BOOL "" FORCE)
  set(BUILD_PYBULLET_NUMPY OFF CACHE BOOL "" FORCE)
  set(INSTALL_LIBS ON CACHE BOOL "" FORCE)
  set(INSTALL_EXTRA_LIBS ON CACHE BOOL "" FORCE)
  FetchContent_Declare(
    bullet
    URL https://github.com/bulletphysics/bullet3/archive/refs/tags/3.25.tar.gz
    URL_HASH SHA256=c45afb6399e3f68036ddb641c6bf6f552bf332d5ab6be62f7e6c54eda05ceb77
  )
  FetchContent_MakeAvailable(bullet)
  set(DART_BULLET_SOURCE_DIR "${bullet_SOURCE_DIR}" CACHE INTERNAL "Bullet source dir.")
  set(DART_BULLET_BINARY_DIR "${bullet_BINARY_DIR}" CACHE INTERNAL "Bullet binary dir.")

  # Wire fetched Bullet into find_package flow by setting Bullet_FOUND and
  # BULLET_* variables so dart_find_package(Bullet) succeeds.
  set(Bullet_FOUND TRUE CACHE BOOL "Bullet found via FetchContent" FORCE)
  set(BULLET_FOUND TRUE CACHE BOOL "Bullet found via FetchContent" FORCE)
  set(BULLET_INCLUDE_DIRS "${bullet_SOURCE_DIR}/src" CACHE STRING "Bullet include dirs" FORCE)
  set(BULLET_INCLUDE_DIR "${bullet_SOURCE_DIR}/src" CACHE STRING "Bullet include dir" FORCE)
  # Bullet builds BulletCollision, BulletDynamics, BulletSoftBody, LinearMath targets
  set(BULLET_LIBRARIES BulletCollision BulletDynamics BulletSoftBody LinearMath CACHE STRING "Bullet libraries" FORCE)
  # Pre-set BT_USE_DOUBLE_PRECISION since we built Bullet with USE_DOUBLE_PRECISION=ON.
  # This avoids the check_cxx_source_compiles probe in dart/collision/bullet which
  # would fail at configure time because the Bullet targets aren't built yet.
  set(BT_USE_DOUBLE_PRECISION TRUE CACHE BOOL "Bullet double precision (FetchContent)" FORCE)

  set(BUILD_SHARED_LIBS "${_dart_build_shared_libs}" CACHE BOOL "" FORCE)
  unset(_dart_build_shared_libs)
elseif(DART_BUILD_COLLISION_BULLET)
  # Using system Bullet - clear any stale FetchContent cache values so dart_find_package
  # and the precision probe run fresh
  unset(Bullet_FOUND CACHE)
  unset(BULLET_FOUND CACHE)
  unset(BULLET_INCLUDE_DIRS CACHE)
  unset(BULLET_INCLUDE_DIR CACHE)
  unset(BULLET_LIBRARIES CACHE)
  unset(DART_BULLET_SOURCE_DIR CACHE)
  unset(DART_BULLET_BINARY_DIR CACHE)
  # Clear BT_USE_DOUBLE_PRECISION so the probe in dart/collision/bullet runs
  unset(BT_USE_DOUBLE_PRECISION CACHE)
endif()

#--------------------
# GUI dependencies
#--------------------

# ImGui
if(DART_BUILD_GUI)
  if(DART_USE_SYSTEM_IMGUI)
    # Use system-installed ImGui
    dart_find_package(imgui)
    dart_check_required_package(imgui "imgui")
    if(DART_VERBOSE)
      message(STATUS "Using system-installed ImGui")
    endif()
  else()
    # Fetch ImGui from GitHub using FetchContent
    if(DART_VERBOSE)
      message(STATUS "")
      message(STATUS "[ Fetching ImGui from GitHub ]")
    endif()

    include(FetchContent)

    # ImGui version constraint
    # Current: v1.84.2 (released 2021-08-23)
    # Minimum required: v1.80 for stable table API
    set(IMGUI_MIN_VERSION "1.80")
    set(IMGUI_TARGET_VERSION "v1.84.2")

    message(STATUS "Fetching ImGui ${IMGUI_TARGET_VERSION} from GitHub...")

    FetchContent_Declare(
      imgui
      GIT_REPOSITORY https://github.com/ocornut/imgui.git
      GIT_TAG ${IMGUI_TARGET_VERSION}
      GIT_SHALLOW TRUE
      SOURCE_DIR
      "${CMAKE_BINARY_DIR}/_deps/imgui-src"
    )

    # Populate imgui using the modern helper (avoids CMP0169 warnings)
    FetchContent_GetProperties(imgui)
    if(NOT imgui_POPULATED)
      FetchContent_MakeAvailable(imgui)
    endif()

    # Check OpenGL dependency for ImGui
    dart_find_package(OpenGL)
    dart_check_optional_package(OPENGL "imgui" "OpenGL")

    # Define the imgui source files
    # Core imgui files
    set(
      IMGUI_CORE_SOURCES
      ${imgui_SOURCE_DIR}/imgui.cpp
      ${imgui_SOURCE_DIR}/imgui_draw.cpp
      ${imgui_SOURCE_DIR}/imgui_tables.cpp
      ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    )

    set(
      IMGUI_CORE_HEADERS
      ${imgui_SOURCE_DIR}/imgui.h
      ${imgui_SOURCE_DIR}/imgui_internal.h
      ${imgui_SOURCE_DIR}/imconfig.h
      ${imgui_SOURCE_DIR}/imstb_rectpack.h
      ${imgui_SOURCE_DIR}/imstb_textedit.h
      ${imgui_SOURCE_DIR}/imstb_truetype.h
    )

    # Backend files - OpenGL2 backend for OSG compatibility
    set(IMGUI_BACKEND_SOURCES ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl2.cpp)

    set(IMGUI_BACKEND_HEADERS ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl2.h)

    # Create the ImGui library target
    # Use a unique name to avoid conflicts with example executables
    # Note: We use dart-imgui-lib as the real target name (not an alias)
    # System ImGui provides imgui::imgui, but we can't use that name for export
    set(imgui_library_name dart-imgui-lib)
    set(imgui_component_name imgui)

    dart_add_library(${imgui_library_name}
      ${IMGUI_CORE_SOURCES}
      ${IMGUI_CORE_HEADERS}
      ${IMGUI_BACKEND_SOURCES}
      ${IMGUI_BACKEND_HEADERS}
    )

    if(WIN32 AND BUILD_SHARED_LIBS)
      # ImGui does not export symbols by default; ensure an import library is
      # generated so dart-gui can link against dart-imgui-lib on Windows.
      set_target_properties(${imgui_library_name} PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON)
    endif()

    # Configure include directories
    # Build tree: use fetched source directory
    # Install tree: use standard include paths (like system-installed imgui)
    target_include_directories(
      ${imgui_library_name}
      PUBLIC
        $<BUILD_INTERFACE:${imgui_SOURCE_DIR}>
        $<BUILD_INTERFACE:${imgui_SOURCE_DIR}/backends>
        $<INSTALL_INTERFACE:include>
        $<INSTALL_INTERFACE:include/backends>
    )

    # Link against OpenGL
    target_link_libraries(${imgui_library_name} PUBLIC OpenGL::GL)
    if(APPLE)
      target_link_libraries(${imgui_library_name} PUBLIC "-framework Cocoa")
    endif()

    # Compiler options - suppress warnings for third-party code
    if(CMAKE_COMPILER_IS_GNUCXX)
      target_compile_options(${imgui_library_name} PRIVATE -w)
    endif()

    # Set position independent code for linking into shared libraries (e.g., Python extensions)
    # This is the modern way to add -fPIC
    set_target_properties(${imgui_library_name} PROPERTIES POSITION_INDEPENDENT_CODE ON)

    # Ensure MSVC generates an import library when building shared libs.
    # Without exports, linkers can fail to find dart-imgui-lib.lib (LNK1181).
    if(MSVC AND BUILD_SHARED_LIBS)
      set_target_properties(${imgui_library_name} PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON)
    endif()

    # Define IMGUI_DISABLE_OBSOLETE_FUNCTIONS to avoid using deprecated APIs
    target_compile_definitions(${imgui_library_name} PUBLIC IMGUI_DISABLE_OBSOLETE_FUNCTIONS)

    # Component registration
    # Note: We use dart-imgui-lib as the real target name (not imgui::imgui with ALIAS)
    # because ALIAS targets cannot be exported by CMake install(EXPORT) commands.
    # dart-gui links directly to dart-imgui-lib when using fetched ImGui.
    add_component(${PROJECT_NAME} ${imgui_component_name})
    add_component_targets(${PROJECT_NAME} ${imgui_component_name} ${imgui_library_name})

    # Install fetched ImGui headers to standard system-like paths
    # This allows downstream projects to use standard includes like <imgui.h>
    install(FILES ${IMGUI_CORE_HEADERS} DESTINATION include COMPONENT headers)
    install(FILES ${IMGUI_BACKEND_HEADERS} DESTINATION include/backends COMPONENT headers)

    message(STATUS "ImGui ${IMGUI_TARGET_VERSION} fetched successfully")

    # Add install-time warning about installing fetched ImGui
    install(
      CODE
        "message(WARNING \"Installing fetched ImGui headers to \${CMAKE_INSTALL_PREFIX}/include/. If you have system ImGui installed, this may cause conflicts. For production use, consider building with -DDART_USE_SYSTEM_IMGUI=ON instead.\")"
      COMPONENT headers
    )
  endif()
endif()

# Raylib (experimental)
if(DART_BUILD_GUI_RAYLIB)
  if(DART_USE_SYSTEM_RAYLIB)
    dart_find_package(raylib)
    if(NOT raylib_FOUND)
      message(
        FATAL_ERROR
        "Raylib was requested (DART_BUILD_GUI_RAYLIB=ON, DART_USE_SYSTEM_RAYLIB=ON) but could not be found. Install raylib or set DART_USE_SYSTEM_RAYLIB=OFF to fetch it."
      )
    endif()
    if(DART_VERBOSE)
      message(STATUS "Using system-installed Raylib")
    endif()
  else()
    include(FetchContent)

    set(DART_RAYLIB_GIT_TAG "latest" CACHE STRING "Raylib git tag to fetch when DART_USE_SYSTEM_RAYLIB=OFF")
    mark_as_advanced(DART_RAYLIB_GIT_TAG)

    set(_dart_raylib_git_tag "${DART_RAYLIB_GIT_TAG}")
    if(_dart_raylib_git_tag STREQUAL "" OR _dart_raylib_git_tag STREQUAL "latest")
      set(_dart_raylib_git_tag_resolved "")
      find_package(Git QUIET)
      if(Git_FOUND)
        execute_process(
          COMMAND "${GIT_EXECUTABLE}" ls-remote --tags --refs https://github.com/raysan5/raylib.git
          OUTPUT_VARIABLE _dart_raylib_ls_remote
          RESULT_VARIABLE _dart_raylib_ls_remote_result
          OUTPUT_STRIP_TRAILING_WHITESPACE
        )
        if(_dart_raylib_ls_remote_result EQUAL 0 AND _dart_raylib_ls_remote)
          string(REGEX MATCHALL "refs/tags/[^\n\r]+" _dart_raylib_refs "${_dart_raylib_ls_remote}")
          set(_dart_raylib_tags)
          foreach(_dart_raylib_ref IN LISTS _dart_raylib_refs)
            string(REPLACE "refs/tags/" "" _dart_raylib_tag "${_dart_raylib_ref}")
            if(_dart_raylib_tag MATCHES "^[0-9]+\\.[0-9]+(\\.[0-9]+)?$")
              list(APPEND _dart_raylib_tags "${_dart_raylib_tag}")
            endif()
          endforeach()
          if(_dart_raylib_tags)
            list(SORT _dart_raylib_tags COMPARE NATURAL ORDER DESCENDING)
            list(GET _dart_raylib_tags 0 _dart_raylib_git_tag_resolved)
          endif()
        endif()
      endif()
      if(_dart_raylib_git_tag_resolved)
        set(_dart_raylib_git_tag "${_dart_raylib_git_tag_resolved}")
      endif()
    endif()

    if(_dart_raylib_git_tag STREQUAL "" OR _dart_raylib_git_tag STREQUAL "latest")
      set(_dart_raylib_git_tag "5.5")
      message(
        WARNING
        "Failed to determine the latest Raylib tag; falling back to ${_dart_raylib_git_tag}. Set -DDART_RAYLIB_GIT_TAG=<tag> to override."
      )
    endif()

    message(STATUS "Fetching Raylib ${_dart_raylib_git_tag} from GitHub...")
    FetchContent_Declare(
      raylib
      GIT_REPOSITORY https://github.com/raysan5/raylib.git
      GIT_TAG ${_dart_raylib_git_tag}
      GIT_SHALLOW TRUE
      GIT_PROGRESS TRUE
    )
    FetchContent_MakeAvailable(raylib)
  endif()

  if(NOT TARGET raylib::raylib)
    if(TARGET raylib)
      add_library(raylib::raylib ALIAS raylib)
    endif()
  endif()

  if(NOT TARGET raylib::raylib)
    message(FATAL_ERROR "Raylib was requested (DART_BUILD_GUI_RAYLIB=ON) but no CMake target was provided by the dependency.")
  endif()
endif()

#--------------------
# Misc. dependencies
#--------------------

# Doxygen
find_package(Doxygen QUIET)
dart_check_optional_package(DOXYGEN "generating API documentation" "doxygen")
