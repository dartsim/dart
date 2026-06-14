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

# ASSIMP
dart_find_package(assimp)
dart_check_required_package(assimp "assimp")

#=======================
# Optional dependencies
#=======================

# OctoMap is intentionally not discovered for core DART. Tests and benchmarks
# may find it locally for correctness and performance comparisons.
set(DART_HAVE_OCTOMAP FALSE CACHE BOOL "Check if octomap found." FORCE)

if(DART_BUILD_PROFILE AND DART_PROFILE_TRACY)
  if(DART_USE_SYSTEM_TRACY)
    find_package(Tracy CONFIG REQUIRED)
  else()
    include(FetchContent)
    FetchContent_Declare(
      tracy
      GIT_REPOSITORY https://github.com/wolfpld/tracy.git
      GIT_TAG v0.13.1
      GIT_SHALLOW TRUE
      GIT_PROGRESS TRUE
    )
    set(
      TRACY_STATIC
      ON
      CACHE BOOL
      "Build fetched Tracy client as a static library"
      FORCE
    )
    FetchContent_MakeAvailable(tracy)
    set_target_properties(TracyClient PROPERTIES POSITION_INDEPENDENT_CODE ON)
    if(MSVC)
      target_compile_options(TracyClient PRIVATE /W0)
    else()
      target_compile_options(TracyClient PRIVATE -w)
    endif()
  endif()
endif()

find_package(Python3 COMPONENTS Interpreter Development)

# Optional collision reference dependencies. Core DART does not require these.
if(_dart_build_collision_references)
  dart_find_package(fcl)
  dart_check_required_package(fcl "fcl")
  set(DART_HAVE_FCL TRUE CACHE BOOL "Check if fcl found." FORCE)
else()
  set(DART_HAVE_FCL FALSE CACHE BOOL "Check if fcl found." FORCE)
endif()

option(DART_SKIP_spdlog "If ON, do not use spdlog even if it is found." OFF)
mark_as_advanced(DART_SKIP_spdlog)
# spdlog is OPTIONAL for core DART, so probe for it here WITHOUT the FetchContent
# fallback in dart_find_spdlog.cmake. That fallback exists for the simulation
# module's hard requirement and is invoked later by
# dart_simulation_dependencies.cmake, which runs only when the module is
# actually added -- its add_subdirectory is gated at the top level on dart-io /
# dart-collision-native. Probing plainly here keeps core-only, offline and
# minimal/graceful-skip configurations (where simulation ends up skipped) from
# fetching spdlog during this global dependency pass, before the build has decided
# whether simulation can be built. A plain probe still sets spdlog_FOUND, which
# is all of core's DART_HAVE_spdlog logic consumes.
find_package(spdlog 1.9.2 QUIET CONFIG)

# Only fetch ODE/Bullet if the corresponding collision module is enabled
# and system libraries are not being used.
set(_dart_need_fetch_content OFF)
if(_dart_build_collision_references AND NOT DART_USE_SYSTEM_ODE)
  set(_dart_need_fetch_content ON)
endif()
if(_dart_build_collision_references AND NOT DART_USE_SYSTEM_BULLET)
  set(_dart_need_fetch_content ON)
endif()
if(_dart_need_fetch_content)
  include(FetchContent)
endif()
unset(_dart_need_fetch_content)

if(_dart_build_collision_references AND NOT DART_USE_SYSTEM_ODE)
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
    URL_HASH
      SHA256=c91a28c6ff2650284784a79c726a380d6afec87ecf7a35c32a6be0c5b74513e8
    EXCLUDE_FROM_ALL
    SYSTEM
  )
  FetchContent_MakeAvailable(ode)
  # Restore CMAKE_POLICY_VERSION_MINIMUM
  if(_dart_old_policy_min)
    set(
      CMAKE_POLICY_VERSION_MINIMUM
      "${_dart_old_policy_min}"
      CACHE STRING
      ""
      FORCE
    )
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
  set(
    ODE_INCLUDE_DIRS
    "${ode_SOURCE_DIR}/include;${ode_BINARY_DIR}/include"
    CACHE STRING
    "ODE include dirs"
    FORCE
  )

  set(BUILD_SHARED_LIBS "${_dart_build_shared_libs}" CACHE BOOL "" FORCE)
  unset(_dart_build_shared_libs)
elseif(_dart_build_collision_references)
  # Using system ODE - clear any stale FetchContent cache values so dart_find_package runs fresh
  unset(ODE_FOUND CACHE)
  unset(ODE_LIBRARIES CACHE)
  unset(ODE_INCLUDE_DIRS CACHE)
  unset(DART_ODE_SOURCE_DIR CACHE)
  unset(DART_ODE_BINARY_DIR CACHE)
endif()

if(_dart_build_collision_references)
  # Skip find_package when ODE was already provided via FetchContent
  # (ODE_FOUND and ODE::ODE target already set above).
  if(NOT ODE_FOUND OR NOT TARGET ODE::ODE)
    dart_find_package(ODE)
    if(NOT ODE_FOUND)
      message(
        FATAL_ERROR
        "Collision reference tests or benchmarks are enabled but ODE (>= 0.13) "
        "was not found. Please install libode-dev or disable the collision "
        "reference test/benchmark options."
      )
    endif()
  endif()
  set(DART_HAVE_ODE TRUE CACHE BOOL "Check if ODE found." FORCE)
  set(_dart_ode_public_link ${ODE_LIBRARIES})
  if(NOT _dart_ode_public_link)
    if(TARGET ODE::ODE)
      set(_dart_ode_public_link ODE::ODE)
    else()
      message(
        FATAL_ERROR
        "ODE was found but did not export ODE::ODE or populate ODE_LIBRARIES. "
        "Please install a complete ODE package."
      )
    endif()
  endif()
else()
  set(DART_HAVE_ODE FALSE CACHE BOOL "Check if ODE found." FORCE)
endif()

if(_dart_build_collision_references AND NOT DART_USE_SYSTEM_BULLET)
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
    URL_HASH
      SHA256=c45afb6399e3f68036ddb641c6bf6f552bf332d5ab6be62f7e6c54eda05ceb77
  )
  FetchContent_MakeAvailable(bullet)
  set(
    DART_BULLET_SOURCE_DIR
    "${bullet_SOURCE_DIR}"
    CACHE INTERNAL
    "Bullet source dir."
  )
  set(
    DART_BULLET_BINARY_DIR
    "${bullet_BINARY_DIR}"
    CACHE INTERNAL
    "Bullet binary dir."
  )

  # Wire fetched Bullet into find_package flow by setting Bullet_FOUND and
  # BULLET_* variables so dart_find_package(Bullet) succeeds.
  set(Bullet_FOUND TRUE CACHE BOOL "Bullet found via FetchContent" FORCE)
  set(BULLET_FOUND TRUE CACHE BOOL "Bullet found via FetchContent" FORCE)
  set(
    BULLET_INCLUDE_DIRS
    "${bullet_SOURCE_DIR}/src"
    CACHE STRING
    "Bullet include dirs"
    FORCE
  )
  set(
    BULLET_INCLUDE_DIR
    "${bullet_SOURCE_DIR}/src"
    CACHE STRING
    "Bullet include dir"
    FORCE
  )
  # Bullet builds BulletCollision, BulletDynamics, BulletSoftBody, LinearMath targets
  set(
    BULLET_LIBRARIES
    BulletCollision
    BulletDynamics
    BulletSoftBody
    LinearMath
    CACHE STRING
    "Bullet libraries"
    FORCE
  )
  # Pre-set BT_USE_DOUBLE_PRECISION since we built Bullet with USE_DOUBLE_PRECISION=ON.
  # This avoids the check_cxx_source_compiles probe in dart/collision/bullet which
  # would fail at configure time because the Bullet targets aren't built yet.
  set(
    BT_USE_DOUBLE_PRECISION
    TRUE
    CACHE BOOL
    "Bullet double precision (FetchContent)"
    FORCE
  )

  set(BUILD_SHARED_LIBS "${_dart_build_shared_libs}" CACHE BOOL "" FORCE)
  unset(_dart_build_shared_libs)
elseif(_dart_build_collision_references)
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

if(_dart_build_collision_references)
  # Skip find_package when Bullet was already provided via FetchContent
  # (BULLET_FOUND and BULLET_LIBRARIES already set above).
  if(NOT BULLET_FOUND)
    # Force MODULE mode via dart_find_package to avoid BulletConfig.cmake
    # relative-path issues from some package distributions.
    dart_find_package(Bullet)
    if(NOT BULLET_FOUND)
      message(
        FATAL_ERROR
        "Collision reference tests or benchmarks are enabled but Bullet was not "
        "found. Please install libbullet-dev (>= 3.25) or disable the "
        "collision reference test/benchmark options."
      )
    endif()
  endif()

  # Test whether Bullet was built with double precision. If so, define
  # BT_USE_DOUBLE_PRECISION before including any Bullet headers. Skip the probe
  # if FetchContent already set the precision flag.
  if(NOT DEFINED BT_USE_DOUBLE_PRECISION)
    include(CheckCXXSourceCompiles)
    set(CMAKE_REQUIRED_FLAGS "-w")
    set(CMAKE_REQUIRED_DEFINITIONS "-DBT_USE_DOUBLE_PRECISION")
    set(CMAKE_REQUIRED_INCLUDES "${BULLET_INCLUDE_DIRS}")
    set(CMAKE_REQUIRED_LIBRARIES "${BULLET_LIBRARIES}")
    check_cxx_source_compiles(
      "
      #include <btBulletCollisionCommon.h>
      int main()
      {
        btVector3 v(0., 0., 1.);
        btStaticPlaneShape planeShape(v, 0.);
        return 0;
      }
      "
      BT_USE_DOUBLE_PRECISION
    )
  endif()

  if(DART_VERBOSE)
    if(BT_USE_DOUBLE_PRECISION)
      message(STATUS "Looking for Bullet - found (double precision)")
    else()
      message(STATUS "Looking for Bullet - found (single precision)")
    endif()
  endif()

  set(DART_HAVE_BULLET TRUE CACHE BOOL "Check if BULLET found." FORCE)
else()
  set(DART_HAVE_BULLET FALSE CACHE BOOL "Check if BULLET found." FORCE)
  unset(BT_USE_DOUBLE_PRECISION CACHE)
  unset(BT_USE_DOUBLE_PRECISION)
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
    # Current: v1.92.8-docking (docking branch release)
    # The docking variant provides the multi-viewport dockspace API
    # (IMGUI_HAS_DOCK) required by the standalone dartsim editor. The version
    # matches the system ImGui that DART already builds against.
    # Minimum required: v1.92. The dart::gui viewer theme uses ImGui style slots
    # and fields from the 1.89-1.91 series (e.g. ImGuiCol_NavCursor, 1.91.4) and
    # enforces this minimum at compile time in
    # dart/gui/detail/imgui_overlay.cpp; older system ImGui is not supported.
    set(IMGUI_MIN_VERSION "1.92")
    set(IMGUI_TARGET_VERSION "v1.92.8-docking")

    message(STATUS "Fetching ImGui ${IMGUI_TARGET_VERSION} from GitHub...")

    FetchContent_Declare(
      imgui
      GIT_REPOSITORY https://github.com/ocornut/imgui.git
      GIT_TAG ${IMGUI_TARGET_VERSION}
      GIT_SHALLOW TRUE
      SOURCE_DIR
      "${CMAKE_BINARY_DIR}/_deps/imgui-src"
      PATCH_COMMAND
        ${CMAKE_COMMAND} -DIMGUI_SOURCE_DIR=${CMAKE_BINARY_DIR}/_deps/imgui-src
        -P ${CMAKE_CURRENT_LIST_DIR}/patches/imgui_null_font_guard.cmake
    )

    # Populate imgui using the modern helper (avoids CMP0169 warnings)
    FetchContent_GetProperties(imgui)
    if(NOT imgui_POPULATED)
      FetchContent_MakeAvailable(imgui)
    endif()

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

    # Create the ImGui library target
    # Use a unique name to avoid conflicts with example executables
    # Note: We use dart-imgui-lib as the real target name (not an alias)
    # System ImGui provides imgui::imgui, but we can't use that name for export
    set(imgui_library_name dart-imgui-lib)
    set(imgui_component_name imgui)

    dart_add_library(${imgui_library_name}
      ${IMGUI_CORE_SOURCES}
      ${IMGUI_CORE_HEADERS}
    )

    if(WIN32 AND BUILD_SHARED_LIBS)
      # ImGui does not export symbols by default; ensure an import library is
      # generated so dart-gui can link against dart-imgui-lib on Windows.
      set_target_properties(
        ${imgui_library_name}
        PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON
      )
    endif()

    # Configure include directories
    # Build tree: use fetched source directory
    # Install tree: use standard include paths (like system-installed imgui)
    target_include_directories(
      ${imgui_library_name}
      PUBLIC $<BUILD_INTERFACE:${imgui_SOURCE_DIR}> $<INSTALL_INTERFACE:include>
    )

    # Compiler options - suppress warnings for third-party code
    if(CMAKE_COMPILER_IS_GNUCXX)
      target_compile_options(${imgui_library_name} PRIVATE -w)
    endif()

    # Set position independent code for linking into shared libraries (e.g., Python extensions)
    # This is the modern way to add -fPIC
    set_target_properties(
      ${imgui_library_name}
      PROPERTIES POSITION_INDEPENDENT_CODE ON
    )

    # Ensure MSVC generates an import library when building shared libs.
    # Without exports, linkers can fail to find dart-imgui-lib.lib (LNK1181).
    if(MSVC AND BUILD_SHARED_LIBS)
      set_target_properties(
        ${imgui_library_name}
        PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON
      )
    endif()

    # Note: IMGUI_DISABLE_OBSOLETE_FUNCTIONS is intentionally NOT defined here.
    # The docking branch tracks ImGui 1.92.x, whose font rework marks
    # io.FontGlobalScale and ImFontAtlas::GetTexDataAsRGBA32() obsolete. DART's
    # font handling (dart/gui/detail) still uses them, matching the system-ImGui
    # path (which keeps obsolete functions available). Defining the macro would
    # remove those symbols and break the GUI build.

    # Component registration
    # Note: We use dart-imgui-lib as the real target name (not imgui::imgui with ALIAS)
    # because ALIAS targets cannot be exported by CMake install(EXPORT) commands.
    # dart-gui links directly to dart-imgui-lib when using fetched ImGui.
    add_component(${PROJECT_NAME} ${imgui_component_name})
    add_component_targets(${PROJECT_NAME} ${imgui_component_name} ${imgui_library_name})

    # Install fetched ImGui headers to standard system-like paths
    # This allows downstream projects to use standard includes like <imgui.h>
    install(FILES ${IMGUI_CORE_HEADERS} DESTINATION include COMPONENT headers)

    message(STATUS "ImGui ${IMGUI_TARGET_VERSION} fetched successfully")

    # Add install-time warning about installing fetched ImGui
    install(
      CODE
        "message(WARNING \"Installing fetched ImGui headers to \${CMAKE_INSTALL_PREFIX}/include/. If you have system ImGui installed, this may cause conflicts. For production use, consider building with -DDART_USE_SYSTEM_IMGUI=ON instead.\")"
      COMPONENT headers
    )
  endif()
endif()

# Filament GUI
if(DART_BUILD_GUI)
  if(DART_USE_SYSTEM_FILAMENT)
    set(_dart_filament_find_quietly_was_defined FALSE)
    if(DEFINED Filament_FIND_QUIETLY)
      set(_dart_filament_find_quietly_was_defined TRUE)
      set(_dart_filament_find_quietly_saved "${Filament_FIND_QUIETLY}")
    endif()
    if(DART_FETCH_FILAMENT)
      set(Filament_FIND_QUIETLY TRUE)
    endif()
    dart_find_package(Filament)
    if(_dart_filament_find_quietly_was_defined)
      set(Filament_FIND_QUIETLY "${_dart_filament_find_quietly_saved}")
    else()
      unset(Filament_FIND_QUIETLY)
    endif()
    unset(_dart_filament_find_quietly_saved)
    unset(_dart_filament_find_quietly_was_defined)
  elseif(NOT DART_FETCH_FILAMENT)
    message(
      FATAL_ERROR
      "DART_BUILD_GUI=ON requires DART_USE_SYSTEM_FILAMENT=ON unless DART_FETCH_FILAMENT=ON is explicitly set."
    )
  endif()

  if(NOT Filament_FOUND AND DART_FETCH_FILAMENT)
    message(
      STATUS
      "Filament was not found in system paths; fetching Filament ${DART_FILAMENT_VERSION}"
    )
    if(NOT DART_FILAMENT_VERSION STREQUAL "1.71.3")
      message(
        FATAL_ERROR
        "DART_FETCH_FILAMENT has a pinned hash only for DART_FILAMENT_VERSION=1.71.3. Update the URL hash before changing the version."
      )
    endif()

    set(_dart_filament_target_arch "${CMAKE_SYSTEM_PROCESSOR}")
    if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
      set(_dart_filament_target_arch "")
      foreach(
        _dart_filament_arch_candidate
        IN
        ITEMS
          CMAKE_CXX_COMPILER_ARCHITECTURE_ID
          CMAKE_GENERATOR_PLATFORM
          CMAKE_VS_PLATFORM_NAME
      )
        if(
          NOT _dart_filament_target_arch
          AND DEFINED ${_dart_filament_arch_candidate}
          AND NOT "${${_dart_filament_arch_candidate}}" STREQUAL ""
        )
          set(_dart_filament_target_arch "${${_dart_filament_arch_candidate}}")
        endif()
      endforeach()

      if(NOT _dart_filament_target_arch)
        set(_dart_filament_target_arch "${CMAKE_SYSTEM_PROCESSOR}")
      endif()
    endif()
    string(
      TOLOWER "${_dart_filament_target_arch}"
      _dart_filament_target_arch_lower
    )

    if(
      CMAKE_SYSTEM_NAME STREQUAL "Linux"
      AND CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|AMD64)$"
    )
      set(_dart_filament_archive_platform "linux")
      set(
        _dart_filament_archive_hash
        "d41963799c156e2eceff6c8f89d76ce26c3213972f63aa90add5e446a712e12e"
      )
    elseif(
      CMAKE_SYSTEM_NAME STREQUAL "Linux"
      AND CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$"
    )
      set(_dart_filament_archive_platform "arm-linux")
      set(
        _dart_filament_archive_hash
        "048b5bffffcafcec7fcfa718fe65ef512514c65c00ed954e7bf340e003c146c2"
      )
    elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
      set(_dart_filament_archive_platform "mac")
      set(
        _dart_filament_archive_hash
        "d8f253e262d731fb60f8be7d5ae6af76651bdc597d564171790bc78ac3696e04"
      )
    elseif(
      CMAKE_SYSTEM_NAME STREQUAL "Windows"
      AND _dart_filament_target_arch_lower MATCHES "^(x64|x86_64|amd64)$"
    )
      set(_dart_filament_archive_platform "windows")
      set(
        _dart_filament_archive_hash
        "67c08eb259aec39061b02b06f56bf7910ab78c97a95da03b1f83b86b61d1d7e2"
      )
    elseif(CMAKE_SYSTEM_NAME STREQUAL "Windows")
      message(
        FATAL_ERROR
        "DART_FETCH_FILAMENT has a pinned Windows Filament archive only for "
        "x64 targets, but the configured target architecture is "
        "'${_dart_filament_target_arch}'. Provide Filament_ROOT for this "
        "target architecture or disable DART_BUILD_GUI."
      )
    else()
      message(
        FATAL_ERROR
        "DART_FETCH_FILAMENT does not have a pinned Filament archive for ${CMAKE_SYSTEM_NAME}-${CMAKE_SYSTEM_PROCESSOR}. Provide Filament_ROOT or disable DART_BUILD_GUI."
      )
    endif()

    include(FetchContent)
    FetchContent_Declare(
      filament_prebuilt
      URL
        "https://github.com/google/filament/releases/download/v${DART_FILAMENT_VERSION}/filament-v${DART_FILAMENT_VERSION}-${_dart_filament_archive_platform}.tgz"
      URL_HASH SHA256=${_dart_filament_archive_hash}
      DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )
    FetchContent_GetProperties(filament_prebuilt)
    if(NOT filament_prebuilt_POPULATED)
      cmake_policy(PUSH)
      cmake_policy(SET CMP0169 OLD)
      FetchContent_Populate(filament_prebuilt)
      cmake_policy(POP)
    endif()

    set(_dart_fetched_filament_root "${filament_prebuilt_SOURCE_DIR}")
    if(EXISTS "${_dart_fetched_filament_root}/filament/include")
      set(_dart_fetched_filament_root "${_dart_fetched_filament_root}/filament")
    endif()
    set(
      Filament_ROOT
      "${_dart_fetched_filament_root}"
      CACHE PATH
      "Fetched Filament install tree"
      FORCE
    )
    unset(_dart_filament_archive_hash)
    unset(_dart_filament_archive_platform)
    unset(_dart_filament_arch_candidate)
    unset(_dart_filament_target_arch)
    unset(_dart_filament_target_arch_lower)

    dart_find_package(Filament)
  endif()

  if(NOT Filament_FOUND)
    message(
      FATAL_ERROR
      "Filament GUI was requested (DART_BUILD_GUI=ON) but Filament could not be found. Set Filament_ROOT to a Filament install tree that contains include/, lib/, and bin/matc."
    )
  endif()

  find_package(glfw3 CONFIG REQUIRED)

  if(NOT TARGET imgui::imgui AND NOT TARGET dart-imgui-lib)
    message(
      FATAL_ERROR
      "Filament GUI was requested but no ImGui target is available."
    )
  endif()
endif()

#--------------------
# Misc. dependencies
#--------------------

# Doxygen
find_package(Doxygen QUIET)
dart_check_optional_package(DOXYGEN "generating API documentation" "doxygen")
