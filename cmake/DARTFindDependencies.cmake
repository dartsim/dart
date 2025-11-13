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
if(ASSIMP_FOUND)
  # Check for missing symbols in ASSIMP (see #451)
  include(CheckCXXSourceCompiles)
  set(CMAKE_REQUIRED_DEFINITIONS "")
  if(MSVC)
    set(CMAKE_REQUIRED_FLAGS "-w")
  else()
    set(CMAKE_REQUIRED_FLAGS "-std=c++11 -w")
  endif()
  set(CMAKE_REQUIRED_INCLUDES "${ASSIMP_INCLUDE_DIRS}")
  set(CMAKE_REQUIRED_LIBRARIES "${ASSIMP_LIBRARIES}")

  check_cxx_source_compiles(
    "
    #include <assimp/scene.h>
    int main()
    {
      aiScene* scene = new aiScene;
      delete scene;
      return 1;
    }
    "
    ASSIMP_AISCENE_CTOR_DTOR_DEFINED
  )

  if(NOT ASSIMP_AISCENE_CTOR_DTOR_DEFINED)
    if(DART_VERBOSE)
      message(
        WARNING
        "The installed version of ASSIMP (${ASSIMP_VERSION}) is "
        "missing symbols for the constructor and/or destructor of "
        "aiScene. DART will use its own implementations of these "
        "functions. We recommend using a version of ASSIMP that "
        "does not have this issue, once one becomes available."
      )
    endif()
  endif()

  check_cxx_source_compiles(
    "
    #include <assimp/material.h>
    int main()
    {
      aiMaterial* material = new aiMaterial;
      delete material;
      return 1;
    }
    "
    ASSIMP_AIMATERIAL_CTOR_DTOR_DEFINED
  )

  if(NOT ASSIMP_AIMATERIAL_CTOR_DTOR_DEFINED)
    if(DART_VERBOSE)
      message(
        WARNING
        "The installed version of ASSIMP (${ASSIMP_VERSION}) is "
        "missing symbols for the constructor and/or destructor of "
        "aiMaterial. DART will use its own implementations of "
        "these functions. We recommend using a version of ASSIMP "
        "that does not have this issue, once one becomes available."
      )
    endif()
  endif()

  unset(CMAKE_REQUIRED_FLAGS)
  unset(CMAKE_REQUIRED_INCLUDES)
  unset(CMAKE_REQUIRED_LIBRARIES)
endif()

# octomap
dart_find_package(octomap)
if(OCTOMAP_FOUND OR octomap_FOUND)
  if(NOT DEFINED octomap_VERSION)
    set(DART_HAVE_OCTOMAP FALSE CACHE BOOL "Check if octomap found." FORCE)
    message(WARNING "Looking for octomap - octomap_VERSION is not defined, "
        "please install octomap with version information"
    )
  else()
    set(DART_HAVE_OCTOMAP TRUE CACHE BOOL "Check if octomap found." FORCE)
    if(DART_VERBOSE)
      message(STATUS "Looking for octomap - version ${octomap_VERSION} found")
    endif()
  endif()
else()
  set(DART_HAVE_OCTOMAP FALSE CACHE BOOL "Check if octomap found." FORCE)
  message(WARNING "Looking for octomap - NOT found, to use VoxelGridShape, "
      "please install octomap"
  )
endif()

#=======================
# Optional dependencies
#=======================

if(DART_BUILD_PROFILE)
  if(DART_USE_SYSTEM_TRACY)
    find_package(Tracy CONFIG REQUIRED)
  else()
    include(FetchContent)
    FetchContent_Declare(tracy
      GIT_REPOSITORY https://github.com/wolfpld/tracy.git
      GIT_TAG v0.11.1
      GIT_SHALLOW TRUE
      GIT_PROGRESS TRUE
    )
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

#--------------------
# GUI dependencies
#--------------------

# ImGui
if(NOT DART_BUILD_GUI_OSG STREQUAL "OFF")
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
      GIT_TAG        ${IMGUI_TARGET_VERSION}
      GIT_SHALLOW    TRUE
      SOURCE_DIR     "${CMAKE_BINARY_DIR}/_deps/imgui-src"
    )

    # Populate imgui
    FetchContent_GetProperties(imgui)
    if(NOT imgui_POPULATED)
      FetchContent_Populate(imgui)
    endif()

    # Check OpenGL dependency for ImGui
    dart_find_package(OpenGL)
    dart_check_optional_package(OPENGL "imgui" "OpenGL")

    # Define the imgui source files
    # Core imgui files
    set(IMGUI_CORE_SOURCES
      ${imgui_SOURCE_DIR}/imgui.cpp
      ${imgui_SOURCE_DIR}/imgui_draw.cpp
      ${imgui_SOURCE_DIR}/imgui_tables.cpp
      ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    )

    set(IMGUI_CORE_HEADERS
      ${imgui_SOURCE_DIR}/imgui.h
      ${imgui_SOURCE_DIR}/imgui_internal.h
      ${imgui_SOURCE_DIR}/imconfig.h
      ${imgui_SOURCE_DIR}/imstb_rectpack.h
      ${imgui_SOURCE_DIR}/imstb_textedit.h
      ${imgui_SOURCE_DIR}/imstb_truetype.h
    )

    # Backend files - OpenGL2 backend for OSG compatibility
    set(IMGUI_BACKEND_SOURCES
      ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl2.cpp
    )

    set(IMGUI_BACKEND_HEADERS
      ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl2.h
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
      ${IMGUI_BACKEND_SOURCES}
      ${IMGUI_BACKEND_HEADERS}
    )

    # Configure include directories
    # Build tree: use fetched source directory
    # Install tree: use standard include paths (like system-installed imgui)
    target_include_directories(${imgui_library_name}
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

    # Define IMGUI_DISABLE_OBSOLETE_FUNCTIONS to avoid using deprecated APIs
    target_compile_definitions(${imgui_library_name} PUBLIC IMGUI_DISABLE_OBSOLETE_FUNCTIONS)

    # Component registration
    # Note: We use dart-imgui-lib as the real target name (not imgui::imgui with ALIAS)
    # because ALIAS targets cannot be exported by CMake install(EXPORT) commands.
    # dart-gui-osg links directly to dart-imgui-lib when using fetched ImGui.
    add_component(${PROJECT_NAME} ${imgui_component_name})
    add_component_targets(${PROJECT_NAME} ${imgui_component_name} ${imgui_library_name})

    # Install fetched ImGui headers to standard system-like paths
    # This allows downstream projects to use standard includes like <imgui.h>
    install(
      FILES ${IMGUI_CORE_HEADERS}
      DESTINATION include
      COMPONENT headers
    )
    install(
      FILES ${IMGUI_BACKEND_HEADERS}
      DESTINATION include/backends
      COMPONENT headers
    )

    message(STATUS "ImGui ${IMGUI_TARGET_VERSION} fetched successfully")

    # Add install-time warning about installing fetched ImGui
    install(CODE "message(WARNING \"Installing fetched ImGui headers to \${CMAKE_INSTALL_PREFIX}/include/. If you have system ImGui installed, this may cause conflicts. For production use, consider building with -DDART_USE_SYSTEM_IMGUI=ON instead.\")" COMPONENT headers)
  endif()
endif()

#--------------------
# Misc. dependencies
#--------------------

# Doxygen
find_package(Doxygen QUIET)
dart_check_optional_package(DOXYGEN "generating API documentation" "doxygen")
