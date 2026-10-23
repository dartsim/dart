# If you add a dependency, please add the corresponding rosdep key as a
# dependency in package.xml.

#======================================
# Required dependencies for DART core
#======================================
if(DART_VERBOSE)
  message(STATUS "")
  message(STATUS "[ Required dependencies for DART core ]")
endif()

# Threads
dart_find_package(Threads)

# fmt
# Keep the installed fmt finder unchanged; only DART builds opt into fetching.
# Snapshot the finder's fetch predicate: the target exists once it has run.
set(_dart_fmt_fetched FALSE)
if(
  DEFINED DART_USE_SYSTEM_FMT
  AND NOT DART_USE_SYSTEM_FMT
  AND NOT TARGET fmt::fmt
)
  include(FetchContent)
  set(_dart_fmt_fetched TRUE)
endif()
dart_find_package(fmt)
if(_dart_fmt_fetched)
  dart_check_fetchcontent(fmt DART_USE_SYSTEM_FMT)
endif()
unset(_dart_fmt_fetched)
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

# octomap
dart_find_package(octomap)
if(OCTOMAP_FOUND OR octomap_FOUND)
  set(HAVE_OCTOMAP TRUE CACHE BOOL "Check if octomap found." FORCE)
  if(DART_VERBOSE)
    message(STATUS "Looking for octomap - version ${octomap_VERSION} found")
  endif()
else()
  set(HAVE_OCTOMAP FALSE CACHE BOOL "Check if octomap found." FORCE)
  message(
    WARNING
    "Looking for octomap - NOT found, to use VoxelGridShape, "
    "please install octomap"
  )
endif()

# Discover sdformat here so its imported dependency targets reach sibling
# executables and tests that also use its API.
if(DART_BUILD_IO)
  dart_find_package(sdformat)
endif()

#=======================
# Optional dependencies
#=======================

if(DART_BUILD_PROFILE AND DART_PROFILE_TRACY)
  if(DART_USE_SYSTEM_TRACY)
    # Installed Tracy packages ship no CMake version file, so an unversioned
    # package is accepted; a reported or header-declared version needs 0.11.1.
    find_package(Tracy CONFIG REQUIRED)
    set(_dart_tracy_version "")
    foreach(_name Tracy_VERSION TRACY_VERSION TRACY_VERSION_STRING)
      if(_dart_tracy_version STREQUAL "" AND NOT "${${_name}}" STREQUAL "")
        set(_dart_tracy_version "${${_name}}")
        set(_dart_tracy_source "${_name}")
      endif()
    endforeach()
    if(_dart_tracy_version STREQUAL "" AND TARGET Tracy::TracyClient)
      # Upstream declares the version in common/TracyVersion.hpp as an enum.
      get_target_property(
        _dart_tracy_dirs
        Tracy::TracyClient
        INTERFACE_INCLUDE_DIRECTORIES
      )
      foreach(_dir IN LISTS _dart_tracy_dirs)
        if(_dir MATCHES "^\\$<BUILD_INTERFACE:(.*)>$")
          set(_dir "${CMAKE_MATCH_1}")
        endif()
        if(
          _dart_tracy_version STREQUAL ""
          AND EXISTS "${_dir}/common/TracyVersion.hpp"
        )
          file(READ "${_dir}/common/TracyVersion.hpp" _dart_tracy_header)
          set(_dart_tracy_source "${_dir}/common/TracyVersion.hpp")
          set(_dart_tracy_parts "")
          foreach(_part Major Minor Patch)
            if(
              _dart_tracy_header MATCHES "${_part}[ \t\r\n]*=[ \t\r\n]*([0-9]+)"
            )
              list(APPEND _dart_tracy_parts "${CMAKE_MATCH_1}")
            endif()
          endforeach()
          list(LENGTH _dart_tracy_parts _dart_tracy_count)
          if(NOT _dart_tracy_count EQUAL 3)
            message(
              FATAL_ERROR
              "Cannot read the Tracy version from ${_dart_tracy_source}; DART needs Tracy >= 0.11.1."
            )
          endif()
          string(REPLACE ";" "." _dart_tracy_version "${_dart_tracy_parts}")
        endif()
      endforeach()
    endif()
    if(NOT _dart_tracy_version STREQUAL "")
      if(NOT _dart_tracy_version MATCHES "^[0-9]+(\\.[0-9]+)*")
        message(
          FATAL_ERROR
          "Cannot parse Tracy version '${_dart_tracy_version}' from ${_dart_tracy_source}; DART needs Tracy >= 0.11.1."
        )
      endif()
      if(CMAKE_MATCH_0 VERSION_LESS 0.11.1)
        message(
          FATAL_ERROR
          "Found Tracy ${_dart_tracy_version}; DART needs Tracy >= 0.11.1."
        )
      endif()
    endif()
    unset(_dart_tracy_version)
    unset(_dart_tracy_source)
    unset(_dart_tracy_dirs)
    unset(_dart_tracy_header)
    unset(_dart_tracy_parts)
    unset(_dart_tracy_count)
  else()
    include(FetchContent)
    FetchContent_Declare(
      tracy
      GIT_REPOSITORY https://github.com/wolfpld/tracy.git
      GIT_TAG v0.11.1
      GIT_SHALLOW TRUE
      GIT_PROGRESS TRUE
    )
    FetchContent_MakeAvailable(tracy)
    dart_check_fetchcontent(tracy DART_USE_SYSTEM_TRACY)
    if(MSVC)
      target_compile_options(TracyClient PRIVATE /W0)
    else()
      target_compile_options(TracyClient PRIVATE -w)
    endif()
  endif()
endif()

if(DART_BUILD_DARTPY AND Python_EXECUTABLE)
  # Share nanobind's selected interpreter with tests, examples, and tutorials.
  set(
    Python3_EXECUTABLE
    "${Python_EXECUTABLE}"
    CACHE FILEPATH
    "Python interpreter shared with nanobind"
    FORCE
  )
endif()
if(DART_BUILD_DARTPY AND DART_BUILD_WHEELS)
  # manylinux provides extension headers, but deliberately omits libpython.
  find_package(Python3 3.12 REQUIRED COMPONENTS Interpreter Development.Module)
elseif(DART_BUILD_DARTPY)
  find_package(Python3 3.12 REQUIRED COMPONENTS Interpreter Development)
else()
  find_package(Python3 3.12 COMPONENTS Interpreter Development)
endif()

# Deprecated compatibility option; DART owns its logging implementation.
option(DART_SKIP_spdlog "Deprecated compatibility option (no effect)." OFF)
mark_as_advanced(DART_SKIP_spdlog)

if(
  (NOT DART_USE_SYSTEM_ODE AND NOT DART_SKIP_ODE)
  OR (NOT DART_USE_SYSTEM_BULLET AND NOT DART_SKIP_BULLET)
)
  include(FetchContent)
endif()

if(NOT DART_USE_SYSTEM_ODE AND NOT DART_SKIP_ODE)
  # Match Ubuntu/conda-forge libode settings (libccd enabled, box-cylinder disabled).
  set(_dart_build_shared_libs "${BUILD_SHARED_LIBS}")
  set(BUILD_SHARED_LIBS ON CACHE BOOL "" FORCE)

  set(ODE_DOUBLE_PRECISION ON CACHE BOOL "" FORCE)
  set(ODE_WITH_LIBCCD ON CACHE BOOL "" FORCE)
  set(ODE_WITH_LIBCCD_SYSTEM ON CACHE BOOL "" FORCE)
  set(ODE_WITH_LIBCCD_BOX_CYL OFF CACHE BOOL "" FORCE)
  set(ODE_WITH_DEMOS OFF CACHE BOOL "" FORCE)
  set(ODE_WITH_TESTS OFF CACHE BOOL "" FORCE)
  FetchContent_Declare(
    ode
    URL https://github.com/odedevs/ode/archive/refs/tags/0.16.6.tar.gz
    URL_HASH
      SHA256=91fa10c811db757c113ba5ced02e578587f84331d51041b9fd4cadb1522fcef6
  )
  dart_fetchcontent_makeavailable_legacy(ode)
  dart_check_fetchcontent(ode DART_USE_SYSTEM_ODE)
  set(ODE_VERSION 0.16.6)
  set(DART_ODE_SOURCE_DIR "${ode_SOURCE_DIR}" CACHE INTERNAL "ODE source dir.")
  set(DART_ODE_BINARY_DIR "${ode_BINARY_DIR}" CACHE INTERNAL "ODE binary dir.")

  set(BUILD_SHARED_LIBS "${_dart_build_shared_libs}" CACHE BOOL "" FORCE)
  unset(_dart_build_shared_libs)
endif()

if(NOT DART_USE_SYSTEM_BULLET AND NOT DART_SKIP_BULLET)
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
  dart_check_fetchcontent(bullet DART_USE_SYSTEM_BULLET)
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

  set(BUILD_SHARED_LIBS "${_dart_build_shared_libs}" CACHE BOOL "" FORCE)
  unset(_dart_build_shared_libs)
endif()

#--------------------
# Misc. dependencies
#--------------------

# Doxygen
find_package(Doxygen QUIET)
