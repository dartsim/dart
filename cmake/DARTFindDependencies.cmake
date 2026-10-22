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
    find_package(Tracy 0.11.1 CONFIG REQUIRED)
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
    if(MSVC)
      target_compile_options(TracyClient PRIVATE /W0)
    else()
      target_compile_options(TracyClient PRIVATE -w)
    endif()
  endif()
endif()

option(DART_SKIP_spdlog "If ON, do not use spdlog even if it is found." OFF)
mark_as_advanced(DART_SKIP_spdlog)
if(NOT DART_SKIP_spdlog)
  dart_find_package(spdlog)
else()
  # dart/CMakeLists.txt keys DART_HAVE_spdlog and the exported package
  # dependency off spdlog_FOUND, so reset a value inherited from a parent
  # project too.
  set(spdlog_FOUND FALSE)
endif()

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

  set(ODE_WITH_LIBCCD ON CACHE BOOL "" FORCE)
  set(ODE_WITH_LIBCCD_SYSTEM ON CACHE BOOL "" FORCE)
  set(ODE_WITH_LIBCCD_BOX_CYL OFF CACHE BOOL "" FORCE)
  set(ODE_WITH_DEMOS OFF CACHE BOOL "" FORCE)
  set(ODE_WITH_TESTS OFF CACHE BOOL "" FORCE)
  FetchContent_Declare(
    ode
    URL https://bitbucket.org/odedevs/ode/downloads/ode-0.16.6.tar.gz
    URL_HASH
      SHA256=c91a28c6ff2650284784a79c726a380d6afec87ecf7a35c32a6be0c5b74513e8
  )
  FetchContent_MakeAvailable(ode)
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
