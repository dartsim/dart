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

# Discover sdformat when IO is enabled, even if SDF is skipped, so the optional
# check can report the skip and imported targets reach sibling tests and examples.
if(NOT DART_BUILD_IO)
  set(HAVE_SDFORMAT FALSE CACHE BOOL "Check if SDFORMAT found." FORCE)
else()
  dart_find_package(sdformat)
endif()

#=======================
# Optional dependencies
#=======================

if(DART_BUILD_PROFILE AND DART_PROFILE_TRACY)
  if(DART_USE_SYSTEM_TRACY)
    # 0.14.1 is the first Tracy release whose package reports its version. Its
    # version file accepts only the same minor series, which is also the series
    # a matching viewer must come from.
    find_package(Tracy 0.14.1 CONFIG REQUIRED)
    # Since Tracy 0.14 the client library is an empty stub unless it was built
    # with TRACY_ENABLE, which the imported target then exports.
    get_target_property(
      _dart_tracy_defines
      Tracy::TracyClient
      INTERFACE_COMPILE_DEFINITIONS
    )
    if(NOT "TRACY_ENABLE" IN_LIST _dart_tracy_defines)
      message(
        FATAL_ERROR
        "Found Tracy ${Tracy_VERSION} built without TRACY_ENABLE, so its client library is empty. Rebuild Tracy with -DTRACY_ENABLE=ON, or set DART_USE_SYSTEM_TRACY=OFF to fetch it."
      )
    endif()
    unset(_dart_tracy_defines)
  else()
    include(FetchContent)
    # Extract with current timestamps on CMake 3.24+ so archive updates rebuild.
    set(dart_tracy_timestamp)
    if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.24)
      set(dart_tracy_timestamp DOWNLOAD_EXTRACT_TIMESTAMP FALSE)
    endif()
    FetchContent_Declare(
      tracy
      URL https://github.com/wolfpld/tracy/archive/refs/tags/v0.14.1.tar.gz
      URL_HASH
        SHA256=bf4af567e9c7524d07f3caa745fad02fb33bd5694f11910750382d1efbb251c1
      TLS_VERIFY ON
      ${dart_tracy_timestamp}
    )
    # Tracy 0.14 builds an empty client unless TRACY_ENABLE is set. A normal
    # variable (CMP0077) enables it in DART's scope only, leaving a parent
    # project's TRACY_ENABLE alone.
    set(TRACY_ENABLE ON)
    FetchContent_MakeAvailable(tracy)
    dart_check_fetchcontent(tracy DART_USE_SYSTEM_TRACY)
    if(MSVC)
      target_compile_options(TracyClient PRIVATE /W0)
    else()
      target_compile_options(TracyClient PRIVATE -w)
    endif()
  endif()
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
  set(BUILD_BULLET3 OFF CACHE BOOL "" FORCE)
  set(BUILD_EXTRAS OFF CACHE BOOL "" FORCE)
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
  set(INSTALL_EXTRA_LIBS OFF CACHE BOOL "" FORCE)
  set(INSTALL_CMAKE_FILES OFF CACHE BOOL "" FORCE)
  FetchContent_Declare(
    bullet
    URL https://github.com/bulletphysics/bullet3/archive/refs/tags/3.25.tar.gz
    URL_HASH
      SHA256=c45afb6399e3f68036ddb641c6bf6f552bf332d5ab6be62f7e6c54eda05ceb77
    # Populate first so provider-created targets retain their install rules.
    SOURCE_SUBDIR
    dart-private-bullet-sdk
  )
  dart_fetchcontent_makeavailable_legacy(bullet)
  dart_check_fetchcontent(bullet DART_USE_SYSTEM_BULLET)
  FetchContent_GetProperties(
    bullet
    SOURCE_DIR _dart_bullet_source_dir
    BINARY_DIR _dart_bullet_binary_dir
  )
  set(_dart_bullet_source_owned FALSE)
  if(NOT TARGET BulletCollision AND NOT TARGET LinearMath)
    if(
      NOT _dart_bullet_source_dir
      OR NOT _dart_bullet_binary_dir
      OR NOT EXISTS "${_dart_bullet_source_dir}/CMakeLists.txt"
    )
      message(
        FATAL_ERROR
        "Bullet requires complete provider targets or usable sources"
      )
    endif()
    block(SCOPE_FOR VARIABLES)
      if(
        NOT DEFINED CMAKE_POLICY_VERSION_MINIMUM
        OR CMAKE_POLICY_VERSION_MINIMUM VERSION_LESS 3.5
      )
        set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
      endif()
      add_subdirectory(
        "${_dart_bullet_source_dir}"
        "${_dart_bullet_binary_dir}"
      )
    endblock()
    set(_dart_bullet_source_owned TRUE)
  endif()
  if(NOT TARGET BulletCollision OR NOT TARGET LinearMath)
    message(
      FATAL_ERROR
      "Bullet must provide BulletCollision and LinearMath targets"
    )
  endif()
  if(_dart_bullet_source_owned)
    get_filename_component(
      _dart_bullet_expected_src
      "${_dart_bullet_source_dir}/src"
      ABSOLUTE
    )
    # Skip upstream's src install rules, including the unused dynamics headers.
    set_property(
      DIRECTORY "${_dart_bullet_expected_src}"
      PROPERTY EXCLUDE_FROM_ALL TRUE
    )
    install(
      TARGETS BulletCollision LinearMath
      RUNTIME DESTINATION bin
      LIBRARY DESTINATION ${LIBRARY_INSTALL_DIR}
      ARCHIVE DESTINATION ${LIBRARY_INSTALL_DIR}
    )
    install(
      DIRECTORY
        "${bullet_SOURCE_DIR}/src/BulletCollision"
        "${bullet_SOURCE_DIR}/src/LinearMath"
      DESTINATION ${INCLUDE_INSTALL_DIR}
      FILES_MATCHING
      PATTERN "*.h"
    )
    install(
      FILES "${bullet_SOURCE_DIR}/src/btBulletCollisionCommon.h"
      DESTINATION ${INCLUDE_INSTALL_DIR}
    )
    install(
      FILES "${bullet_SOURCE_DIR}/src/btBulletCollisionCommon.h"
      DESTINATION ${INCLUDE_INSTALL_DIR}/BulletCollision
    )
    # Installed components search these include and library directories
    # before the consumer's paths, which can list another Bullet first.
    # Relative ones follow the components when the package is relocated.
    set(_dart_bullet_config_dir "${CONFIG_INSTALL_DIR}")
    if(NOT IS_ABSOLUTE "${_dart_bullet_config_dir}")
      set(
        _dart_bullet_config_dir
        "${CMAKE_INSTALL_PREFIX}/${_dart_bullet_config_dir}"
      )
    endif()
    set(DART_BULLET_INSTALL_DIRS)
    foreach(_dart_bullet_dir "${INCLUDE_INSTALL_DIR}" "${LIBRARY_INSTALL_DIR}")
      if(NOT IS_ABSOLUTE "${_dart_bullet_dir}")
        file(
          RELATIVE_PATH
          _dart_bullet_dir
          "${_dart_bullet_config_dir}"
          "${CMAKE_INSTALL_PREFIX}/${_dart_bullet_dir}"
        )
        set(_dart_bullet_dir "\${CMAKE_CURRENT_LIST_DIR}/${_dart_bullet_dir}")
      endif()
      list(APPEND DART_BULLET_INSTALL_DIRS "${_dart_bullet_dir}")
    endforeach()
    unset(_dart_bullet_config_dir)
    unset(_dart_bullet_dir)
    if(EXISTS "${bullet_BINARY_DIR}/bullet.pc")
      file(READ "${bullet_BINARY_DIR}/bullet.pc" _dart_bullet_pc)
      if(NOT _dart_bullet_pc MATCHES "-lBulletSoftBody -lBulletDynamics ")
        message(
          WARNING
          "Cannot narrow bullet.pc to the fetched collision libraries"
        )
      endif()
      string(
        REPLACE
        "-lBulletSoftBody -lBulletDynamics "
        ""
        _dart_bullet_pc
        "${_dart_bullet_pc}"
      )
      if(IS_ABSOLUTE "${LIBRARY_INSTALL_DIR}")
        set(_dart_bullet_pc_libdir "${LIBRARY_INSTALL_DIR}")
      else()
        set(_dart_bullet_pc_libdir "\${exec_prefix}/${LIBRARY_INSTALL_DIR}")
      endif()
      string(
        REPLACE
        "libdir=\${exec_prefix}/${LIB_DESTINATION}"
        "libdir=${_dart_bullet_pc_libdir}"
        _dart_bullet_pc
        "${_dart_bullet_pc}"
      )
      unset(_dart_bullet_pc_libdir)
      if(IS_ABSOLUTE "${INCLUDE_INSTALL_DIR}")
        set(_dart_bullet_pc_includedir "${INCLUDE_INSTALL_DIR}")
      else()
        set(_dart_bullet_pc_includedir "\${prefix}/${INCLUDE_INSTALL_DIR}")
      endif()
      string(
        REGEX REPLACE
        "includedir=[^\r\n]*"
        "includedir=${_dart_bullet_pc_includedir}"
        _dart_bullet_pc
        "${_dart_bullet_pc}"
      )
      unset(_dart_bullet_pc_includedir)
      # All SDK headers live under includedir, including with an absolute root.
      string(
        REPLACE
        " -I\${prefix}/include"
        ""
        _dart_bullet_pc
        "${_dart_bullet_pc}"
      )
      file(WRITE "${bullet_BINARY_DIR}/bullet.pc" "${_dart_bullet_pc}")
      unset(_dart_bullet_pc)
    endif()
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
  else()
    # Do not retain an earlier fetched build's paths for a provider's targets.
    unset(DART_BULLET_SOURCE_DIR)
    unset(DART_BULLET_SOURCE_DIR CACHE)
    unset(DART_BULLET_BINARY_DIR CACHE)
    # Current header provenance does not grant ownership of provider installs.
    if(_dart_bullet_source_dir)
      set(DART_BULLET_SOURCE_DIR "${_dart_bullet_source_dir}")
    endif()
  endif()
  unset(_dart_bullet_source_dir)
  unset(_dart_bullet_binary_dir)
  unset(_dart_bullet_expected_src)
  unset(_dart_bullet_source_owned)

  set(BUILD_SHARED_LIBS "${_dart_build_shared_libs}" CACHE BOOL "" FORCE)
  unset(_dart_build_shared_libs)
endif()

#--------------------
# Misc. dependencies
#--------------------

# Doxygen
find_package(Doxygen QUIET)
