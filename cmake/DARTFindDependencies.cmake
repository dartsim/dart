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
if(NOT DART_USE_SYSTEM_FCL)
  # Fetch FCL 0.7.0 and the double-precision libccd 2.1 it needs, build both in
  # their own CMake runs, and install them with DART. FCL's octree support
  # follows the octomap that DART uses.
  include(FetchContent)
  dart_find_package(octomap)
  block(
    SCOPE_FOR VARIABLES
    PROPAGATE DART_FCL_SEARCH_DIRS DART_FCL_INSTALL_DIRS
  )
    # Stop before downloading when the fetched build cannot fit this one.
    get_property(multi_config GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
    if(multi_config)
      message(
        FATAL_ERROR
        "DART_USE_SYSTEM_FCL=OFF needs a single-configuration generator, such as Ninja; set DART_USE_SYSTEM_FCL=ON to use an installed FCL."
      )
    endif()
    # FCL 0.7.0 compiles as C++11, which Eigen 5 does not support.
    if(Eigen3_VERSION VERSION_GREATER_EQUAL 5)
      message(
        FATAL_ERROR
        "DART_USE_SYSTEM_FCL=OFF builds FCL 0.7.0 as C++11, which Eigen ${Eigen3_VERSION} does not support; set DART_USE_SYSTEM_FCL=ON to use an installed FCL."
      )
    endif()
    # A system ODE may load an installed libccd, and one process cannot hold
    # it and the fetched double-precision libccd if their precisions differ.
    if(DART_USE_SYSTEM_ODE AND NOT DART_SKIP_ODE)
      find_file(ccd_config ccd/config.h NO_CACHE)
      if(ccd_config)
        file(STRINGS "${ccd_config}" single REGEX "^#define CCD_SINGLE")
        if(single)
          message(
            FATAL_ERROR
            "${ccd_config} is a single-precision libccd, which a system ODE may load beside the fetched double-precision one; set DART_USE_SYSTEM_ODE=OFF to fetch ODE as well, or DART_USE_SYSTEM_FCL=ON to use an installed FCL."
          )
        endif()
      endif()
    endif()
    # DART installs the staged packages as they are, under its own prefix.
    foreach(dir BINDIR INCLUDEDIR LIBDIR DATAROOTDIR)
      if(IS_ABSOLUTE "${CMAKE_INSTALL_${dir}}")
        message(
          FATAL_ERROR
          "DART_USE_SYSTEM_FCL=OFF needs a relative CMAKE_INSTALL_${dir}; set DART_USE_SYSTEM_FCL=ON to use an installed FCL."
        )
      endif()
    endforeach()
    # Populate without adding upstream targets or compiler flags to DART.
    FetchContent_Declare(
      dart_libccd
      URL https://github.com/danfis/libccd/archive/refs/tags/v2.1.tar.gz
      URL_HASH
        SHA256=542b6c47f522d581fbf39e51df32c7d1256ac0c626e7c2b41f1040d4b9d50d1e
      TLS_VERIFY ON
      DOWNLOAD_EXTRACT_TIMESTAMP FALSE
      SOURCE_SUBDIR
      dart-private-sdk
    )
    FetchContent_MakeAvailable(dart_libccd)
    dart_check_fetchcontent(dart_libccd DART_USE_SYSTEM_FCL)
    FetchContent_Declare(
      dart_fcl
      URL
        https://github.com/flexible-collision-library/fcl/archive/refs/tags/0.7.0.tar.gz
      URL_HASH
        SHA256=90409e940b24045987506a6b239424a4222e2daf648c86dd146cbcb692ebdcbc
      TLS_VERIFY ON
      DOWNLOAD_EXTRACT_TIMESTAMP FALSE
      SOURCE_SUBDIR
      dart-private-sdk
    )
    FetchContent_MakeAvailable(dart_fcl)
    dart_check_fetchcontent(dart_fcl DART_USE_SYSTEM_FCL)

    # Apply the GJK fix the Linux wheels carry, unless the tree already has it.
    set(
      patch
      "${DART_SOURCE_DIR}/docker/wheels/patches/fcl-0.7.0-gjk-zero-direction.patch"
    )
    find_package(Patch)
    if(NOT Patch_FOUND)
      message(
        FATAL_ERROR
        "DART_USE_SYSTEM_FCL=OFF needs the patch program to fix the fetched FCL; install it, or set DART_USE_SYSTEM_FCL=ON to use an installed FCL."
      )
    endif()
    execute_process(
      COMMAND "${Patch_EXECUTABLE}" -p1 -R -s -f --dry-run -i "${patch}"
      WORKING_DIRECTORY "${dart_fcl_SOURCE_DIR}"
      RESULT_VARIABLE patched
      OUTPUT_QUIET
      ERROR_QUIET
    )
    if(NOT patched EQUAL 0)
      # Check first, so that a tree the fix does not fit stays as it is.
      execute_process(
        COMMAND "${Patch_EXECUTABLE}" -p1 -s -f --dry-run -i "${patch}"
        WORKING_DIRECTORY "${dart_fcl_SOURCE_DIR}"
        RESULT_VARIABLE fits
        OUTPUT_QUIET
        ERROR_QUIET
      )
      if(NOT fits EQUAL 0)
        message(
          FATAL_ERROR
          "Cannot apply ${patch} to ${dart_fcl_SOURCE_DIR}, which must hold the FCL 0.7.0 release sources, with or without the fix. Remove a fetched copy to download it again, or set DART_USE_SYSTEM_FCL=ON to use an installed FCL."
        )
      endif()
      execute_process(
        COMMAND "${Patch_EXECUTABLE}" -p1 -s -f -i "${patch}"
        WORKING_DIRECTORY "${dart_fcl_SOURCE_DIR}"
        COMMAND_ERROR_IS_FATAL ANY
      )
    endif()

    # Build both with DART's toolchain, configuration and install layout, and
    # stage them in the build tree; DART installs the stage under its prefix.
    set(prefix "${FETCHCONTENT_BASE_DIR}/dart_fcl-sdk")
    set(ccd_dir "${CMAKE_INSTALL_LIBDIR}/ccd")
    set(fcl_dir "${CMAKE_INSTALL_LIBDIR}/cmake/fcl")
    if(WIN32 AND NOT CYGWIN)
      set(fcl_dir CMake)
    endif()
    set(rpath "$ORIGIN")
    if(APPLE)
      set(rpath "@loader_path")
    endif()
    set(cache "set(FCL_WITH_OCTOMAP OFF CACHE BOOL \"\" FORCE)\n")
    if(octomap_FOUND)
      string(
        CONCAT cache
        "set(FCL_WITH_OCTOMAP ON CACHE BOOL \"\" FORCE)\n"
        "set(octomap_DIR [==[${octomap_DIR}]==] CACHE PATH \"\" FORCE)\n"
      )
    endif()
    string(TOUPPER "${CMAKE_BUILD_TYPE}" config)
    set(cross)
    if(CMAKE_CROSSCOMPILING)
      set(cross CMAKE_SYSTEM_NAME CMAKE_SYSTEM_VERSION CMAKE_SYSTEM_PROCESSOR)
    endif()
    foreach(
      var
      CMAKE_GENERATOR_PLATFORM
      CMAKE_GENERATOR_TOOLSET
      CMAKE_GENERATOR_INSTANCE
      CMAKE_MAKE_PROGRAM
      CMAKE_TOOLCHAIN_FILE
      ${cross}
      CMAKE_SYSROOT
      CMAKE_SYSROOT_COMPILE
      CMAKE_SYSROOT_LINK
      CMAKE_FIND_ROOT_PATH
      CMAKE_FIND_ROOT_PATH_MODE_PROGRAM
      CMAKE_FIND_ROOT_PATH_MODE_LIBRARY
      CMAKE_FIND_ROOT_PATH_MODE_INCLUDE
      CMAKE_FIND_ROOT_PATH_MODE_PACKAGE
      CMAKE_C_COMPILER
      CMAKE_CXX_COMPILER
      CMAKE_C_COMPILER_ARG1
      CMAKE_CXX_COMPILER_ARG1
      CMAKE_C_COMPILER_TARGET
      CMAKE_CXX_COMPILER_TARGET
      CMAKE_C_COMPILER_EXTERNAL_TOOLCHAIN
      CMAKE_CXX_COMPILER_EXTERNAL_TOOLCHAIN
      CMAKE_C_COMPILER_LAUNCHER
      CMAKE_CXX_COMPILER_LAUNCHER
      CMAKE_LINKER
      CMAKE_AR
      CMAKE_RANLIB
      CMAKE_C_FLAGS
      CMAKE_CXX_FLAGS
      CMAKE_C_FLAGS_${config}
      CMAKE_CXX_FLAGS_${config}
      CMAKE_SHARED_LINKER_FLAGS
      CMAKE_SHARED_LINKER_FLAGS_${config}
      CMAKE_EXE_LINKER_FLAGS
      CMAKE_EXE_LINKER_FLAGS_${config}
      CMAKE_TRY_COMPILE_TARGET_TYPE
      CMAKE_MSVC_RUNTIME_LIBRARY
      CMAKE_OSX_ARCHITECTURES
      CMAKE_OSX_DEPLOYMENT_TARGET
      CMAKE_OSX_SYSROOT
      CMAKE_BUILD_TYPE
      CMAKE_INSTALL_PREFIX
      CMAKE_INSTALL_BINDIR
      CMAKE_INSTALL_INCLUDEDIR
      CMAKE_INSTALL_LIBDIR
      CMAKE_INSTALL_DATAROOTDIR
      Eigen3_DIR
    )
      if(DEFINED ${var})
        string(
          APPEND cache
          "set(${var} [==[${${var}}]==] CACHE STRING \"\" FORCE)\n"
        )
      endif()
    endforeach()
    # libccd's 2.8 minimum needs the policy floor under CMake 4, and both
    # projects predate CMP0091, without which they ignore the MSVC runtime.
    # libccd builds in double precision, and FCL finds it in the stage and
    # its own shared libraries beside it.
    string(
      APPEND cache
      "set(CMAKE_PREFIX_PATH [==[${prefix};${CMAKE_PREFIX_PATH}]==] CACHE STRING \"\" FORCE)\n"
      "set(CMAKE_POLICY_VERSION_MINIMUM 3.5 CACHE STRING \"\" FORCE)\n"
      "set(CMAKE_POLICY_DEFAULT_CMP0091 NEW CACHE STRING \"\" FORCE)\n"
      "set(BUILD_TESTING OFF CACHE BOOL \"\" FORCE)\n"
      "set(BUILD_SHARED_LIBS ON CACHE BOOL \"\" FORCE)\n"
      "set(FCL_STATIC_LIBRARY OFF CACHE BOOL \"\" FORCE)\n"
      "set(ENABLE_DOUBLE_PRECISION ON CACHE BOOL \"\" FORCE)\n"
      "set(ccd_DIR [==[${prefix}/${ccd_dir}]==] CACHE PATH \"\" FORCE)\n"
      "set(FCL_NO_DEFAULT_RPATH OFF CACHE BOOL \"\" FORCE)\n"
      "set(CMAKE_INSTALL_RPATH [==[${rpath}]==] CACHE STRING \"\" FORCE)\n"
    )
    file(WRITE "${dart_fcl_BINARY_DIR}/dart-initial-cache.cmake" "${cache}")
    set(log "${CMAKE_CURRENT_BINARY_DIR}/fcl-source-build.log")
    message(
      STATUS
      "Building fetched libccd 2.1 and FCL 0.7.0; on failure, see ${log}"
    )
    foreach(project libccd fcl)
      set(build "${dart_${project}_BINARY_DIR}")
      execute_process(
        COMMAND
          "${CMAKE_COMMAND}" -C
          "${dart_fcl_BINARY_DIR}/dart-initial-cache.cmake" -G
          "${CMAKE_GENERATOR}" -S "${dart_${project}_SOURCE_DIR}" -B "${build}"
        OUTPUT_FILE "${log}"
        ERROR_FILE "${log}"
        COMMAND_ERROR_IS_FATAL ANY
      )
      execute_process(
        COMMAND
          "${CMAKE_COMMAND}" --build "${build}" --config "${CMAKE_BUILD_TYPE}"
        OUTPUT_FILE "${log}"
        ERROR_FILE "${log}"
        COMMAND_ERROR_IS_FATAL ANY
      )
      # Stage into the build tree even when the environment sets DESTDIR.
      execute_process(
        COMMAND
          "${CMAKE_COMMAND}" -E env --unset=DESTDIR "${CMAKE_COMMAND}" --install
          "${build}" --config "${CMAKE_BUILD_TYPE}" --prefix "${prefix}"
        OUTPUT_FILE "${log}"
        ERROR_FILE "${log}"
        COMMAND_ERROR_IS_FATAL ANY
      )
    endforeach()
    # The projects write DART's install prefix into their pkg-config files;
    # make them relative to the file, as dart.pc is.
    set(pc_dir "${CMAKE_INSTALL_LIBDIR}/pkgconfig")
    file(
      RELATIVE_PATH
      up
      "${CMAKE_INSTALL_PREFIX}/${pc_dir}"
      "${CMAKE_INSTALL_PREFIX}"
    )
    foreach(pc ccd fcl)
      set(file "${prefix}/${pc_dir}/${pc}.pc")
      if(EXISTS "${file}")
        file(READ "${file}" content)
        foreach(
          line
          "prefix=\${pcfiledir}/${up}"
          "libdir=\${prefix}/${CMAKE_INSTALL_LIBDIR}"
          "includedir=\${prefix}/${CMAKE_INSTALL_INCLUDEDIR}"
        )
          string(REGEX MATCH "^[a-z]+=" key "${line}")
          string(
            REGEX REPLACE
            "(^|\n)${key}[^\n]*"
            "\\1${line}"
            content
            "${content}"
          )
        endforeach()
        file(WRITE "${file}" "${content}")
      endif()
    endforeach()
    install(DIRECTORY "${prefix}/" DESTINATION . USE_SOURCE_PERMISSIONS)

    # DARTFindfcl looks in these config directories first: the staged ones
    # here, and in installed components the ones beside them, which follow
    # the components when the package is relocated.
    set(DART_FCL_SEARCH_DIRS "${prefix}/${ccd_dir}" "${prefix}/${fcl_dir}")
    set(DART_FCL_INSTALL_DIRS)
    foreach(dir "${ccd_dir}" "${fcl_dir}")
      cmake_path(RELATIVE_PATH dir BASE_DIRECTORY "${CONFIG_INSTALL_DIR}")
      list(APPEND DART_FCL_INSTALL_DIRS "\${CMAKE_CURRENT_LIST_DIR}/${dir}")
    endforeach()
  endblock()
endif()
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

  # Core-only consumers also need to resolve the fetched ODE shared library.
  list(PREPEND CMAKE_BUILD_RPATH "$<TARGET_FILE_DIR:ODE>")
  if(CMAKE_SYSTEM_NAME MATCHES "^(Linux|FreeBSD)$")
    add_link_options("LINKER:-rpath-link,$<TARGET_FILE_DIR:ODE>")
  endif()

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
  # Core-only links and loads must prefer fetched DSOs over system Bullet's SONAMEs.
  foreach(_dart_bullet_target BulletCollision LinearMath)
    get_target_property(_dart_bullet_type ${_dart_bullet_target} TYPE)
    if(_dart_bullet_type STREQUAL "SHARED_LIBRARY")
      list(
        PREPEND CMAKE_BUILD_RPATH
        "$<TARGET_FILE_DIR:${_dart_bullet_target}>"
      )
      if(CMAKE_SYSTEM_NAME MATCHES "^(Linux|FreeBSD)$")
        add_link_options(
          "LINKER:-rpath-link,$<TARGET_FILE_DIR:${_dart_bullet_target}>"
        )
      endif()
    endif()
  endforeach()
  unset(_dart_bullet_target)
  unset(_dart_bullet_type)
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
