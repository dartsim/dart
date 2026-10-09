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

# Check only during DART's own configure; the opt-out bypasses all header lookup.
if(NOT DART_ALLOW_SINGLE_PRECISION_LIBCCD)
  # Find the ccd/config.h that compiling against FCL would use, in the
  # compiler's order: the include directories of FCL's link interface, then
  # FCL_INCLUDE_DIRS, then the compiler's implicit directories.
  # Keep what $<BUILD_INTERFACE:...> wraps (possibly a list) and drop other
  # generator expressions; $<LINK_ONLY:...> entries carry no include
  # directories anyway.
  set(_dart_fcl_ccd_targets "")
  set(_dart_fcl_pending fcl)
  while(_dart_fcl_pending)
    list(POP_FRONT _dart_fcl_pending _dart_fcl_target)
    if(
      NOT TARGET "${_dart_fcl_target}"
      OR _dart_fcl_target IN_LIST _dart_fcl_ccd_targets
    )
      continue()
    endif()
    list(APPEND _dart_fcl_ccd_targets "${_dart_fcl_target}")
    get_target_property(
      _dart_fcl_links
      "${_dart_fcl_target}"
      INTERFACE_LINK_LIBRARIES
    )
    if(_dart_fcl_links)
      string(
        REGEX REPLACE
        "\\$<BUILD_INTERFACE:([^<>]*)>"
        "\\1"
        _dart_fcl_links
        "${_dart_fcl_links}"
      )
      string(GENEX_STRIP "${_dart_fcl_links}" _dart_fcl_links)
      list(PREPEND _dart_fcl_pending ${_dart_fcl_links})
    endif()
  endwhile()

  # CMake passes normal include directories (-I) before system ones
  # (-isystem): those of imported or SYSTEM targets and those listed in
  # INTERFACE_SYSTEM_INCLUDE_DIRECTORIES. Each group keeps the walk's order.
  set(_dart_fcl_ccd_normal_includes "")
  set(_dart_fcl_ccd_system_includes "")
  foreach(_dart_fcl_target IN LISTS _dart_fcl_ccd_targets)
    foreach(
      _dart_fcl_property
      IN
      ITEMS INTERFACE_INCLUDE_DIRECTORIES INTERFACE_SYSTEM_INCLUDE_DIRECTORIES
    )
      get_target_property(
        _dart_fcl_dirs
        "${_dart_fcl_target}"
        ${_dart_fcl_property}
      )
      if(_dart_fcl_dirs)
        string(
          REGEX REPLACE
          "\\$<BUILD_INTERFACE:([^<>]*)>"
          "\\1"
          _dart_fcl_dirs
          "${_dart_fcl_dirs}"
        )
        string(GENEX_STRIP "${_dart_fcl_dirs}" _dart_fcl_dirs)
      else()
        set(_dart_fcl_dirs "")
      endif()
      set(_dart_fcl_${_dart_fcl_property} "${_dart_fcl_dirs}")
    endforeach()
    get_target_property(_dart_fcl_imported "${_dart_fcl_target}" IMPORTED)
    get_target_property(
      _dart_fcl_no_system
      "${_dart_fcl_target}"
      IMPORTED_NO_SYSTEM
    )
    get_target_property(_dart_fcl_system "${_dart_fcl_target}" SYSTEM)
    foreach(_dart_fcl_include IN LISTS _dart_fcl_INTERFACE_INCLUDE_DIRECTORIES)
      if(
        (_dart_fcl_imported AND NOT _dart_fcl_no_system)
        OR _dart_fcl_system
        OR
          _dart_fcl_include
            IN_LIST
            _dart_fcl_INTERFACE_SYSTEM_INCLUDE_DIRECTORIES
      )
        list(APPEND _dart_fcl_ccd_system_includes "${_dart_fcl_include}")
      else()
        list(APPEND _dart_fcl_ccd_normal_includes "${_dart_fcl_include}")
      endif()
    endforeach()
  endforeach()
  set(
    _dart_fcl_ccd_includes
    ${_dart_fcl_ccd_normal_includes}
    ${_dart_fcl_ccd_system_includes}
    ${FCL_INCLUDE_DIRS}
  )
  # CMake doesn't pass the compiler's implicit directories as flags; the
  # compiler searches them last, in their own order.
  if(_dart_fcl_ccd_includes AND CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES)
    list(
      REMOVE_ITEM _dart_fcl_ccd_includes
      ${CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES}
    )
  endif()
  # The first directory with a ccd/config.h decides.
  set(_dart_fcl_ccd_header "")
  foreach(
    _dart_fcl_include
    IN
    LISTS _dart_fcl_ccd_includes CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES
  )
    if(EXISTS "${_dart_fcl_include}/ccd/config.h")
      set(_dart_fcl_ccd_header "${_dart_fcl_include}/ccd/config.h")
      break()
    endif()
  endforeach()
  set(_dart_fcl_ccd_single "")
  if(_dart_fcl_ccd_header)
    file(
      STRINGS "${_dart_fcl_ccd_header}"
      _dart_fcl_ccd_single
      REGEX "^[ \t]*#[ \t]*define[ \t]+CCD_SINGLE([ \t]|$)"
    )
  endif()

  if(_dart_fcl_ccd_single)
    # CPATH, CPLUS_INCLUDE_PATH and include flags in CMAKE_CXX_FLAGS can put
    # another libccd ahead of this one, so only warn when any is present.
    set(_dart_fcl_ccd_message_type FATAL_ERROR)
    if(
      NOT "$ENV{CPATH}$ENV{CPLUS_INCLUDE_PATH}" STREQUAL ""
      OR CMAKE_CXX_FLAGS MATCHES "(^|[ \t])(-I|-isystem|/I)"
    )
      set(_dart_fcl_ccd_message_type WARNING)
    endif()
    # FreeBSD's math/libccd port does spell its option DOUBLE_PECISION.
    message(
      ${_dart_fcl_ccd_message_type}
      "FCL's GJK/EPA runs in single precision because ${_dart_fcl_ccd_header} "
      "defines CCD_SINGLE, causing momentum drift on shallow contacts "
      "and weaker soft-contact push recovery. Rebuild libccd with "
      "-DENABLE_DOUBLE_PRECISION=ON and rebuild FCL against it "
      "(FreeBSD math/libccd: DOUBLE_PECISION; vcpkg: ccd[double-precision]), "
      "or skip this check with -DDART_ALLOW_SINGLE_PRECISION_LIBCCD=ON."
    )
  elseif(_dart_fcl_ccd_header)
    message(
      STATUS
      "FCL's libccd headers use double precision: ${_dart_fcl_ccd_header}"
    )
  else()
    message(
      STATUS
      "FCL's ccd/config.h was not found; libccd precision not checked."
    )
  endif()
endif()

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
  set(CMAKE_REQUIRED_INCLUDES ${ASSIMP_INCLUDE_DIRS})
  set(CMAKE_REQUIRED_LIBRARIES ${ASSIMP_LIBRARIES})

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
  endif(NOT ASSIMP_AISCENE_CTOR_DTOR_DEFINED)

  check_cxx_source_compiles(
    "
  #include <assimp/scene.h>
  int main()
  {
    (void)sizeof(((aiScene*)nullptr)->mNumSkeletons);
    (void)sizeof(((aiScene*)nullptr)->mSkeletons);
    return 0;
  }
  "
    ASSIMP_AISCENE_HAS_SKELETONS
  )

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
  endif(NOT ASSIMP_AIMATERIAL_CTOR_DTOR_DEFINED)

  unset(CMAKE_REQUIRED_FLAGS)
  unset(CMAKE_REQUIRED_INCLUDES)
  unset(CMAKE_REQUIRED_LIBRARIES)
endif()

# octomap
dart_find_package(octomap)
if(OCTOMAP_FOUND OR octomap_FOUND)
  if(NOT DEFINED octomap_VERSION)
    set(HAVE_OCTOMAP FALSE CACHE BOOL "Check if octomap found." FORCE)
    message(
      WARNING
      "Looking for octomap - octomap_VERSION is not defined, "
      "please install octomap with version information"
    )
  else()
    set(HAVE_OCTOMAP TRUE CACHE BOOL "Check if octomap found." FORCE)
    if(DART_VERBOSE)
      message(STATUS "Looking for octomap - version ${octomap_VERSION} found")
    endif()
  endif()
else()
  set(HAVE_OCTOMAP FALSE CACHE BOOL "Check if octomap found." FORCE)
  message(
    WARNING
    "Looking for octomap - NOT found, to use VoxelGridShape, "
    "please install octomap"
  )
endif()

#=======================
# Optional dependencies
#=======================

if(DART_BUILD_PROFILE AND DART_PROFILE_TRACY)
  if(DART_USE_SYSTEM_TRACY)
    find_package(Tracy CONFIG REQUIRED)
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

find_package(Python3 COMPONENTS Interpreter Development)

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

if(NOT DART_USE_SYSTEM_ODE OR NOT DART_USE_SYSTEM_BULLET)
  include(FetchContent)
endif()

if(NOT DART_USE_SYSTEM_ODE)
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
  set(DART_ODE_SOURCE_DIR "${ode_SOURCE_DIR}" CACHE INTERNAL "ODE source dir.")
  set(DART_ODE_BINARY_DIR "${ode_BINARY_DIR}" CACHE INTERNAL "ODE binary dir.")

  set(BUILD_SHARED_LIBS "${_dart_build_shared_libs}" CACHE BOOL "" FORCE)
  unset(_dart_build_shared_libs)
endif()

if(NOT DART_USE_SYSTEM_BULLET)
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
dart_check_optional_package(DOXYGEN "generating API documentation" "doxygen")
