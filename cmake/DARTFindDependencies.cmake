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
# Keep the installed fmt finder unchanged; only DART builds opt into fetching.
if(
  DEFINED DART_USE_SYSTEM_FMT
  AND NOT DART_USE_SYSTEM_FMT
  AND NOT TARGET fmt::fmt
)
  include(FetchContent)
  dart_check_fetchcontent(fmt DART_USE_SYSTEM_FMT)
endif()
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
    # Installed Tracy packages ship no CMake version file; DART expects 0.11.
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
    dart_check_fetchcontent(tracy DART_USE_SYSTEM_TRACY)
    FetchContent_MakeAvailable(tracy)
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
  dart_check_fetchcontent(ode DART_USE_SYSTEM_ODE)
  dart_fetchcontent_makeavailable_legacy(ode)
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
  )
  dart_check_fetchcontent(bullet DART_USE_SYSTEM_BULLET)
  dart_fetchcontent_makeavailable_legacy(bullet)
  # Skip upstream's src install rules, including the unused dynamics headers.
  set_property(
    DIRECTORY "${bullet_SOURCE_DIR}/src"
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
    string(
      REPLACE
      "libdir=\${exec_prefix}/${LIB_DESTINATION}"
      "libdir=\${exec_prefix}/${LIBRARY_INSTALL_DIR}"
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

  set(BUILD_SHARED_LIBS "${_dart_build_shared_libs}" CACHE BOOL "" FORCE)
  unset(_dart_build_shared_libs)
endif()

#--------------------
# Misc. dependencies
#--------------------

# Doxygen
find_package(Doxygen QUIET)
dart_check_optional_package(DOXYGEN "generating API documentation" "doxygen")
