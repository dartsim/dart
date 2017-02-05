# If you add a dependency, please add the corresponding rosdep key as a
# dependency in package.xml.

#======================================
# Mandatory dependencies for DART core
#======================================
if(DART_VERBOSE)
  message(STATUS "")
  message(STATUS "[ Mandatory dependencies for DART core ]")
endif()

# Eigen
find_package(EIGEN3 3.0.5 REQUIRED)
dart_check_required_package(EIGEN3 "eigen3")

# CCD
find_package(CCD 1.4.0 REQUIRED)
dart_check_required_package(CCD "libccd")

# FCL
find_package(FCL 0.2.9 REQUIRED)
dart_check_required_package(FCL "fcl")

# ASSIMP
find_package(ASSIMP 3.0.0 REQUIRED)
dart_check_required_package(ASSIMP "assimp")
if(ASSIMP_FOUND)
  # Check for missing symbols in ASSIMP (see #451)
  include(CheckCXXSourceCompiles)
  set(CMAKE_REQUIRED_DEFINITIONS "")
  set(CMAKE_REQUIRED_FLAGS "")
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
  ASSIMP_AISCENE_CTOR_DTOR_DEFINED)

  if(NOT ASSIMP_AISCENE_CTOR_DTOR_DEFINED)
    if(DART_VERBOSE)
      message(WARNING "The installed version of ASSIMP (${ASSIMP_VERSION}) is "
                      "missing symbols for the constructor and/or destructor of "
                      "aiScene. DART will use its own implementations of these "
                      "functions. We recommend using a version of ASSIMP that "
                      "does not have this issue, once one becomes available.")
    endif()
  endif(NOT ASSIMP_AISCENE_CTOR_DTOR_DEFINED)

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
  ASSIMP_AIMATERIAL_CTOR_DTOR_DEFINED)

  if(NOT ASSIMP_AIMATERIAL_CTOR_DTOR_DEFINED)
    if(DART_VERBOSE)
      message(WARNING "The installed version of ASSIMP (${ASSIMP_VERSION}) is "
                      "missing symbols for the constructor and/or destructor of "
                      "aiMaterial. DART will use its own implementations of "
                      "these functions. We recommend using a version of ASSIMP "
                      "that does not have this issue, once one becomes available.")
    endif()
  endif(NOT ASSIMP_AIMATERIAL_CTOR_DTOR_DEFINED)

  unset(CMAKE_REQUIRED_INCLUDES)
  unset(CMAKE_REQUIRED_LIBRARIES)
endif()

# Boost
set(DART_MIN_BOOST_VERSION 1.46.0 CACHE INTERNAL "Boost min version requirement" FORCE)
if(MSVC)
  add_definitions(-DBOOST_ALL_NO_LIB)
endif()
add_definitions(-DBOOST_TEST_DYN_LINK)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
if(DART_VERBOSE)
  find_package(Boost ${DART_MIN_BOOST_VERSION} REQUIRED COMPONENTS regex system)
else()
  find_package(Boost ${DART_MIN_BOOST_VERSION} QUIET REQUIRED COMPONENTS regex system)
endif()

#===========================================
# Optional dependencies for DART components
#===========================================
if(DART_VERBOSE)
  message(STATUS "")
  message(STATUS "[ Optional dependencies for DART components ]")
endif()

# OpenMP
if(ENABLE_OPENMP)
  find_package(OpenMP QUIET)
  if(DART_VERBOSE)
    if(OPENMP_FOUND)
      message(STATUS "Looking for OpenMP - found")
    else()
      message(STATUS "Looking for OpenMP - NOT found")
    endif()
  endif()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

#---------------------------------
# Dependencies for dart-optimizer
#---------------------------------

# NLOPT
find_package(NLOPT 2.4.1 QUIET)
dart_check_optional_package(NLOPT "dart-optimizer-nlopt" "nlopt" "2.4.1")

# IPOPT
find_package(IPOPT 3.11.9 QUIET)
dart_check_optional_package(IPOPT "dart-optimizer-ipopt" "ipopt" "3.11.9")

# Shark
find_package(SHARK QUIET)
if(SHARK_FOUND)
  if(DART_VERBOSE)
    message(STATUS "Looking for SHARK - ${SHARK_VERSION} found")
  endif()
  set(HAVE_SHARK TRUE)
else()
  # message(STATUS "Looking for SHARK - NOT found, please install SHARK (http://image.diku.dk/shark)")
  # TODO(JS): Disabled since not used anywhere for now
  set(HAVE_SHARK FALSE)
endif()

#---------------------------------
# Dependencies for dart-collision
#---------------------------------

# Bullet. Force MODULE mode to use the FindBullet.cmake file distributed with
# CMake. Otherwise, we may end up using the BulletConfig.cmake file distributed
# with Bullet, which uses relative paths and may break transitive dependencies.
find_package(Bullet COMPONENTS BulletMath BulletCollision MODULE QUIET)
if(BULLET_FOUND)
  # Test whether Bullet was built with double precision. If so, we need to
  # define the BT_USE_DOUBLE_PRECISION pre-processor directive before including
  # any Bullet headers. This is a workaround for the fact that Bullet does not
  # add the definition to BULLET_DEFINITIONS or generate a #cmakedefine header.
  include(CheckCXXSourceCompiles)
  set(CMAKE_REQUIRED_FLAGS "")
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

  if(DART_VERBOSE)
    if(BT_USE_DOUBLE_PRECISION)
      message(STATUS "Looking for Bullet - found (double precision)")
    else()
      message(STATUS "Looking for Bullet - found (single precision)")
    endif()
  endif()

  set(HAVE_BULLET_COLLISION TRUE)
else()
  message(STATUS "Looking for Bullet - NOT found, to use dart-collision-bullet, please install libbullet-dev")
  set(HAVE_BULLET_COLLISION FALSE)
endif()

#--------------------------------
# Dependencies for dart-planning
#--------------------------------

# FLANN
find_package(FLANN 1.8.4 QUIET)
dart_check_optional_package(FLANN "dart-planning" "flann" "1.8.4")

#-----------------------------
# Dependencies for dart-utils
#-----------------------------

# TINYXML
find_package(TINYXML 2.6.2 QUIET)
dart_check_optional_package(TINYXML "dart-utils" "tinyxml" "2.6.2")

# TINYXML2
find_package(TINYXML2 QUIET)
dart_check_optional_package(TINYXML2 "dart-utils" "tinyxml2" "1.0.1")

# urdfdom
find_package(urdfdom QUIET)
if(urdfdom_FOUND)
  if(DART_VERBOSE)
    message(STATUS "Looking for urdfdom - ${urdfdom_headers_VERSION} found")
  endif()
else()
  message(STATUS "Looking for urdfdom - NOT found, please install liburdfdom-dev")
endif()
if(MSVC)
  set(urdfdom_LIBRARIES optimized urdfdom_sensor      debug urdfdom_sensord
                        optimized urdfdom_model_state debug urdfdom_model_stated
                        optimized urdfdom_model       debug urdfdom_modeld
                        optimized urdfdom_world       debug urdfdom_worldd
                        optimized console_bridge      debug console_bridged)
endif()

#---------------------------
# Dependencies for dart-gui
#---------------------------

# OpenGL
find_package(OpenGL QUIET)
dart_check_optional_package(OPENGL "dart-gui" "OpenGL")

# GLUT
if(WIN32 AND NOT CYGWIN)
  set(GLUT_INCLUDE_DIR "@CMAKE_INSTALL_PREFIX@/include")
  set(GLUT_LIBRARIES glut32)
  set(HAVE_GLUT TRUE)
else()
  find_package(GLUT QUIET)
  dart_check_optional_package(GLUT "dart-gui" "freeglut3")
  if(GLUT_FOUND)
    set(GLUT_LIBRARIES ${GLUT_glut_LIBRARY})
    set(HAVE_GLUT TRUE)
  else()
    set(HAVE_GLUT FALSE)
  endif()
endif()

# OpenSceneGraph
if(DART_BUILD_GUI_OSG)

  find_package(OpenSceneGraph 3.0 QUIET
    COMPONENTS osg osgViewer osgManipulator osgGA osgDB)
  if(OPENSCENEGRAPH_FOUND)
    if(DART_VERBOSE)
      message(STATUS "Looking for OpenSceneGraph - ${OPENSCENEGRAPH_VERSION} found")
    endif()
    set(HAVE_OPENSCENEGRAPH TRUE)
  else(OPENSCENEGRAPH_FOUND)
    # dart-gui-osg requires both OSG and OpenThreads. This section attempts to
    # identify which of those are missing from the building machine and offer
    # advice to the user for getting dart-gui-osg to build.
    find_package(OpenThreads QUIET)
    if(OPENTHREADS_FOUND)
      set(warning_msg "Could NOT find OpenSceneGraph")
    else(OPENTHREADS_FOUND)
      if(OSG_LIBRARY)
        set(warning_msg "Could NOT find OpenThreads")
      else(OSG_LIBRARY)
        set(warning_msg "Could NOT find OpenSceneGraph nor OpenThreads")
      endif(OSG_LIBRARY)
    endif(OPENTHREADS_FOUND)
    message(WARNING "${warning_msg} -- we will skip dart-gui-osg\n"
            "If you believe you do have both OSG and OpenThreads installed, try setting OSG_DIR")
    set(HAVE_OPENSCENEGRAPH FALSE)
  endif(OPENSCENEGRAPH_FOUND)

else()

  message(STATUS "Skipping OpenSceneGraph (DART_BUILD_GUI_OSG == ${DART_BUILD_GUI_OSG})")
  set(HAVE_OPENSCENEGRAPH FALSE)

endif(DART_BUILD_GUI_OSG)

#--------------------
# Misc. dependencies
#--------------------

# Perl modules
find_package(PerlModules COMPONENTS Regexp::Common Getopt::ArgvFile Getopt::Long Term::ANSIColor QUIET)
if(DART_VERBOSE)
  if("${PERLMODULES_FOUND}" STREQUAL "TRUE")
    message(STATUS "Looking for PerlModules - found")
  else()
    message(STATUS "Looking for PerlModules - NOT found, to colorize gcc messages, please install Regexp::Common Getopt::ArgvFile Getopt::Long Term::ANSIColor (http://www.cpan.org/modules/INSTALL.html)")
  endif()
endif()

# Doxygen
find_package(Doxygen QUIET)
dart_check_optional_package(DOXYGEN "generating API documentation" "doxygen")
