# Copyright (c) 2011-2021, The DART development contributors

#===============================================================================
# Required dependencies
#===============================================================================

# fmt
find_package(fmt REQUIRED)

# Eigen3
find_package(Eigen3 3.3.4 REQUIRED CONFIG)
if(Eigen3_FOUND AND NOT TARGET Eigen3::Eigen)
  if(DART_DEBUG)
    message("Defining Eigen3::Eigen target.")
  endif()
  add_library(Eigen3::Eigen INTERFACE IMPORTED)
  set_target_properties(
    Eigen3::Eigen PROPERTIES INTERFACE_INCLUDE_DIRECTORIES
                             "${EIGEN3_INCLUDE_DIRS}"
  )
endif()

#===============================================================================
# Optional dependencies
#===============================================================================

# TBB
find_package(TBB QUIET)

# OpenMP
find_package(OpenMP QUIET)

# spdlog
find_package(spdlog 1.3.0 QUIET)

# ccd
find_package(ccd MODULE QUIET)

# fcl
find_package(fcl MODULE QUIET)

# OpenMP
find_package(OpenGL QUIET MODULE)

# OpenSceneGraph
find_package(OpenSceneGraph 3.2.3 QUIET
  COMPONENTS osg osgViewer osgManipulator osgGA osgDB osgShadow
)
if(MSVC)
  list(REMOVE_ITEM OPENSCENEGRAPH_LIBRARIES optimized debug)
endif()
if((OPENSCENEGRAPH_FOUND OR OpenSceneGraph_FOUND) AND NOT TARGET osg::osg)
  add_library(osg::osg INTERFACE IMPORTED)
  set_target_properties(osg::osg PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${OPENSCENEGRAPH_INCLUDE_DIRS}"
    INTERFACE_LINK_LIBRARIES "${OPENSCENEGRAPH_LIBRARIES}"
  )
endif()

# imgui
# find_package(imgui MODULE REQUIRED)
