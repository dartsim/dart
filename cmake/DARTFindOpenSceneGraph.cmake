# Copyright (c) 2011-2023, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/master/LICENSE
#
# This file is provided under the "BSD-style" License

find_package(OpenSceneGraph 3.6.5 MODULE
  COMPONENTS osg osgViewer osgManipulator osgGA osgDB osgShadow osgUtil
)

# It seems that OPENSCENEGRAPH_FOUND will inadvertently get set to true when
# OpenThreads is found, even if OpenSceneGraph is not installed. This is quite
# possibly a bug in OSG's cmake configuration file. For now, it seems that
# requiring OSG_FOUND to be true as well fixes this.
if(OPENSCENEGRAPH_FOUND AND OSG_FOUND)
  if(DART_VERBOSE)
    message(STATUS "Looking for OpenSceneGraph - ${OPENSCENEGRAPH_VERSION} found")
  endif()
else()
  # dart-gui-osg requires both OSG and OpenThreads. This section attempts to
  # identify which of those are missing from the building machine and offer
  # advice to the user for getting dart-gui-osg to build.
  find_package(OpenThreads QUIET)
  if(OPENTHREADS_FOUND)
    set(warning_msg "Could NOT find OpenSceneGraph")
  else()
    if(OSG_LIBRARY)
      set(warning_msg "Could NOT find OpenThreads")
    else()
      set(warning_msg "Could NOT find OpenSceneGraph nor OpenThreads")
    endif()
  endif()
  message(STATUS "${warning_msg} -- we will skip the OpenSceneGraph support in dart-gui\n"
          "If you believe you do have both OSG and OpenThreads installed, try setting OSG_DIR")
  return()
endif()

# OpenSceneGraph 3.6.5 and less are not compatible with macOS 10.15 (Catalina) and greater
# See:
#   - https://github.com/openscenegraph/OpenSceneGraph/issues/926
#   - https://github.com/dartsim/dart/issues/1439
if(APPLE AND OPENSCENEGRAPH_VERSION VERSION_LESS 3.7.0)
  message(WARNING "Found OpenSceneGraph ${OPENSCENEGRAPH_VERSION}, but use >= 3.7.0 that is compatible with macOS 10.15 and greater")
endif()

# Define an imported target "osg::osg" if it's not defined in the provided find
# module.
#
# This is required to make the DART module that uses OpenSceneGraph relocatable.
# For that, any DART targets (e.g., dart-gui-osg) should use "osg::osg" instead
# of OPENSCENEGRAPH_INCLUDE_DIRS and OPENSCENEGRAPH_LIBRARIES because the
# variables contain absolute paths of OpenSceneGraph that could be different in
# where the system that DART is built and where the system that consumes DART.
if((OPENSCENEGRAPH_FOUND OR OpenSceneGraph_FOUND) AND NOT TARGET osg::osg)
  add_library(osg::osg INTERFACE IMPORTED)
  target_include_directories(osg::osg INTERFACE ${OPENSCENEGRAPH_INCLUDE_DIRS})
  target_link_libraries(osg::osg INTERFACE ${OPENSCENEGRAPH_LIBRARIES})
endif()
