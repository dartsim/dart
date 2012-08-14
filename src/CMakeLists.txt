# all libraries
ADD_SUBDIRECTORY(utils)
ADD_SUBDIRECTORY(kinematics)
ADD_SUBDIRECTORY(optimizer)
ADD_SUBDIRECTORY(yui)
ADD_SUBDIRECTORY(renderer)
ADD_SUBDIRECTORY(geometry)
ADD_SUBDIRECTORY(dynamics)
ADD_SUBDIRECTORY(lcpsolver)
ADD_SUBDIRECTORY(collision)
ADD_SUBDIRECTORY(integration)
if (PLANNING)
    message( STATUS "Adding planning library")
    ADD_SUBDIRECTORY(planning)
endif (PLANNING)

# Unit Test
ADD_SUBDIRECTORY(unittests)

# Apps
ADD_SUBDIRECTORY(apps)

