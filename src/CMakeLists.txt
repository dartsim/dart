# all libraries
add_subdirectory(utils)
add_subdirectory(kinematics)
add_subdirectory(optimizer)

if(DART_UI)
	add_subdirectory(yui)
endif(DART_UI)

add_subdirectory(renderer)
add_subdirectory(dynamics)
add_subdirectory(lcpsolver)
add_subdirectory(collision)
add_subdirectory(integration)

if(PLANNING)
    message(STATUS "Adding planning library")
    add_subdirectory(planning)
endif()

if(ROBOTICS)
   message(STATUS "Adding robotics library")
   add_subdirectory(robotics)
endif()

# Unit Test
add_subdirectory(unittests)

# Apps
add_subdirectory(apps)

