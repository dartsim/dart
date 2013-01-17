# A list of applications
set_property(DIRECTORY PROPERTY FOLDER Apps)
set_property(DIRECTORY PROPERTY EXCLUDE_FROM_DEFAULT_BUILD ON)

# List of all the subdirectories to include
foreach(APPDIR
    exampleCollisionWithSkelton
    meshCollision
	viewer
	forwardSim
	cubes
	motionAnalysis
	pdController
	balance
	ik
	hybrid
	)
    add_subdirectory(${APPDIR})
endforeach(APPDIR)

foreach(APPTARGET
    exampleCollisionWithSkelton
    meshCollision
	viewer
	forwardSim
	cubes
	motionAnalysis
	pdController
	balance
	ik
	hybrid
	)
    if(TARGET ${APPTARGET})
        set_target_properties(${APPTARGET} PROPERTIES
                FOLDER Apps
                #EXCLUDE_FROM_DEFAULT_BUILD ON
                )
        if(MSVC)
            set_target_properties(${APPTARGET} PROPERTIES STATIC_LIBRARY_FLAGS_RELEASE "/LTCG")
        endif()
    endif(TARGET ${APPTARGET})
endforeach(APPTARGET)
