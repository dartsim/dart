# A list of applications
set_property(DIRECTORY PROPERTY FOLDER Apps)
set_property(DIRECTORY PROPERTY EXCLUDE_FROM_DEFAULT_BUILD ON)

# List of all the subdirectories to include
foreach(APPDIR
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
    if(WIN32)
        if(TARGET ${APPTARGET})
            set_target_properties(${APPTARGET} PROPERTIES FOLDER Apps
                    #EXCLUDE_FROM_DEFAULT_BUILD ON
                    )
            set_target_properties(${APPTARGET} PROPERTIES STATIC_LIBRARY_FLAGS_RELEASE "/LTCG")
        endif(TARGET ${APPTARGET})
    endif(WIN32)
endforeach(APPDIR)
