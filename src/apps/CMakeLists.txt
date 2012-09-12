# A list of applications

SET_PROPERTY( DIRECTORY PROPERTY FOLDER Apps )
SET_PROPERTY( DIRECTORY PROPERTY EXCLUDE_FROM_DEFAULT_BUILD ON )

# List of all the subdirectories to include
foreach( APPDIR
	viewer
	forwardSim
	cubes
	motionAnalysis
	pdController
#	balance # No snopt
	ik
	hybrid
#	manipulator
	)
    ADD_SUBDIRECTORY( ${APPDIR} )
endforeach(APPDIR)

foreach( APPTARGET
	viewer
	forwardSim
	cubes
	motionAnalysis
	pdController
#	balance	# no snopt
	ik
	hybrid
#	manipulator
	)
    IF(TARGET ${APPTARGET})
        SET_TARGET_PROPERTIES( ${APPTARGET} PROPERTIES
                FOLDER Apps
                #EXCLUDE_FROM_DEFAULT_BUILD ON
                )
    ENDIF(TARGET ${APPTARGET})
endforeach(APPTARGET)
