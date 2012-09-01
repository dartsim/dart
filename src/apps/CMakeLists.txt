# A list of applications

SET_PROPERTY( DIRECTORY PROPERTY FOLDER Apps )
SET_PROPERTY( DIRECTORY PROPERTY EXCLUDE_FROM_DEFAULT_BUILD ON )

# List of all the subdirectories to include
# disabled:  exampleCollision
#foreach( APPDIR
#            loadModel viewer mayaExporter handTracker grab biped
#            exampleUI exampleDynamics exampleSim ${SNOPT_APPS}
#            exampleLCP )
#    ADD_SUBDIRECTORY( ${APPDIR} )
#endforeach(APPDIR)

foreach( APPDIR
            exampleCollisionWithSkelton
            superCoolSimulation
            HandSPD 
             HandRedBall
             cubes
             exampleJointLimit
             meshTest
             meshCollision
             leftLeg
             rightLeg
             lowerBodyFreeFall 
			swinging
						 lowerBodyWalk )
    ADD_SUBDIRECTORY( ${APPDIR} )
endforeach(APPDIR)

# List of all the targets added by the subdirectories
#foreach( APPTARGET
#            loadModel viewer mayaExportMotion mayaExportSkeleton handTracker
#           grab biped
#            exampleUI exampleDynamics exampleSim exampleOpt exampleIK
#            exampleLCP )
#    IF(TARGET ${APPTARGET})
 #       SET_TARGET_PROPERTIES( ${APPTARGET} PROPERTIES
 #                               FOLDER Apps
 #                               #EXCLUDE_FROM_DEFAULT_BUILD ON
 #                               )
 #   ENDIF(TARGET ${APPTARGET})
#endforeach(APPTARGET)

foreach( APPTARGET
		swinging
            exampleCollisionWithSkelton
            superCoolSimulation
            HandSPD
            HandRedBall
            cubes
            exampleJointLimits
            meshTest
            meshCollision
            leftLeg
            rightLeg
             lowerBodyFreeFall 
						lowerBodyWalk)
    IF(TARGET ${APPTARGET})
        SET_TARGET_PROPERTIES( ${APPTARGET} PROPERTIES
                                FOLDER Apps
                                #EXCLUDE_FROM_DEFAULT_BUILD ON
                                )
    ENDIF(TARGET ${APPTARGET})
endforeach(APPTARGET)
