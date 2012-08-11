# A list of applications

SET_PROPERTY( DIRECTORY PROPERTY FOLDER Apps )
SET_PROPERTY( DIRECTORY PROPERTY EXCLUDE_FROM_DEFAULT_BUILD ON )

# List of all the subdirectories to include
# disabled:  exampleCollision
#foreach( APPDIR
#            loadModel viewer mayaExporter handTracker grab cubes biped
#            exampleUI exampleDynamics exampleSim ${SNOPT_APPS}
#            exampleCollisionWithSkelton exampleLCP exampleJointLimit )
#    ADD_SUBDIRECTORY( ${APPDIR} )
#endforeach(APPDIR)

foreach( APPDIR
            HandSPD 
             HandRedBall
             cubes
             exampleJointLimit )
    ADD_SUBDIRECTORY( ${APPDIR} )
endforeach(APPDIR)

# List of all the targets added by the subdirectories
#foreach( APPTARGET
#            loadModel viewer mayaExportMotion mayaExportSkeleton handTracker
#           grab cubes biped
#            exampleUI exampleDynamics exampleSim exampleOpt exampleIK
#            exampleCollisionWithSkelton exampleLCP exampleJointLimit )
#    IF(TARGET ${APPTARGET})
 #       SET_TARGET_PROPERTIES( ${APPTARGET} PROPERTIES
 #                               FOLDER Apps
 #                               #EXCLUDE_FROM_DEFAULT_BUILD ON
 #                               )
 #   ENDIF(TARGET ${APPTARGET})
#endforeach(APPTARGET)

foreach( APPTARGET
            HandSPD
            HandRedBall
            cubes
            exampleJointLimits )
    IF(TARGET ${APPTARGET})
        SET_TARGET_PROPERTIES( ${APPTARGET} PROPERTIES
                                FOLDER Apps
                                #EXCLUDE_FROM_DEFAULT_BUILD ON
                                )
    ENDIF(TARGET ${APPTARGET})
endforeach(APPTARGET)
