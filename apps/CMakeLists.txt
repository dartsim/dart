# A list of applications
set_property(DIRECTORY PROPERTY FOLDER Apps)

# List of all the subdirectories to include
foreach(APPDIR
    atlasRobot
    meshCollision
    #viewer
    forwardSim
    cubes
    #motionAnalysis
    #pdController
    balance
    #ik
    #hybrid
    #hanging
    hardcodedDesign
    closedLoop
    doublePendulumWithBase
    sdfLoader
    ballJointConstraintTest
    #hingeJointConstraintTest
    #holding
    harnessTest
    vehicle

    softArticulatedBodiesTest
    softCubes
    softDoublePendulumTest
    softDropBoxTest
    softDropBoxCOMOffsetTest
    softDropBoxLowStiffnessTest
    softDropSphereTest
    softFeetAtlasRobot
    softOpenChain
    softSingleBodyTest
    softSinglePendulumTest






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
