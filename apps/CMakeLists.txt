# A list of applications
set_property(DIRECTORY PROPERTY FOLDER Apps)

# List of all the subdirectories to include
foreach(APPDIR
    atlasRobot
    balance
    ballJointConstraintTest
    closedLoop
    cube
    cubes
    doublePendulumWithBase
    forwardSim
    hanging
    hardcodedDesign
    harnessTest
    #hingeJointConstraintTest
    #holding
    #hybrid
    #ik
    jointLimitTest
    meshCollision
    #motionAnalysis
    #pdController
    sdfLoader
    softArticulatedBodiesTest
    softCubes
    softDoublePendulumTest
    softDropBoxTest
    softDropBoxCOMOffsetTest
    softDropBoxLowStiffnessTest
    softDropSphereTest
    softOpenChain
    softSingleBodyTest
    softSinglePendulumTest
    vehicle
    #viewer
    #inverseDyn
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

if(HAVE_BULLET_COLLISION)
  add_subdirectory(bulletCollision)
  if(WIN32)
      if(TARGET ${APPTARGET})
          set_target_properties(${APPTARGET} PROPERTIES FOLDER Apps)
          set_target_properties(${APPTARGET} PROPERTIES STATIC_LIBRARY_FLAGS_RELEASE "/LTCG")
      endif(TARGET ${APPTARGET})
  endif(WIN32)
endif(HAVE_BULLET_COLLISION)
