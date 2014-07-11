# A list of applications
set_property(DIRECTORY PROPERTY FOLDER Apps)

# List of all the subdirectories to include
foreach(APPDIR
    addDeleteSkels
    atlasSimbicon
    bipedStand
    rigidLoop
    rigidCubes
    rigidChain
    hardcodedDesign
    jointConstraints
    mixedChain
    softBodies
    vehicle
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
