# A list of applications
set_property(DIRECTORY PROPERTY FOLDER Apps)

message(STATUS "")
message(STATUS "[ Applications ]")
# Automatically identify all directories in the apps folder
file(GLOB children RELATIVE ${CMAKE_CURRENT_LIST_DIR} "*")
set(directories "")
foreach(child ${children})
    if(IS_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/${child}")
        if(child MATCHES "CMakeFiles")
            # Travis-CI automatically generates
            # a directory with this name, so we
            # need to explicitly ignore it.
        else(child MATCHES "CMakeFiles")
            message(STATUS "Adding application: " ${child})
            list(APPEND directories ${child})
        endif(child MATCHES "CMakeFiles")
    endif(IS_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/${child}")
endforeach(child)

# List of all the subdirectories to include
foreach(APPDIR ${directories})
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
