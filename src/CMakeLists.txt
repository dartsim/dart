function(add_library_to_dart target source headers deps)
    add_library(${target} ${source} ${headers})
    target_link_libraries(${target} ${deps})
    set_target_properties(${target} PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib"
        ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib")
    if(MSVC)
        set_target_properties(${target} PROPERTIES STATIC_LIBRARY_FLAGS_RELEASE "/LTCG")
    endif()
    install(TARGETS ${target} DESTINATION lib)
endfunction() 

add_subdirectory(utils)
add_subdirectory(kinematics)
add_subdirectory(optimizer)
add_subdirectory(yui)
add_subdirectory(renderer)
add_subdirectory(dynamics)
add_subdirectory(lcpsolver)
add_subdirectory(collision)
add_subdirectory(integration)

if(BUILD_ROBOTICS)
    add_subdirectory(planning)
    add_subdirectory(robotics)
endif()

# Unit Test
add_subdirectory(unittests)

# Apps
add_subdirectory(apps)

install(DIRECTORY ./ DESTINATION include/dart 
                FILES_MATCHING PATTERN "*.h*" 
                PATTERN "*CMake*" EXCLUDE
                PATTERN "apps" EXCLUDE
                PATTERN "unittests" EXCLUDE)