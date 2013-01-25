project(dart)

# Search all header and source files
file(GLOB_RECURSE dart_srcs "*.cpp")
file(GLOB_RECURSE dart_hdrs "*.h")

# Library 
add_library(dart ${dart_srcs} ${dart_hdrs})
target_link_libraries(dart ${DARTExt_LIBRARIES})
set_target_properties(dart PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib"
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/lib")
if(MSVC)
    set_target_properties(${target} PROPERTIES STATIC_LIBRARY_FLAGS_RELEASE "/LTCG")
endif()

install(DIRECTORY ./ DESTINATION include/dart 
        FILES_MATCHING PATTERN "*.h*" 
        PATTERN "*CMake*" EXCLUDE)
        
install(TARGETS dart DESTINATION lib)