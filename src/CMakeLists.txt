project(dart)

# Enable multi-threaded compilation.
# We do this here and not in the root folder since the example apps
# do not have enough source files to benefit from this.
if(MSVC)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
endif()

set(CORE_SUBDIRS common math dynamics renderer collision lcpsolver constraint integration simulation)
set(NONCORE_SUBDIRS yui planning utils optimizer)

# Search all header and source files
foreach(SUBDIR ${CORE_SUBDIRS})
	file(GLOB_RECURSE subdir_srcs ${SUBDIR}/*.cpp ${SUBDIR}/*.h)
	set(core_srcs ${core_srcs} ${subdir_srcs})
endforeach()
foreach(SUBDIR ${NONCORE_SUBDIRS})
	file(GLOB_RECURSE subdir_srcs ${SUBDIR}/*.cpp ${SUBDIR}/*.h)
	set(noncore_srcs ${noncore_srcs} ${subdir_srcs})
endforeach()

# Setup directory structure for Visual Studio
foreach(SUBDIR ${CORE_SUBDIRS} ${NONCORE_SUBDIRS})
        file(GLOB_RECURSE dir_files ${SUBDIR}/*.cpp ${SUBDIR}/*.h)
        source_group(${SUBDIR} FILES ${dir_files})
endforeach()

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

# Library
add_library(dart-core ${core_srcs})
target_link_libraries(dart-core ${DART_CORE_DEPENDENCIES} )

add_library(dart ${noncore_srcs})
target_link_libraries(dart optimized dart-core debug dart-cored ${DART_NONCORE_DEPENDENCIES})

if(MSVC)
    set_target_properties(${target} PROPERTIES STATIC_LIBRARY_FLAGS_RELEASE "/LTCG")
endif()

install(DIRECTORY ./ DESTINATION include/dart FILES_MATCHING PATTERN "*.h")
install(TARGETS dart-core dart DESTINATION lib)
