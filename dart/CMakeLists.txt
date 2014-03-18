# Enable multi-threaded compilation.
# We do this here and not in the root folder since the example apps
# do not have enough source files to benefit from this.
if(MSVC)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
endif()

set(CORE_SUBDIRS common math dynamics renderer lcpsolver constraint integration simulation)
set(NONCORE_SUBDIRS gui planning utils)
# optimizer is also a NONCORE_SUBDIR, but it will be handled separately

# Search all header and source files
foreach(SUBDIR ${CORE_SUBDIRS})
  file(GLOB_RECURSE subdir_srcs ${SUBDIR}/*.cpp ${SUBDIR}/*.h)
  set(core_srcs ${core_srcs} ${subdir_srcs})
endforeach()

file(GLOB subdir_srcs optimizer/*.cpp optimizer/*.h)
set(core_srcs ${core_srcs} ${subdir_srcs})

file(GLOB_RECURSE subdir_srcs optimizer/nlopt/*.cpp optimizer/nlopt/*.h)
set(core_srcs ${core_srcs} ${subdir_srcs})

if(HAVE_IPOPT)
  file(GLOB_RECURSE subdir_srcs optimizer/ipopt/*.cpp optimizer/ipopt/*.h)
  set(core_srcs ${core_srcs} ${subdir_srcs})
endif()

if(HAVE_SNOPT)
  file(GLOB_RECURSE subdir_srcs optimizer/snopt/*.cpp optimizer/snopt/*.h)
  set(core_srcs ${core_srcs} ${subdir_srcs})
endif()

file(GLOB subdir_srcs collision/*.cpp collision/*.h)
set(core_srcs ${core_srcs} ${subdir_srcs})

file(GLOB_RECURSE subdir_srcs collision/fcl_mesh/*.cpp collision/fcl_mesh/*.h)
set(core_srcs ${core_srcs} ${subdir_srcs})

file(GLOB_RECURSE subdir_srcs collision/fcl/*.cpp collision/fcl/*.h)
set(core_srcs ${core_srcs} ${subdir_srcs})

file(GLOB_RECURSE subdir_srcs collision/dart/*.cpp collision/dart/*.h)
set(core_srcs ${core_srcs} ${subdir_srcs})

if(HAVE_BULLET_COLLISION)
  file(GLOB_RECURSE subdir_srcs collision/bullet/*.cpp collision/bullet/*.h)
  set(core_srcs ${core_srcs} ${subdir_srcs})
endif()

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
target_link_libraries(dart-core ${DART_CORE_DEPENDENCIES})
set_target_properties(dart-core PROPERTIES OUTPUT_NAME "dart-core${DART_MAJOR_VERSION}"
                                           SOVERSION ${DART_MINOR_VERSION}
                                           VERSION "${DART_MINOR_VERSION}.${DART_PATCH_VERSION}.0")

if(NOT BUILD_CORE_ONLY)
  add_library(dart ${noncore_srcs})
  target_link_libraries(dart dart-core ${DART_DEPENDENCIES})
  set_target_properties(dart PROPERTIES OUTPUT_NAME "dart${DART_MAJOR_VERSION}"
                                        SOVERSION ${DART_MINOR_VERSION}
                                        VERSION "${DART_MINOR_VERSION}.${DART_PATCH_VERSION}.0")
endif()


if(MSVC)
    set_target_properties(${target} PROPERTIES STATIC_LIBRARY_FLAGS_RELEASE "/LTCG")
endif()

install(DIRECTORY ./ DESTINATION include/dart FILES_MATCHING PATTERN "*.h")
install(TARGETS dart-core EXPORT DARTCoreTargets DESTINATION lib)
if(NOT BUILD_CORE_ONLY)
    install(TARGETS dart-core dart EXPORT DARTTargets DESTINATION lib)
endif()
