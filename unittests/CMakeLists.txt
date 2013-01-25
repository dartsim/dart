###############################################
# Unittests
project(unittests)

if(FALSE)

    file(GLOB gtest_hdrs "/usr/include/gtest/*.h" "/usr/include/gtest/internal/*.h")
    include_directories(/usr/include/gtest)
    include_directories(/usr/src/gtest)
    add_library(gtest /usr/src/gtest/src/gtest-all.cc ${gtest_hdrs})
    add_library(gtest_main /usr/src/gtest/src/gtest_main.cc ${gtest_hdrs})
    target_link_libraries(gtest_main gtest)
    set_target_properties (gtest_main gtest PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)


# Search all header and source files
file(GLOB unittests_srcs "*.cpp")
file(GLOB unittests_hdrs "*.h")

add_executable(unittests ${unittests_srcs} ${unittests_hdrs})
target_link_libraries(unittests dart gtest ${DARTExt_LIBRARIES})

if(APPLE OR UNIX)
  target_link_libraries(unittests pthread)
endif()

set_target_properties(unittests PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")

endif()
