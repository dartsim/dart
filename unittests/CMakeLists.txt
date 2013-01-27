###############################################
# Unittests
project(unittests)

if(NOT WIN32)
  if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    file(GLOB gtest_hdrs "/usr/include/gtest/*.h" "/usr/include/gtest/internal/*.h")
    include_directories(/usr/include/gtest)
    include_directories(/usr/src/gtest)
    add_library(gtest /usr/src/gtest/src/gtest-all.cc ${gtest_hdrs})
  elseif(APPLE)
    add_definitions (-DGTEST_USE_OWN_TR1_TUPLE)    
    file(GLOB gtest_hdrs "/usr/local/share/gtest/include/*.h" "/usr/local/share/gtest/include/internal/*.h")
    include_directories(/usr/local/share/gtest/include)
    include_directories(/usr/local/share/gtest)
    add_library(gtest /usr/local/share/gtest/src/gtest-all.cc ${gtest_hdrs})
  endif()    

  target_link_libraries(gtest)
  set_target_properties (gtest PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
endif()

  # Search all header and source files
  file(GLOB unittests_srcs "*.cpp")
  file(GLOB unittests_hdrs "*.h")

add_executable(unittests ${unittests_srcs} ${unittests_hdrs})
target_link_libraries(unittests optimized dart debug dartd optimized gtest debug gtestd ${DARTExt_LIBRARIES})

if(APPLE OR UNIX)
  target_link_libraries(unittests pthread)
endif()

set_target_properties(unittests PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")

