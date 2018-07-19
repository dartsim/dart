if(DART_IN_SOURCE_BUILD)
  include(${CMAKE_CURRENT_SOURCE_DIR}/InSourceBuild.cmake)
  return()
endif()

set(CMAKE_DEBUG_POSTFIX "d")

add_library(wamIk SHARED ikfast71.Transform6D.4_6_9_10_11_12_f8.cpp)
target_link_libraries(wamIk PUBLIC ${DART_LIBRARIES})
target_compile_definitions(wamIk PUBLIC IKFAST_NO_MAIN IKFAST_CLIBRARY)
target_compile_options(wamIk PRIVATE -w)
set_target_properties(wamIk
  PROPERTIES
   LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
   ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)
