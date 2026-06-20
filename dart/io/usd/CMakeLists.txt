# OpenUSD (pxr) scene loader sources for dart-io.
#
# This directory is only added (from dart/io/CMakeLists.txt) when
# DART_BUILD_IO_USD=ON, so it is the single place that requires OpenUSD. It
# appends its sources to the existing dart-io target and turns on the
# DART_IO_HAS_USD front-door switch consumed by dart/io/read.cpp.

include(dart_find_pxr)

if(NOT PXR_FOUND)
  message(
    FATAL_ERROR
    "DART_BUILD_IO_USD=ON requires OpenUSD (pxr), but no pxr CMake config "
    "package was found. Install OpenUSD or set CMAKE_PREFIX_PATH so CMake can "
    "find pxrConfig.cmake, or set DART_BUILD_IO_USD=OFF."
  )
endif()

target_sources(dart-io PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/usd_parser.cpp)
target_compile_definitions(dart-io PUBLIC DART_IO_HAS_USD=1)

if(DART_PXR_INCLUDE_DIRS)
  target_include_directories(dart-io PRIVATE ${DART_PXR_INCLUDE_DIRS})
endif()
if(DART_PXR_LIBRARIES)
  target_link_libraries(dart-io PRIVATE ${DART_PXR_LIBRARIES})
endif()

install(FILES usd_parser.hpp DESTINATION include/dart/io/usd COMPONENT headers)

dart_format_add(usd_parser.hpp usd_parser.cpp)
