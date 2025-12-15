# Ensure gz-physics test targets compile against the vendored GoogleTest headers
# that match the vendored gtest static library built in `test/gtest_vendor`.
#
# Some environments also provide GoogleTest headers in their global include
# paths (e.g., `${prefix}/include/gtest`). If those headers take precedence over
# gz-physics' vendored headers, the build can fail at link time due to mismatched
# internal symbols (e.g., `MakeAndRegisterTestInfo(std::string, ...)`).

if(PROJECT_NAME STREQUAL "gz-physics")
  set(_gz_physics_vendor_gtest_include "${CMAKE_SOURCE_DIR}/test/gtest_vendor/include")
  if(EXISTS "${_gz_physics_vendor_gtest_include}/gtest/gtest.h")
    # Use a build-tree copy of the headers so we can force a distinct `-I`
    # entry that reliably wins include resolution.
    set(_gtest_include_overlay "${CMAKE_BINARY_DIR}/gtest_vendor_include")
    if(NOT EXISTS "${_gtest_include_overlay}/gtest/gtest.h")
      file(MAKE_DIRECTORY "${_gtest_include_overlay}")
      file(COPY "${_gz_physics_vendor_gtest_include}/gtest" DESTINATION "${_gtest_include_overlay}")
    endif()

    include_directories(BEFORE "${_gtest_include_overlay}")
  endif()
endif()
