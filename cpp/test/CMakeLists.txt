# Copyright (c) 2011-2021, The DART development contributors

# Googletest
if(DOWNLOAD_DEPENDENT_PACKAGES)
  # Fetch GoogleTest
  dart_fetch_git_repo(
    PROJECT_NAME googletest
    WORKING_DIR ${CMAKE_CURRENT_BINARY_DIR}/.deps
    GIT_URL https://github.com/google/googletest.git
    GIT_TAG release-1.11.0
  )

  # Set options
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  set(gtest_disable_pthreads ON CACHE BOOL "" FORCE)
  set(BUILD_GMOCK ON CACHE BOOL "" FORCE)
  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)

  # Adds the targers: gtest, gtest_main, gmock, gmock_main
  add_subdirectory(
    ${googletest_SOURCE_DIR}
    ${googletest_BINARY_DIR}
    EXCLUDE_FROM_ALL
  )
else()
  find_package(GTest)
  if(NOT GTest_FOUND)
    message("[WARN] Skipping [tests] due to missing [GTest] package.")
  endif()
endif()

# Test utilities
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)
add_subdirectory(src)

# Unit tests
add_subdirectory(unit)

# Integration tests
add_subdirectory(integration)

# Add custom target to build all the tests as a single target
dart_get_test_list(test_targets)
add_custom_target(tests DEPENDS ${test_targets})

# Add custom target to build all the tests and run the tests
add_custom_target(tests_and_run COMMAND ${CMAKE_CTEST_COMMAND}
  DEPENDS
    ${test_targets}
    ${integration_tests}
    ${regression_tests}
    ${unit_tests}
)
