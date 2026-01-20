# Copyright (c) 2011, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

cmake_minimum_required(VERSION 3.22.1)

include(dart_defs)
include(dart_test_libccd)

enable_testing()

message(STATUS "collision-experimental: Configuring unit tests...")

if(DART_BUILD_TESTS AND TARGET GTest::gtest)
  set(COLLISION_EXPERIMENTAL_TESTS "" CACHE INTERNAL "" FORCE)

  file(GLOB test_files "${CMAKE_CURRENT_SOURCE_DIR}/test_*.cpp")
  if(NOT DART_TESTS_LIBCCD_FOUND)
    list(FILTER test_files EXCLUDE REGEX "test_libccd_.*\\.cpp$")
  endif()

  foreach(test_file ${test_files})
    get_filename_component(test_name ${test_file} NAME_WE)

    add_executable(${test_name} ${test_file})
    target_link_libraries(${test_name}
      PRIVATE
        dart-collision-experimental
        GTest::gtest
        GTest::gtest_main
    )
    if(DART_TESTS_LIBCCD_FOUND AND test_name MATCHES "^test_libccd_")
      target_link_libraries(${test_name} PRIVATE dart_tests_libccd)
    endif()
    if(test_name STREQUAL "test_reference_backends")
      target_link_libraries(${test_name} PRIVATE dart)
    endif()

    add_test(NAME ${test_name} COMMAND ${test_name})
    set_tests_properties(${test_name} PROPERTIES LABELS "collision-experimental")

    list(APPEND COLLISION_EXPERIMENTAL_TESTS ${test_name})
  endforeach()

  if(COLLISION_EXPERIMENTAL_TESTS)
    add_custom_target(dart_collision_experimental_tests
      DEPENDS ${COLLISION_EXPERIMENTAL_TESTS}
      COMMENT "Building all collision-experimental unit tests"
    )
  endif()

  list(LENGTH test_files num_tests)
  message(STATUS "collision-experimental: Total ${num_tests} unit test(s)")
else()
  message(STATUS "collision-experimental: Unit tests disabled (GTest not found or tests disabled)")
endif()
