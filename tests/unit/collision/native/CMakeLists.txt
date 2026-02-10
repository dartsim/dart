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

message(STATUS "collision-native: Configuring unit tests...")

if(DART_BUILD_TESTS AND TARGET GTest::gtest)
  set(COLLISION_EXPERIMENTAL_TESTS "" CACHE INTERNAL "" FORCE)

  set(DART_EXPERIMENTAL_VOXBLOX_ROOT "" CACHE PATH "Path to Voxblox for experimental collision tests")
  if(DART_EXPERIMENTAL_VOXBLOX_ROOT)
    find_path(VOXBLOX_INCLUDE_DIR
      NAMES voxblox/core/esdf_map.h
      PATHS
        ${DART_EXPERIMENTAL_VOXBLOX_ROOT}
        ${DART_EXPERIMENTAL_VOXBLOX_ROOT}/voxblox/include
        ${DART_EXPERIMENTAL_VOXBLOX_ROOT}/include
    )
    find_library(VOXBLOX_LIBRARY
      NAMES voxblox
      PATHS
        ${DART_EXPERIMENTAL_VOXBLOX_ROOT}/lib
        ${DART_EXPERIMENTAL_VOXBLOX_ROOT}/build
        ${DART_EXPERIMENTAL_VOXBLOX_ROOT}/devel/lib
    )
  endif()
  set(VOXBLOX_FOUND FALSE)
  if(VOXBLOX_INCLUDE_DIR AND VOXBLOX_LIBRARY)
    set(VOXBLOX_FOUND TRUE)
  endif()

  file(GLOB test_files "${CMAKE_CURRENT_SOURCE_DIR}/test_*.cpp")
  if(NOT DART_TESTS_LIBCCD_FOUND)
    list(FILTER test_files EXCLUDE REGEX "test_libccd_.*\\.cpp$")
  endif()
  list(APPEND test_files "${CMAKE_CURRENT_SOURCE_DIR}/test_parallel_determinism.cpp")
  list(REMOVE_DUPLICATES test_files)

  foreach(test_file ${test_files})
    get_filename_component(test_name ${test_file} NAME_WE)

    add_executable(${test_name} ${test_file})
    target_link_libraries(${test_name}
      PRIVATE
        dart-collision-native
        GTest::gtest
        GTest::gtest_main
    )
    if(DART_TESTS_LIBCCD_FOUND AND test_name MATCHES "^test_libccd_")
      target_link_libraries(${test_name} PRIVATE dart_tests_libccd)
    endif()
    if(test_name STREQUAL "test_reference_backends")
      target_link_libraries(${test_name} PRIVATE dart)
      if(TARGET dart-collision-fcl)
        target_link_libraries(${test_name} PRIVATE
          -Wl,--no-as-needed dart-collision-fcl -Wl,--as-needed)
      endif()
      if(TARGET dart-collision-bullet)
        target_link_libraries(${test_name} PRIVATE
          -Wl,--no-as-needed dart-collision-bullet -Wl,--as-needed)
      endif()
      if(TARGET dart-collision-ode)
        target_link_libraries(${test_name} PRIVATE
          -Wl,--no-as-needed dart-collision-ode -Wl,--as-needed)
      endif()
    endif()
    if(test_name STREQUAL "test_sdf_compare" AND VOXBLOX_FOUND)
      target_include_directories(${test_name} PRIVATE ${VOXBLOX_INCLUDE_DIR})
      target_link_libraries(${test_name} PRIVATE ${VOXBLOX_LIBRARY})
      target_compile_definitions(${test_name} PRIVATE DART_EXPERIMENTAL_HAVE_VOXBLOX=1)
    endif()

    add_test(NAME ${test_name} COMMAND ${test_name})
    set_tests_properties(${test_name} PROPERTIES LABELS "collision-native")

    list(APPEND COLLISION_EXPERIMENTAL_TESTS ${test_name})
  endforeach()

  if(COLLISION_EXPERIMENTAL_TESTS)
    add_custom_target(dart_collision_native_tests
      DEPENDS ${COLLISION_EXPERIMENTAL_TESTS}
      COMMENT "Building all collision-native unit tests"
    )
  endif()

  list(LENGTH test_files num_tests)
  message(STATUS "collision-native: Total ${num_tests} unit test(s)")
else()
  message(STATUS "collision-native: Unit tests disabled (GTest not found or tests disabled)")
endif()
