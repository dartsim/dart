# Fix gz-physics COMMON_TEST_* builds when system gtest headers are newer than
# the vendored gtest static library shipped with gz-physics.
#
# gz-physics' COMMON_TEST_* executables explicitly link against the vendored
# `gtest`/`gtest_main` targets, but may pick up system gtest headers via other
# dependencies, leading to link errors due to ABI/signature differences.
#
# This script is injected via CMAKE_PROJECT_TOP_LEVEL_INCLUDES during DART's
# `pixi run -e gazebo ...` integration test and does not modify gz-physics
# sources.

function(_dart_collect_buildsystem_targets dir out_var)
  get_property(dir_targets DIRECTORY "${dir}" PROPERTY BUILDSYSTEM_TARGETS)
  if(dir_targets STREQUAL "dir_targets-NOTFOUND")
    set(dir_targets "")
  endif()

  set(all_targets "${dir_targets}")

  get_property(subdirs DIRECTORY "${dir}" PROPERTY SUBDIRECTORIES)
  if(subdirs STREQUAL "subdirs-NOTFOUND")
    set(subdirs "")
  endif()

  foreach(subdir IN LISTS subdirs)
    _dart_collect_buildsystem_targets("${subdir}" sub_targets)
    list(APPEND all_targets ${sub_targets})
  endforeach()

  set(${out_var} "${all_targets}" PARENT_SCOPE)
endfunction()

function(_dart_fix_common_test_gtest_includes)
  set(vendor_include_dir "${CMAKE_SOURCE_DIR}/test/gtest_vendor/include")
  if(NOT EXISTS "${vendor_include_dir}")
    message(WARNING "Expected gz-physics gtest_vendor headers at '${vendor_include_dir}', but directory was not found.")
    return()
  endif()

  _dart_collect_buildsystem_targets("${CMAKE_SOURCE_DIR}" all_targets)
  foreach(target_name IN LISTS all_targets)
    if(NOT target_name MATCHES "^COMMON_TEST_")
      continue()
    endif()
    if(NOT TARGET "${target_name}")
      continue()
    endif()
    target_include_directories("${target_name}" BEFORE PRIVATE "${vendor_include_dir}")
  endforeach()
endfunction()

cmake_language(DEFER CALL _dart_fix_common_test_gtest_includes)
