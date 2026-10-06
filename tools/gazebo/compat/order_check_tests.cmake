# Included at the top-level project() of gz-physics and gz-sim by lane.sh
# (CMAKE_PROJECT_TOP_LEVEL_INCLUDES). gz-cmake adds a check_<test> test that
# writes a failing result for <test> when its GoogleTest XML is missing or
# empty. Under `ctest --parallel` the two can run at once, and check_<test>
# then writes over the XML <test> is writing. Run every check_<test> after its
# test.
function(gz_compat_order_check_tests dir)
  get_property(tests DIRECTORY "${dir}" PROPERTY TESTS)
  foreach(test IN LISTS tests)
    if(test MATCHES "^check_(.+)$")
      set(checked "${CMAKE_MATCH_1}")
      if(checked IN_LIST tests)
        set_property(
          TEST "${test}"
          DIRECTORY "${dir}"
          APPEND
          PROPERTY DEPENDS "${checked}"
        )
      endif()
    endif()
  endforeach()
  get_property(subdirs DIRECTORY "${dir}" PROPERTY SUBDIRECTORIES)
  foreach(subdir IN LISTS subdirs)
    gz_compat_order_check_tests("${subdir}")
  endforeach()
endfunction()

cmake_language(DEFER CALL gz_compat_order_check_tests "${CMAKE_SOURCE_DIR}")
