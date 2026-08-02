# Run the broad CTest gate without inheriting GoogleTest selector or control
# variables from the caller. Per-test selections remain owned by CTest
# properties and are validated separately by the AI infrastructure checker.

if(NOT DEFINED DART_CTEST_COMMAND OR DART_CTEST_COMMAND STREQUAL "")
  message(
    FATAL_ERROR
    "DART_CTEST_COMMAND must name the configured CTest executable"
  )
endif()

execute_process(
  COMMAND "${CMAKE_COMMAND}" -E environment
  OUTPUT_VARIABLE _dart_test_environment
  RESULT_VARIABLE _dart_environment_result
)
if(NOT _dart_environment_result EQUAL 0)
  message(FATAL_ERROR "Failed to inspect the test environment")
endif()

string(REPLACE "\r\n" "\n" _dart_test_environment "${_dart_test_environment}")
string(REPLACE "\r" "\n" _dart_test_environment "${_dart_test_environment}")
string(REPLACE ";" "\\;" _dart_test_environment "${_dart_test_environment}")
string(REPLACE "\n" ";" _dart_test_environment "${_dart_test_environment}")

set(_dart_gtest_unsets)
foreach(_dart_environment_entry IN LISTS _dart_test_environment)
  string(FIND "${_dart_environment_entry}" "=" _dart_separator)
  if(_dart_separator GREATER 0)
    string(
      SUBSTRING "${_dart_environment_entry}"
      0
      ${_dart_separator}
      _dart_name
    )
    string(TOUPPER "${_dart_name}" _dart_name_upper)
    if(_dart_name_upper MATCHES "^GTEST_")
      list(APPEND _dart_gtest_unsets "--unset=${_dart_name}")
    endif()
  endif()
endforeach()

set(_dart_ctest_arguments --output-on-failure)
if(
  DEFINED DART_CTEST_CONFIGURATION
  AND NOT DART_CTEST_CONFIGURATION STREQUAL ""
)
  list(APPEND _dart_ctest_arguments -C "${DART_CTEST_CONFIGURATION}")
endif()

# Long-measurement entries are optimized-build parameter sweeps (hundreds of
# settle-push-recover trials per gate); an unoptimized aggregate run
# re-executes the same code paths tens of times slower, and the assertion
# (CMAKE_BUILD_TYPE=None) lane blew CI's six-hour job ceiling doing exactly
# that. Optimized configurations keep the sweeps; everything else runs the
# fast gates only, matching the FreeBSD and coverage label exclusions.
set(_dart_effective_configuration "${DART_CTEST_CONFIGURATION}")
if(_dart_effective_configuration STREQUAL "")
  set(_dart_effective_configuration "${DART_CTEST_BUILD_TYPE}")
endif()
if(
  NOT
    _dart_effective_configuration
      MATCHES
      "^(Release|RelWithDebInfo|MinSizeRel)$"
)
  list(APPEND _dart_ctest_arguments --label-exclude long-measurement)
endif()

execute_process(
  COMMAND
    "${CMAKE_COMMAND}" -E env ${_dart_gtest_unsets} "${DART_CTEST_COMMAND}"
    ${_dart_ctest_arguments}
  COMMAND_ERROR_IS_FATAL ANY
)
