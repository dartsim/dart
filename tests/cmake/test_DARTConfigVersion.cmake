cmake_minimum_required(VERSION 3.22.1)

function(render_version version pointer_size)
  set(DART_VERSION "${version}")
  set(CMAKE_SIZEOF_VOID_P "${pointer_size}")
  set(package_dir "${TEST_BINARY_DIR}/${version}-${pointer_size}")
  file(MAKE_DIRECTORY "${package_dir}")
  configure_file(
    "${SOURCE_DIR}/cmake/DARTConfigVersion.cmake.in"
    "${package_dir}/DARTConfigVersion.cmake"
    @ONLY
  )
  file(WRITE "${package_dir}/DARTConfig.cmake" "set(DART_FOUND TRUE)\n")
  set(version_file "${package_dir}/DARTConfigVersion.cmake" PARENT_SCOPE)
  set(package_dir "${package_dir}" PARENT_SCOPE)
endfunction()

function(
  check_request
  request
  require_exact
  expected
  expected_exact
  unsuitable
)
  unset(PACKAGE_VERSION_COMPATIBLE)
  unset(PACKAGE_VERSION_EXACT)
  unset(PACKAGE_VERSION_UNSUITABLE)
  unset(PACKAGE_FIND_VERSION_RANGE)
  if(request MATCHES "^(.+)\\.\\.\\.(<)?(.+)$")
    set(PACKAGE_FIND_VERSION_RANGE "${request}")
    set(PACKAGE_FIND_VERSION_MIN "${CMAKE_MATCH_1}")
    set(PACKAGE_FIND_VERSION_MAX "${CMAKE_MATCH_3}")
    set(PACKAGE_FIND_VERSION_RANGE_MIN INCLUDE)
    if(CMAKE_MATCH_2 STREQUAL "<")
      set(PACKAGE_FIND_VERSION_RANGE_MAX EXCLUDE)
    else()
      set(PACKAGE_FIND_VERSION_RANGE_MAX INCLUDE)
    endif()
    set(PACKAGE_FIND_VERSION "${PACKAGE_FIND_VERSION_MIN}")
  else()
    set(PACKAGE_FIND_VERSION "${request}")
  endif()
  string(REPLACE "." ";" parts "${PACKAGE_FIND_VERSION}")
  list(LENGTH parts PACKAGE_FIND_VERSION_COUNT)
  set(index 0)
  foreach(component MAJOR MINOR PATCH TWEAK)
    set(PACKAGE_FIND_VERSION_${component} 0)
    if(index LESS PACKAGE_FIND_VERSION_COUNT)
      list(GET parts ${index} PACKAGE_FIND_VERSION_${component})
    endif()
    math(EXPR index "${index} + 1")
  endforeach()

  include("${version_file}")
  set(accepted FALSE)
  if(PACKAGE_VERSION_COMPATIBLE AND NOT PACKAGE_VERSION_UNSUITABLE)
    # CMake enforces EXACT by selecting only a file with PACKAGE_VERSION_EXACT.
    if(NOT require_exact OR PACKAGE_VERSION_EXACT)
      set(accepted TRUE)
    endif()
  endif()
  if(NOT accepted STREQUAL expected)
    message(
      FATAL_ERROR
      "${version_file}: ${request}, EXACT=${require_exact}: expected acceptance ${expected}, got ${accepted}"
    )
  endif()
  if(
    (expected_exact AND NOT PACKAGE_VERSION_EXACT)
    OR (NOT expected_exact AND PACKAGE_VERSION_EXACT)
  )
    message(
      FATAL_ERROR
      "${version_file}: ${request}: unexpected PACKAGE_VERSION_EXACT=${PACKAGE_VERSION_EXACT}"
    )
  endif()
  if(
    (unsuitable AND NOT PACKAGE_VERSION_UNSUITABLE)
    OR (NOT unsuitable AND PACKAGE_VERSION_UNSUITABLE)
  )
    message(
      FATAL_ERROR
      "${version_file}: ${request}: unexpected PACKAGE_VERSION_UNSUITABLE=${PACKAGE_VERSION_UNSUITABLE}"
    )
  endif()
  if(unsuitable)
    math(EXPR installed_bits "${producer_size} * 8")
    if(NOT PACKAGE_VERSION STREQUAL "${version} (${installed_bits}bit)")
      message(FATAL_ERROR "Missing architecture suffix: ${PACKAGE_VERSION}")
    endif()
  elseif(NOT PACKAGE_VERSION STREQUAL version)
    message(FATAL_ERROR "Unexpected PACKAGE_VERSION=${PACKAGE_VERSION}")
  endif()
endfunction()

file(MAKE_DIRECTORY "${TEST_BINARY_DIR}/consumer")
file(
  WRITE "${TEST_BINARY_DIR}/consumer/CMakeLists.txt"
  [=[
cmake_minimum_required(VERSION 3.22.1)
project(DARTVersionConsumer NONE)
find_package(DART ${REQUEST} CONFIG QUIET PATHS "${DART_DIR}" NO_DEFAULT_PATH)
if((EXPECT_FOUND AND NOT DART_FOUND) OR (NOT EXPECT_FOUND AND DART_FOUND))
  message(FATAL_ERROR "Unexpected find_package result for ${REQUEST}: ${DART_FOUND}")
endif()
]=]
)

function(check_find_package request expected)
  string(MAKE_C_IDENTIFIER "${request}" case_name)
  execute_process(
    COMMAND
      "${CMAKE_COMMAND}" -S "${TEST_BINARY_DIR}/consumer" -B
      "${package_dir}/consumer-${case_name}" "-DDART_DIR=${package_dir}"
      "-DREQUEST=${request}" "-DEXPECT_FOUND=${expected}"
    RESULT_VARIABLE result
    OUTPUT_VARIABLE output
    ERROR_VARIABLE error
  )
  if(NOT result EQUAL 0)
    message(
      FATAL_ERROR
      "find_package(${request}) for ${version} failed:\n${output}\n${error}"
    )
  endif()
endfunction()

set(producer_size "${CMAKE_SIZEOF_VOID_P}")
set(
  versions
  6.20.0
  7.0.0
  7.4.2
  8.0.0
  "${DART_VERSION}"
)
list(REMOVE_DUPLICATES versions)
foreach(version IN LISTS versions)
  render_version("${version}" "${producer_size}")
  foreach(request 6.0 6.10 6.16)
    check_request("${request}" FALSE TRUE FALSE FALSE)
  endforeach()
  foreach(request 5.0 5.99 99.0.0)
    check_request("${request}" FALSE FALSE FALSE FALSE)
  endforeach()
  check_request(6.16 TRUE FALSE FALSE FALSE)
  check_request("${version}" TRUE TRUE TRUE FALSE)
  check_request("6.0...${version}" FALSE TRUE FALSE FALSE)
  check_request("6.0...<${version}" FALSE FALSE FALSE FALSE)
  check_request("5.99...99" FALSE FALSE FALSE FALSE)
  if(version VERSION_LESS 7)
    set(range_accepted TRUE)
  else()
    set(range_accepted FALSE)
  endif()
  check_request("6.10...<7" FALSE "${range_accepted}" FALSE FALSE)

  check_find_package(6.10 TRUE)
  check_find_package(5.99 FALSE)
  check_find_package("6.16;EXACT" FALSE)
  check_find_package("${version};EXACT" TRUE)
  check_find_package("6.10...<7" "${range_accepted}")

  math(EXPR CMAKE_SIZEOF_VOID_P "${producer_size} + 1")
  check_request("${version}" TRUE FALSE TRUE TRUE)
  unset(CMAKE_SIZEOF_VOID_P)
  check_request("${version}" TRUE TRUE TRUE FALSE)
  set(CMAKE_SIZEOF_VOID_P "${producer_size}")
  render_version("${version}" "")
  check_request("${version}" TRUE TRUE TRUE FALSE)
endforeach()
message(STATUS "DART package version tests passed for ${versions}")
