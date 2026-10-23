cmake_minimum_required(VERSION 3.22.1)

file(REMOVE_RECURSE "${TEST_BINARY_DIR}")
set(prefix "${TEST_BINARY_DIR}/install")
# A headers-only installation must be self-contained: every DART header it
# installs finds the DART headers it includes, generated ones included.
execute_process(
  COMMAND
    "${CMAKE_COMMAND}" --install "${DART_BINARY_DIR}" --prefix "${prefix}"
    --component headers --config "${TEST_CONFIG}"
  RESULT_VARIABLE result
  OUTPUT_VARIABLE output
  ERROR_VARIABLE error
)
if(NOT result EQUAL 0)
  message(FATAL_ERROR "Headers installation failed:\n${output}\n${error}")
endif()
if(NOT EXISTS "${prefix}/include/dart/config.hpp")
  message(FATAL_ERROR "The headers component does not install dart/config.hpp")
endif()

file(
  GLOB_RECURSE headers
  "${prefix}/include/dart/*.hpp"
  "${prefix}/include/dart/*.h"
)
set(include_regex "^[ \t]*#[ \t]*include[ \t]*[<\"](dart/[^>\"]+)[>\"]")
set(missing)
foreach(header IN LISTS headers)
  if(header MATCHES "/(IncludeUrdf|urdf_world_parser)\\.hpp$")
    message(FATAL_ERROR "Private URDF header installed: ${header}")
  endif()
  file(
    STRINGS "${header}"
    urdf_includes
    REGEX "^[ \t]*#[ \t]*include[ \t]*[<\"]urdf[^>\"]*[>\"]"
  )
  if(urdf_includes)
    message(FATAL_ERROR "Installed header includes urdfdom: ${header}")
  endif()
  file(STRINGS "${header}" includes REGEX "${include_regex}")
  foreach(line IN LISTS includes)
    string(REGEX REPLACE "${include_regex}.*" "\\1" path "${line}")
    if(NOT EXISTS "${prefix}/include/${path}")
      list(APPEND missing "${path} (included by ${header})")
    endif()
  endforeach()
endforeach()
if(missing)
  list(REMOVE_DUPLICATES missing)
  list(JOIN missing "\n  " missing)
  message(FATAL_ERROR "The headers component misses:\n  ${missing}")
endif()
