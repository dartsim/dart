cmake_minimum_required(VERSION 3.22.1)

file(REMOVE_RECURSE "${TEST_BINARY_DIR}")
set(prefix "${TEST_BINARY_DIR}/install")
# Stage the C++ package without dartpy's interpreter-specific installation.
execute_process(
  COMMAND
    "${CMAKE_COMMAND}" --install "${DART_BINARY_DIR}" --prefix "${prefix}"
    --component Unspecified --config "${TEST_CONFIG}"
  RESULT_VARIABLE result
  OUTPUT_VARIABLE output
  ERROR_VARIABLE error
)
if(NOT result EQUAL 0)
  message(FATAL_ERROR "DART installation failed:\n${output}\n${error}")
endif()

# A header-only ODE fixture forces the probe, regardless of system ODE's
# compile definitions, and exercises both values of the feature macro.
set(ode_dir "${TEST_BINARY_DIR}/ode")
file(MAKE_DIRECTORY "${ode_dir}/include/ode" "${TEST_BINARY_DIR}/consumer")
file(
  WRITE "${ode_dir}/ODEConfig.cmake"
  [=[
set(ODE_VERSION 0.16.2)
if(NOT TARGET ODE::ODE)
  add_library(ODE::ODE INTERFACE IMPORTED)
  set_target_properties(ODE::ODE PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_LIST_DIR}/include")
endif()
]=]
)
file(
  WRITE "${TEST_BINARY_DIR}/consumer/CMakeLists.txt"
  [=[
cmake_minimum_required(VERSION 3.22.1)
project(consumer LANGUAGES ${TEST_LANGUAGES})
find_package(DART CONFIG REQUIRED COMPONENTS collision-ode)
if(NOT DART_ODE_HAS_LIBCCD_BOX_CYL STREQUAL EXPECT_LIBCCD)
  message(FATAL_ERROR
    "Expected libccd box-cylinder support ${EXPECT_LIBCCD}, got ${DART_ODE_HAS_LIBCCD_BOX_CYL}")
endif()
]=]
)

foreach(has_macro 0 1)
  if(has_macro)
    file(WRITE "${ode_dir}/include/ode/odeconfig.h" "#define dLIBCCD_BOX_CYL\n")
  else()
    file(WRITE "${ode_dir}/include/ode/odeconfig.h" "/* libccd disabled */\n")
  endif()
  foreach(languages C CXX "C;CXX")
    string(REPLACE ";" "-" case_name "${languages}-${has_macro}")
    execute_process(
      COMMAND
        "${CMAKE_COMMAND}" -S "${TEST_BINARY_DIR}/consumer" -B
        "${TEST_BINARY_DIR}/${case_name}"
        "-DDART_DIR=${prefix}/share/dart/cmake" "-DODE_DIR=${ode_dir}"
        "-DCMAKE_PREFIX_PATH=${TEST_PREFIX_PATH}"
        "-DTEST_LANGUAGES=${languages}" "-DEXPECT_LIBCCD=${has_macro}"
      RESULT_VARIABLE result
      OUTPUT_VARIABLE output
      ERROR_VARIABLE error
    )
    if(NOT result EQUAL 0)
      message(
        FATAL_ERROR
        "ODE consumer ${case_name} failed:\n${output}\n${error}"
      )
    endif()
    message(STATUS "Installed DART ODE consumer ${case_name} passed")
  endforeach()
endforeach()
