# Copyright (c) 2011, The DART development contributors
# All rights reserved.

execute_process(
  COMMAND
    "${Python_EXECUTABLE}" -c
    "import sysconfig; print(sysconfig.get_path('platlib'))"
  OUTPUT_VARIABLE PYTHON_SITE_PACKAGES
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
if(NOT IS_ABSOLUTE "${PYTHON_SITE_PACKAGES}")
  set(PYTHON_SITE_PACKAGES "${CMAKE_INSTALL_PREFIX}/${PYTHON_SITE_PACKAGES}")
endif()
set(PYTHON_SITE_PACKAGES "${PYTHON_SITE_PACKAGES}" PARENT_SCOPE)

file(GLOB_RECURSE dartpy_headers CONFIGURE_DEPENDS "*.h" "*.hpp")
file(GLOB_RECURSE dartpy_sources CONFIGURE_DEPENDS "*.cpp")
execute_process(
  COMMAND
    "${Python_EXECUTABLE}"
    "${PROJECT_SOURCE_DIR}/scripts/nanobind/check_guards.py"
    "${CMAKE_CURRENT_SOURCE_DIR}"
  RESULT_VARIABLE dartpy_guard_result
  OUTPUT_VARIABLE dartpy_guard_output
  ERROR_VARIABLE dartpy_guard_error
)
if(NOT dartpy_guard_result EQUAL 0)
  message(
    FATAL_ERROR
    "Invalid dartpy caster selection:\n${dartpy_guard_output}${dartpy_guard_error}"
  )
endif()

nanobind_add_module(dartpy NB_STATIC PROTECT_STACK ${dartpy_headers} ${dartpy_sources})
set_target_properties(
  dartpy
  PROPERTIES
    OUTPUT_NAME dartpy
    DEBUG_POSTFIX ""
    EXCLUDE_FROM_ALL TRUE
    LIBRARY_OUTPUT_DIRECTORY "${DART_PYTHON_BUILD_DIR}/dartpy"
)
target_compile_features(dartpy PRIVATE cxx_std_17)
target_include_directories(
  dartpy
  PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}" "${PROJECT_SOURCE_DIR}/python/include"
)
target_include_directories(dartpy SYSTEM PRIVATE "${NB_DIR}/src")
target_link_libraries(
  dartpy
  PRIVATE dart dart-utils dart-utils-urdf tsl::robin_map
)
foreach(optional dart-collision-bullet dart-collision-ode)
  if(TARGET ${optional})
    target_link_libraries(dartpy PRIVATE ${optional})
  endif()
endforeach()
target_compile_definitions(
  dartpy
  PRIVATE DARTPY_VERSION_INFO="${DARTPY_VERSION_INFO}"
)
install(TARGETS dartpy LIBRARY DESTINATION "${PYTHON_SITE_PACKAGES}")
# Headers for extension modules that exchange DART objects with dartpy.
set(
  dartpy_public_headers
  "${PROJECT_SOURCE_DIR}/python/include/dartpy/c_api.hpp"
  "${PROJECT_SOURCE_DIR}/python/include/dartpy/pybind11_interop.hpp"
)
install(
  FILES ${dartpy_public_headers}
  DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/dartpy"
  COMPONENT headers
)
list(
  REMOVE_ITEM dartpy_headers
  "${CMAKE_CURRENT_SOURCE_DIR}/detail/dart_eigen_dense.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/detail/dart_numpy_capi.h"
)
dart_format_add(${dartpy_headers} ${dartpy_sources} ${dartpy_public_headers})
