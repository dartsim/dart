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

set(dartpy_abi)
if(DART_DARTPY_STABLE_ABI)
  set(dartpy_abi STABLE_ABI)
endif()
# dartpy.viewer binds the private demo app host, which a vanilla system ImGui
# compiles out (see dart/gui/CMakeLists.txt).
set(dartpy_build_sources ${dartpy_sources})
if(NOT TARGET dart-gui-app)
  list(FILTER dartpy_build_sources EXCLUDE REGEX "/viewer/[^/]+\\.cpp$")
  message(STATUS "dartpy.viewer is compiled out with the demo app host")
endif()
nanobind_add_module(
  dartpy
  NB_STATIC
  PROTECT_STACK
  ${dartpy_abi}
  ${dartpy_headers}
  ${dartpy_build_sources}
)
set_target_properties(
  dartpy
  PROPERTIES OUTPUT_NAME dartpy DEBUG_POSTFIX "" EXCLUDE_FROM_ALL TRUE
)
# nanobind builds its library with -mtls-dialect=gnu2. Wheels link DART
# statically, which makes dartpy's TLS block too large for static TLS, and
# glibc 2.39 and older corrupt registers on the dynamic TLS-descriptor path
# (glibc bug 31372); Python overrides then silently stop dispatching. Use the
# traditional dialect, which every glibc handles.
if(NB_HAS_MTLS_GNU2)
  get_target_property(dartpy_link_libraries dartpy LINK_LIBRARIES)
  foreach(library IN LISTS dartpy_link_libraries)
    if(library MATCHES "^nanobind" AND TARGET ${library})
      target_compile_options(${library} PRIVATE -mtls-dialect=gnu)
    endif()
  endforeach()
endif()

# Wheel builds set CMAKE_LIBRARY_OUTPUT_DIRECTORY to setuptools' directory.
if(NOT CMAKE_LIBRARY_OUTPUT_DIRECTORY)
  set_target_properties(
    dartpy
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${DART_PYTHON_BUILD_DIR}/dartpy"
  )
endif()
target_compile_features(dartpy PRIVATE cxx_std_17)
target_include_directories(dartpy PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_include_directories(dartpy SYSTEM PRIVATE "${NB_DIR}/src")
set(dartpy_libraries dart dart-io dart-gui)
foreach(dartpy_library ${dartpy_libraries})
  if(NOT TARGET ${dartpy_library})
    message(
      FATAL_ERROR
      "Cannot find the library ${dartpy_library}. Please check the configuration of DART."
    )
  endif()
endforeach()
target_link_libraries(dartpy PRIVATE ${dartpy_libraries} tsl::robin_map)
if(TARGET dart-gui-app)
  target_link_libraries(dartpy PRIVATE dart-gui-app)
  target_compile_definitions(dartpy PRIVATE DARTPY_HAS_VIEWER)
  if(NOT DART_BUILD_DEMOS_MEMORY_DIAGNOSTICS)
    # The default build must not ship the opt-in memory diagnostics.
    add_custom_command(
      TARGET dartpy
      POST_BUILD
      COMMAND
        ${CMAKE_COMMAND} "-DDART_HOST_BINARY=$<TARGET_FILE:dartpy>" -P
        "${PROJECT_SOURCE_DIR}/dart/gui/app/tests/check_memory_diagnostics_compiled_out.cmake"
      COMMENT "Verifying memory diagnostics are absent from dartpy"
      VERBATIM
    )
  endif()
endif()
if(TARGET dart-utils-assets)
  target_compile_definitions(dartpy PRIVATE DARTPY_HAS_UTILS_ASSETS)
endif()
if("${DARTPY_VERSION_INFO}" STREQUAL "")
  set(DARTPY_VERSION_INFO "${DART_VERSION}")
endif()
target_compile_definitions(
  dartpy
  PRIVATE DARTPY_VERSION_INFO="${DARTPY_VERSION_INFO}"
)
# A separate component lets C++ package checks skip the absolute site-packages.
install(
  TARGETS dartpy
  LIBRARY DESTINATION "${PYTHON_SITE_PACKAGES}" COMPONENT python
)
list(
  REMOVE_ITEM dartpy_headers
  "${CMAKE_CURRENT_SOURCE_DIR}/detail/dart_eigen_dense.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/detail/dart_numpy_capi.h"
)
dart_format_add(${dartpy_headers} ${dartpy_sources})
