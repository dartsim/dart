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
# The target keeps the public name; the module it builds is the private
# dartpy._core extension inside the staged package.
set_target_properties(
  dartpy
  PROPERTIES
    OUTPUT_NAME _core
    DEBUG_POSTFIX ""
    EXCLUDE_FROM_ALL TRUE
    LIBRARY_OUTPUT_DIRECTORY "${DART_DARTPY_BUILD_DIR}/dartpy"
)
# Stage the pure-Python package files beside the extension at build time, so
# every configuration a multi-config generator builds gets a complete package
# and an edited file re-stages without a reconfigure. They live under package/
# so that this C++ source directory is never importable as the dartpy package:
# tests and examples put python/ on sys.path ahead of the build tree.
set(dartpy_staged_files)
foreach(python_file IN ITEMS __init__.py py.typed experimental/__init__.py)
  set(dartpy_staged "${DART_DARTPY_BUILD_DIR}/dartpy/${python_file}")
  add_custom_command(
    OUTPUT "${dartpy_staged}"
    COMMAND
      ${CMAKE_COMMAND} -E copy_if_different
      "${CMAKE_CURRENT_SOURCE_DIR}/package/${python_file}" "${dartpy_staged}"
    DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/package/${python_file}"
    COMMENT "Staging dartpy/${python_file}"
    VERBATIM
  )
  list(APPEND dartpy_staged_files "${dartpy_staged}")
endforeach()
add_custom_target(dartpy_package DEPENDS ${dartpy_staged_files})
add_dependencies(dartpy dartpy_package)
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
if("${DARTPY_VERSION_INFO}" STREQUAL "" AND SKBUILD_PROJECT_VERSION_FULL)
  # Wheels report the version scikit-build-core read from package.xml.
  set(DARTPY_VERSION_INFO "${SKBUILD_PROJECT_VERSION_FULL}")
elseif("${DARTPY_VERSION_INFO}" STREQUAL "")
  set(DARTPY_VERSION_INFO "${DART_VERSION}")
endif()
target_compile_definitions(
  dartpy
  PRIVATE DARTPY_VERSION_INFO="${DARTPY_VERSION_INFO}"
)
# A separate component lets C++ package checks skip the absolute site-packages.
# scikit-build-core installs the `python` component relative to the wheel root.
if(SKBUILD)
  set(dartpy_install_dir dartpy)
else()
  set(dartpy_install_dir "${PYTHON_SITE_PACKAGES}/dartpy")
endif()
install(
  TARGETS dartpy
  LIBRARY DESTINATION "${dartpy_install_dir}" COMPONENT python
)
install(
  FILES package/__init__.py package/py.typed
  DESTINATION "${dartpy_install_dir}"
  COMPONENT python
)
install(
  FILES package/experimental/__init__.py
  DESTINATION "${dartpy_install_dir}/experimental"
  COMPONENT python
)
if(SKBUILD AND EXISTS "${PROJECT_BINARY_DIR}/dartpy-filament-license.txt")
  # scikit-build-core resolves license files before CMake fetches the Filament
  # SDK, so stage its license inside the package; scripts/pep517/dartpy_backend.py
  # moves it into the wheel's dist-info licenses.
  file(
    READ "${PROJECT_BINARY_DIR}/dartpy-filament-license.txt"
    dartpy_filament_license
  )
  string(STRIP "${dartpy_filament_license}" dartpy_filament_license)
  if(dartpy_filament_license)
    get_filename_component(
      dartpy_filament_license_dir
      "${dartpy_filament_license}"
      DIRECTORY
    )
    install(
      FILES "${dartpy_filament_license}"
      DESTINATION "${dartpy_install_dir}/_filament_licenses"
      COMPONENT python
    )
    install(
      DIRECTORY "${dartpy_filament_license_dir}/notices"
      DESTINATION "${dartpy_install_dir}/_filament_licenses"
      COMPONENT python
      OPTIONAL
    )
  endif()
endif()
list(
  REMOVE_ITEM dartpy_headers
  "${CMAKE_CURRENT_SOURCE_DIR}/detail/dart_eigen_dense.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/detail/dart_numpy_capi.h"
)
dart_format_add(${dartpy_headers} ${dartpy_sources})
