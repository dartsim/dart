# Set local target/component names. Keep these variable names io-specific:
# parser subdirectories and package helpers may use generic names such as
# target_name internally.
set(dart_io_project_name dart)
set(dart_io_target_name dart-io)
set(dart_io_component_name io)

# Dependency checks
dart_find_package(tinyxml2)
dart_check_optional_package(TINYXML2 "dart-io" "tinyxml2" "1.0.1")

# sdformat is optional when DART_ENABLE_SDFORMAT=OFF (e.g., Alt Linux)
if(DART_ENABLE_SDFORMAT)
  dart_find_package(sdformat)
  dart_check_optional_package(SDFORMAT "dart-io" "sdformat" "16.0.0")
  if(NOT DART_HAVE_SDFORMAT)
    message(
      FATAL_ERROR
      "libsdformat (>= 16.0.0) is required for dart-io. Please install it or "
      "set CMAKE_PREFIX_PATH so CMake can find sdformat-config.cmake. "
      "Alternatively, set DART_ENABLE_SDFORMAT=OFF to build without sdformat support."
    )
  endif()
  if(TARGET sdformat::sdformat)
    set_property(TARGET sdformat::sdformat PROPERTY IMPORTED_GLOBAL TRUE)
    dart_prune_msvc_posix_link_libraries(sdformat::sdformat)
  else()
    message(
      FATAL_ERROR
      "libsdformat was found but the sdformat::sdformat imported target is "
      "missing. Please install the CMake config package that exports this "
      "target."
    )
  endif()
else()
  # Clear cached DART_HAVE_SDFORMAT to prevent stale values from previous
  # configurations where SDF was enabled.
  set(DART_HAVE_SDFORMAT FALSE CACHE BOOL "Whether sdformat is available" FORCE)
endif()

dart_find_package(urdfdom)
set(DART_IO_HAS_URDF FALSE)
if(urdfdom_FOUND)
  set(DART_IO_HAS_URDF TRUE)
  if(DART_VERBOSE)
    message(STATUS "Looking for urdfdom - ${urdfdom_headers_VERSION} found")
  endif()
else()
  message(
    WARNING
    "Looking for urdfdom - NOT found, URDF parser support in dart-io will be disabled"
  )
endif()
set(
  DART_IO_HAS_URDF
  ${DART_IO_HAS_URDF}
  CACHE INTERNAL
  "Whether dart-io builds URDF parser support"
  FORCE
)
if(MSVC AND DART_IO_HAS_URDF)
  set(
    urdfdom_LIBRARIES
    optimized
    urdfdom_sensor
    debug
    urdfdom_sensord
    optimized
    urdfdom_model_state
    debug
    urdfdom_model_stated
    optimized
    urdfdom_model
    debug
    urdfdom_modeld
    optimized
    urdfdom_world
    debug
    urdfdom_worldd
    optimized
    console_bridge
    debug
    console_bridged
  )
endif()

find_package(CURL)
if(TARGET CURL::libcurl)
  dart_prune_msvc_posix_link_libraries(CURL::libcurl)
endif()
dart_check_optional_package(CURL "dart-io HTTP retriever" "libcurl")

function(_dart_strip_msvc_posix_math_link target)
  if(NOT MSVC OR NOT TARGET "${target}")
    return()
  endif()

  get_target_property(_interface_libs "${target}" INTERFACE_LINK_LIBRARIES)
  if(NOT _interface_libs)
    return()
  endif()

  set(_filtered_libs "")
  foreach(_lib IN LISTS _interface_libs)
    if(_lib STREQUAL "m" OR _lib STREQUAL "m.lib")
      continue()
    endif()
    list(APPEND _filtered_libs "${_lib}")
    if(TARGET "${_lib}")
      _dart_strip_msvc_posix_math_link("${_lib}")
    endif()
  endforeach()
  set_property(
    TARGET "${target}"
    PROPERTY INTERFACE_LINK_LIBRARIES "${_filtered_libs}"
  )
endfunction()

if(TARGET CURL::libcurl)
  _dart_strip_msvc_posix_math_link(CURL::libcurl)
endif()

# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")
file(GLOB detail_hdrs "detail/*.hpp")
file(GLOB detail_srcs "detail/*.cpp")
if(NOT DART_ENABLE_SDFORMAT)
  list(REMOVE_ITEM hdrs "${CMAKE_CURRENT_SOURCE_DIR}/sdf_writer.hpp")
  list(REMOVE_ITEM srcs "${CMAKE_CURRENT_SOURCE_DIR}/sdf_writer.cpp")
endif()

function(dart_add_io_headers)
  dart_property_add(DART_IO_HEADERS ${ARGN})
endfunction()

function(dart_add_io_sources)
  dart_property_add(DART_IO_SOURCES ${ARGN})
endfunction()

# Add parser subdirectories before creating dart-io so their source lists are
# folded into the single component target.
add_subdirectory(mjcf)
if(DART_ENABLE_SDFORMAT)
  add_subdirectory(sdf)
endif()
if(DART_IO_HAS_URDF)
  add_subdirectory(urdf)
endif()

get_property(dart_io_headers GLOBAL PROPERTY DART_IO_HEADERS)
get_property(dart_io_sources GLOBAL PROPERTY DART_IO_SOURCES)

# Add target
dart_add_library(
  ${dart_io_target_name}
  ${hdrs}
  ${srcs}
  ${detail_hdrs}
  ${detail_srcs}
  ${dart_io_headers}
  ${dart_io_sources}
)
target_link_libraries(${dart_io_target_name} PUBLIC dart tinyxml2::tinyxml2)

if(DART_ENABLE_SDFORMAT)
  target_link_libraries(${dart_io_target_name} PUBLIC sdformat::sdformat)
  target_compile_definitions(${dart_io_target_name} PUBLIC DART_HAS_SDFORMAT=1)
else()
  target_compile_definitions(${dart_io_target_name} PUBLIC DART_HAS_SDFORMAT=0)
endif()

if(HAVE_CURL)
  target_include_directories(
    ${dart_io_target_name}
    PRIVATE ${CURL_INCLUDE_DIRS}
  )
  target_link_libraries(${dart_io_target_name} PRIVATE CURL::libcurl)
  target_compile_definitions(${dart_io_target_name} PUBLIC DART_HAS_CURL=1)
else()
  target_compile_definitions(${dart_io_target_name} PUBLIC DART_HAS_CURL=0)
endif()

if(DART_IO_HAS_URDF)
  target_link_libraries(${dart_io_target_name} PUBLIC urdfdom)
  target_compile_definitions(${dart_io_target_name} PUBLIC DART_IO_HAS_URDF=1)
else()
  target_compile_definitions(${dart_io_target_name} PUBLIC DART_IO_HAS_URDF=0)
endif()

# Optional OpenUSD (pxr) scene loader, compiled into dart-io only when
# DART_BUILD_IO_USD is ON. The subdirectory wires the pxr dependency and sets
# DART_IO_HAS_USD=1; otherwise the front door in read.cpp emits a clean
# "USD support is not available" diagnostic.
if(DART_BUILD_IO_USD)
  add_subdirectory(usd)
else()
  target_compile_definitions(${dart_io_target_name} PUBLIC DART_IO_HAS_USD=0)
endif()

# Component
add_component(${dart_io_project_name} ${dart_io_component_name})
add_component_targets(
  ${dart_io_project_name}
  ${dart_io_component_name}
  ${dart_io_target_name}
)
add_component_dependencies(${dart_io_project_name} ${dart_io_component_name} dart)
add_component_dependency_packages(
  ${dart_io_project_name}
  ${dart_io_component_name}
  tinyxml2
)
if(DART_ENABLE_SDFORMAT)
  add_component_dependency_packages(
    ${dart_io_project_name}
    ${dart_io_component_name}
    sdformat
  )
endif()
if(DART_IO_HAS_URDF)
  add_component_dependency_packages(
    ${dart_io_project_name}
    ${dart_io_component_name}
    urdfdom
  )
endif()

# In a static build the USD loader's private pxr link is exported from the
# installed dart-io target, so register pxr as a component dependency package.
# The generated io component config then loads dart_find_pxr.cmake and resolves
# OpenUSD for downstream static links. Shared builds resolve pxr at libdart-io
# link time, so this is static-only (mirrors dart-simulation's EnTT/Taskflow/
# spdlog registration).
if(DART_BUILD_IO_USD AND NOT BUILD_SHARED_LIBS)
  add_component_dependency_packages(
    ${dart_io_project_name}
    ${dart_io_component_name}
    pxr
  )
endif()

# Generate header for this namespace
dart_get_filename_components(header_names "io headers" ${hdrs})
if(DART_IO_HAS_URDF)
  list(APPEND header_names "urdf/All.hpp")
endif()
list(APPEND header_names "mjcf/All.hpp")
if(DART_ENABLE_SDFORMAT)
  list(APPEND header_names "sdf/All.hpp")
endif()
dart_generate_component_headers(
  COMPONENT_NAME io
  TARGET_DIR "dart/io/"
  OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}"
  HEADERS ${header_names}
  SOURCE_HEADERS ${hdrs} ${detail_hdrs} ${dart_io_headers}
)

# Install (includes All.hpp, io.hpp for component compatibility,
# and generated PascalCase headers for case compatibility)
install(
  FILES
    ${hdrs}
    ${CMAKE_CURRENT_BINARY_DIR}/All.hpp
    ${CMAKE_CURRENT_BINARY_DIR}/io.hpp
  DESTINATION include/dart/io
  COMPONENT headers
)
install(
  FILES ${detail_hdrs}
  DESTINATION include/dart/io/detail
  COMPONENT headers
)

# Install generated compatibility headers preserving subdirectory structure
dart_install_compat_headers(
  COMPAT_HEADERS ${DART_GENERATED_COMPAT_HEADERS}
  DESTINATION_PREFIX include/dart/io
)

dart_format_add(
  ${hdrs}
  ${srcs}
  ${detail_hdrs}
  ${detail_srcs}
  ${dart_io_headers}
  ${dart_io_sources}
)
