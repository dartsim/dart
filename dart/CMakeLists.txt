# Enable multi-threaded compilation.
# We do this here and not in the root folder since the examples and the
# tutorials do not have enough source files to benefit from this.
if(MSVC AND ${CMAKE_CXX_COMPILER_ID} STREQUAL "MSVC")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
endif()

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${DART_BINARY_DIR}/lib")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${DART_BINARY_DIR}/lib")

#===============================================================================
# Source directories
#===============================================================================
# common
# math
# lcpsolver
# optimizer (legacy alias headers for backward compatibility)
# dynamics
# collision
#   dart
#   fcl
#   bullet
# constraint
# simulation
# utils
#   sdf
#   urdf
# gui
#   osg
#     render

#===============================================================================
# Targets - {dependency targets}, source directories, [external dependency libs]
#===============================================================================
# dart - common, math (including math/optimization), lcpsolver, dynamics,
#        collision (fcl + optional Bullet/ODE), constraint, simulation, utils,
#        gui, [assimp, eigen3, fcl, fmt]
# dart-utils - {dart}, utils, utils/sdf, [tinyxml2]
# dart-utils-urdf - {dart-utils}, utils/urdf, [urdfdom]
# dart-gui - {dart-utils}, gui/osg, gui/osg/render, [openscenegraph]

#===============================================================================
# Components - (dependency component), {dependency targets}
#===============================================================================
# dart - {dart}
# utils - (dart), {dart-utils}
# utils-urdf - (utils), {dart-utils-urdf}
# gui - (utils), {dart-gui}

# Component. This should be called before adding subdirectories.
add_component(${PROJECT_NAME} dart)

function(dart_add_core_headers)
  dart_property_add(DART_CORE_HEADERS ${ARGN})
endfunction()

function(dart_add_core_sources)
  dart_property_add(DART_CORE_SOURCES ${ARGN})
endfunction()

add_subdirectory(common)
add_subdirectory(math)
add_subdirectory(lcpsolver)
add_subdirectory(optimizer)
add_subdirectory(dynamics)
add_subdirectory(collision)
add_subdirectory(constraint)
add_subdirectory(simulation)
add_subdirectory(utils) # tinyxml2, libsdformat
add_subdirectory(gui) # osg

set(DART_CONFIG_HPP_IN ${DART_SOURCE_DIR}/dart/config.hpp.in)
set(DART_CONFIG_HPP_OUT ${DART_BINARY_DIR}/dart/config.hpp)
if(DART_VERBOSE)
  message(STATUS ${DART_CONFIG_HPP_OUT})
endif()
configure_file(${DART_CONFIG_HPP_IN} ${DART_CONFIG_HPP_OUT} @ONLY)
install(FILES ${DART_CONFIG_HPP_OUT} DESTINATION include/dart)

# Print building component
get_property(components GLOBAL PROPERTY ${PROJECT_NAME}_COMPONENTS)
if(DART_VERBOSE)
  message(STATUS "")
  message(STATUS "[ Components ]")
  foreach(component IN LISTS components)
    message(STATUS "Adding component: ${component}")
  endforeach()
else()
  list(LENGTH components components_length)
  message(
    STATUS
    "Adding ${components_length} components including the default component 'dart'"
  )
endif()

# Set header and source files
get_filename_component(dart_hpp "dart.hpp" ABSOLUTE)
get_filename_component(dart_all_hpp "All.hpp" ABSOLUTE)
get_filename_component(dart_export_hpp "Export.hpp" ABSOLUTE)
dart_add_core_headers(${dart_hpp} ${dart_all_hpp} ${dart_export_hpp})
get_property(dart_core_headers GLOBAL PROPERTY DART_CORE_HEADERS)
get_property(dart_core_sources GLOBAL PROPERTY DART_CORE_SOURCES)

# Add target
dart_add_library(dart ${dart_core_headers} ${dart_core_sources})
target_include_directories(
  dart BEFORE
  PUBLIC
    $<BUILD_INTERFACE:${DART_SOURCE_DIR}>
    $<BUILD_INTERFACE:${DART_BINARY_DIR}>
    $<INSTALL_INTERFACE:include>
)
get_property(_dart_extra_includes GLOBAL PROPERTY DART_PUBLIC_INCLUDE_DIRS)
if(_dart_extra_includes)
  list(REMOVE_DUPLICATES _dart_extra_includes)
  target_include_directories(dart PUBLIC ${_dart_extra_includes})
endif()
get_property(_dart_system_includes GLOBAL PROPERTY DART_PUBLIC_SYSTEM_INCLUDE_DIRS)
if(_dart_system_includes)
  list(REMOVE_DUPLICATES _dart_system_includes)
  target_include_directories(dart SYSTEM PUBLIC ${_dart_system_includes})
endif()
target_link_libraries(
  dart
  PUBLIC
    ${CMAKE_DL_LIBS}
    Eigen3::Eigen
    fcl
    assimp
)
target_compile_definitions(dart PUBLIC BOOST_ALL_NO_EMBEDDED_GDB_SCRIPTS)
get_property(_dart_extra_compile_options GLOBAL PROPERTY DART_EXTRA_COMPILE_OPTIONS)
if(_dart_extra_compile_options)
  list(REMOVE_DUPLICATES _dart_extra_compile_options)
  target_compile_options(dart PRIVATE ${_dart_extra_compile_options})
endif()
if(TARGET fmt::fmt-header-only)
  target_link_libraries(dart PUBLIC fmt::fmt-header-only)
elseif(TARGET fmt::fmt)
  target_link_libraries(dart PUBLIC fmt::fmt)
else()
  message(FATAL_ERROR "Failed to find fmt targets")
endif()
get_property(_dart_extra_link_libs GLOBAL PROPERTY DART_EXTRA_LINK_LIBS)
if(_dart_extra_link_libs)
  target_link_libraries(dart PUBLIC ${_dart_extra_link_libs})
endif()

# spdlog settings
if(spdlog_FOUND)
  # Prefer header only target
  if(TARGET spdlog::spdlog_header_only)
    target_link_libraries(dart PUBLIC spdlog::spdlog_header_only)
    target_compile_definitions(dart PUBLIC -DDART_HAVE_spdlog=1)
  elseif(TARGET spdlog::spdlog)
    target_link_libraries(dart PUBLIC spdlog::spdlog)
    target_compile_definitions(dart PUBLIC -DDART_HAVE_spdlog=1)
  else()
    message(WARNING "spdlog found, but no target is found. Expected spdlog::spdlog or target_link_library_public spdlog::spdlog_header_only")
    target_compile_definitions(dart PUBLIC -DDART_HAVE_spdlog=0)
  endif()
else()
  target_compile_definitions(dart PUBLIC -DDART_HAVE_spdlog=0)
endif()

# octomap settings
if(TARGET octomap)
  target_link_libraries(dart PUBLIC octomap)
endif()

# C++ standard settings
target_compile_features(dart PUBLIC cxx_std_20)

# Build DART with all available SIMD instructions
if(DART_ENABLE_SIMD)
  if(MSVC)
    target_compile_options(dart PUBLIC /arch:SSE2 /arch:AVX /arch:AVX2)
    # /arch:SSE2 option is effective only for x86 but not for x64 since it's
    # enabled for x64 by default. The option will be ignored on x64 with a
    # warning message. If anyone knows how to detect the system's architecture
    # in CMake time, then I would gratefully apply these options differently
    # depending on the architectures.
  elseif(CMAKE_COMPILER_IS_GNUCXX)
    target_compile_options(dart PUBLIC -march=native)
    if(GCC_VERSION VERSION_GREATER 7.0)
      target_compile_options(dart PUBLIC -faligned-new)
    endif()
  elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    target_compile_options(dart PUBLIC -march=native -faligned-new)
  endif()
else()
  # TODO: Improve this part to detect the system's architecture and apply the
  # appropriate SIMD instructions.
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm64")
    # Remove SSE flags if set elsewhere
    string(REPLACE "-msse" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
    string(REPLACE "-msse2" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
    string(REPLACE "-msse3" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
    string(REPLACE "-mssse3" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
  endif()
endif()

# Active log level
if(DART_ACTIVE_LOG_LEVEL STREQUAL "TRACE")
  target_compile_definitions(dart PUBLIC -DDART_ACTIVE_LOG_LEVEL=0)
elseif(DART_ACTIVE_LOG_LEVEL STREQUAL "DEBUG")
  target_compile_definitions(dart PUBLIC -DDART_ACTIVE_LOG_LEVEL=1)
elseif(DART_ACTIVE_LOG_LEVEL STREQUAL "INFO")
  target_compile_definitions(dart PUBLIC -DDART_ACTIVE_LOG_LEVEL=2)
elseif(DART_ACTIVE_LOG_LEVEL STREQUAL "WARN")
  target_compile_definitions(dart PUBLIC -DDART_ACTIVE_LOG_LEVEL=3)
elseif(DART_ACTIVE_LOG_LEVEL STREQUAL "ERROR")
  target_compile_definitions(dart PUBLIC -DDART_ACTIVE_LOG_LEVEL=4)
elseif(DART_ACTIVE_LOG_LEVEL STREQUAL "FATAL")
  target_compile_definitions(dart PUBLIC -DDART_ACTIVE_LOG_LEVEL=5)
elseif(DART_ACTIVE_LOG_LEVEL STREQUAL "OFF")
  target_compile_definitions(dart PUBLIC -DDART_ACTIVE_LOG_LEVEL=6)
else()
  message(FATAL_ERROR "Invalid active log level: ${DART_ACTIVE_LOG_LEVEL}")
endif()

# Default component
add_component_targets(${PROJECT_NAME} dart dart)
add_component_dependency_packages(${PROJECT_NAME} dart
  assimp Eigen3 fcl fmt
)
if(TARGET octomap)
  add_component_dependency_packages(${PROJECT_NAME} dart octomap)
endif()
if(spdlog_FOUND)
  add_component_dependency_packages(${PROJECT_NAME} dart spdlog)
endif()

if(MSVC)
  set_target_properties(
    dart PROPERTIES
    STATIC_LIBRARY_FLAGS_RELEASE "/LTCG"
  )
endif()

if(DART_CODECOV)
  target_link_libraries(dart PUBLIC coverage_config)
endif()

if(DART_BUILD_PROFILE)
  target_compile_definitions(dart PUBLIC
    DART_PROFILE_ENABLE_TEXT=$<BOOL:${DART_PROFILE_TEXT}>
    DART_PROFILE_ENABLE_TRACY=$<BOOL:${DART_PROFILE_TRACY}>
  )

  if(DART_PROFILE_TRACY)
    target_link_libraries(dart PUBLIC Tracy::TracyClient)
  endif()
endif()

install(
  FILES ${dart_hpp} ${dart_all_hpp} ${dart_export_hpp}
  DESTINATION include/dart/
  COMPONENT headers
)

dart_format_add(${dart_core_headers} ${dart_core_sources})
