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
# optimizer
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
#   detail

#===============================================================================
# Targets - {dependency targets}, source directories, [external dependency libs]
#===============================================================================
# dart - common, math, lcpsolver, optimizer, dynamics, collision,
#        collision/dart, collision/fcl, optional collision/bullet and
#        collision/ode, constraint, simulation, [assimp, eigen3, fcl]
# dart-collision-bullet, dart-collision-ode - forwarding interfaces to dart
# dart-io - {dart}, utils, utils/mjcf, utils/sdf, utils/urdf, [tinyxml2, urdfdom]
# dart-utils, dart-utils-urdf - forwarding interfaces to dart-io
# dart-gui - {dart}, gui, gui/detail, gui/imgui, [filament, glfw, png, jpeg]
# dart-external-imgui - forwarding interface to dart-gui

#===============================================================================
# Components - (dependency component), {dependency targets}
#===============================================================================
# dart - {dart}
# collision-bullet - (dart), {dart-collision-bullet}
# io - (dart), {dart-io}
# utils - (io), {dart-utils}
# utils-urdf - (utils), {dart-utils-urdf}
# gui - (dart), {dart-gui}
# external-imgui - (gui), {dart-external-imgui}

# Component. This should be called before adding subdirectories.
add_component(${PROJECT_NAME} dart)

function(dart_add_core_headers)
  dart_property_add(DART_CORE_HEADERS ${ARGN})
endfunction(dart_add_core_headers)

function(dart_add_core_sources)
  dart_property_add(DART_CORE_SOURCES ${ARGN})
endfunction(dart_add_core_sources)

# Add subdirectories
add_subdirectory(common)
add_subdirectory(math)
add_subdirectory(lcpsolver)
add_subdirectory(optimizer)
add_subdirectory(dynamics)
add_subdirectory(collision)
add_subdirectory(constraint)
add_subdirectory(simulation)
if(DART_BUILD_IO)
  add_subdirectory(utils) # tinyxml2, urdfdom
endif()
if(DART_BUILD_GUI)
  add_subdirectory(gui) # Filament
endif()
add_subdirectory(simd) # header-only SIMD module

get_property(
  dart_core_compile_definitions
  GLOBAL
  PROPERTY DART_CORE_COMPILE_DEFINITIONS
)
set(BT_USE_DOUBLE_PRECISION FALSE)
if("BT_USE_DOUBLE_PRECISION" IN_LIST dart_core_compile_definitions)
  set(BT_USE_DOUBLE_PRECISION TRUE)
endif()
if(NOT TARGET dart-collision-ode)
  set(DART_ODE_HAS_LIBCCD_BOX_CYL FALSE)
endif()

set(DART_CONFIG_HPP_IN ${DART_SOURCE_DIR}/dart/config.hpp.in)
set(DART_CONFIG_HPP_OUT ${DART_BINARY_DIR}/dart/config.hpp)
if(DART_VERBOSE)
  message(STATUS ${DART_CONFIG_HPP_OUT})
endif()
configure_file(${DART_CONFIG_HPP_IN} ${DART_CONFIG_HPP_OUT} @ONLY)
install(FILES ${DART_CONFIG_HPP_OUT} DESTINATION include/dart)
# Also in the headers component; the rule above keeps the default component's
# contents unchanged.
install(
  FILES ${DART_CONFIG_HPP_OUT}
  DESTINATION include/dart
  COMPONENT headers
  EXCLUDE_FROM_ALL
)

# Print building component
get_property(components GLOBAL PROPERTY ${PROJECT_NAME}_COMPONENTS)
if(DART_VERBOSE)
  message(STATUS "")
  message(STATUS "[ Components ]")
  foreach(component ${components})
    message(STATUS "Adding component: ${component}")
  endforeach()
else()
  list(LENGTH components components_length)
  message(
    STATUS
    "Adding ${components_length} components including "
    "the default component 'dart'"
  )
endif()

# Set header and source files
get_filename_component(dart_hpp "dart.hpp" ABSOLUTE)
dart_add_core_headers(${dart_hpp})
get_property(dart_core_headers GLOBAL PROPERTY DART_CORE_HEADERS)
get_property(dart_core_sources GLOBAL PROPERTY DART_CORE_SOURCES)

# Add target
dart_add_library(dart ${dart_core_headers} ${dart_core_sources})
# DantzigLcp-impl.hpp defines its LCP template only under dLCP_FAST, and its
# include guard lets the first inclusion decide. Define it for every source so a
# unity batch whose other files include the header first still gets it.
target_compile_definitions(dart PRIVATE dLCP_FAST)
target_compile_definitions(dart PUBLIC ${dart_core_compile_definitions})
target_include_directories(
  dart
  BEFORE
  PUBLIC
    $<BUILD_INTERFACE:${DART_SOURCE_DIR}>
    $<BUILD_INTERFACE:${DART_BINARY_DIR}>
    $<INSTALL_INTERFACE:include>
)
get_property(dart_core_link_libraries GLOBAL PROPERTY DART_CORE_LINK_LIBRARIES)
target_link_libraries(
  dart
  PUBLIC
    ${CMAKE_DL_LIBS}
    Threads::Threads
    Eigen3::Eigen
    fcl
    assimp
    ${dart_core_link_libraries}
)

# assimp version detection for conditional compilation
if(ASSIMP_VERSION_MAJOR)
  target_compile_definitions(
    dart
    PUBLIC DART_ASSIMP_VERSION_MAJOR=${ASSIMP_VERSION_MAJOR}
  )
endif()

# octomap settings
if(TARGET octomap)
  target_link_libraries(dart PUBLIC octomap)
endif()

# C++ standard settings
target_compile_features(dart PUBLIC cxx_std_17)

# DART's public templates need the MSVC parsing mode DART is compiled with
# (see dart_defs.cmake), so consumers inherit it as well.
foreach(option IN LISTS DART_MSVC_CXX_PARSING_OPTIONS)
  target_compile_options(
    dart
    INTERFACE "$<$<COMPILE_LANG_AND_ID:CXX,MSVC>:${option}>"
  )
endforeach()

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
  elseif("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
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

# Default component
add_component_targets(${PROJECT_NAME} dart dart)
get_property(
  dart_core_dependency_packages
  GLOBAL
  PROPERTY DART_CORE_DEPENDENCY_PACKAGES
)
add_component_dependency_packages(${PROJECT_NAME} dart
  assimp Eigen3 fcl Threads ${dart_core_dependency_packages}
)
if(TARGET octomap)
  add_component_dependency_packages(${PROJECT_NAME} dart octomap)
endif()

if(MSVC AND NOT DART_MSVC_DISABLE_WHOLE_PROGRAM_OPTIMIZATION)
  # /LTCG pairs with the /GL whole-program objects; skip it when whole-program
  # optimization is disabled so no link-time codegen pass remains.
  set_target_properties(dart PROPERTIES STATIC_LIBRARY_FLAGS_RELEASE "/LTCG")
endif()

if(DART_CODECOV)
  target_link_libraries(dart PUBLIC coverage_config)
endif()

if(DART_BUILD_PROFILE)
  target_compile_definitions(
    dart
    PUBLIC
      DART_PROFILE_ENABLE_TEXT=$<BOOL:${DART_PROFILE_BUILTIN}>
      DART_PROFILE_ENABLE_TRACY=$<BOOL:${DART_PROFILE_TRACY}>
  )

  if(DART_PROFILE_TRACY)
    target_link_libraries(dart PUBLIC Tracy::TracyClient)
  endif()
endif()

install(FILES dart.hpp DESTINATION include/dart/ COMPONENT headers)

dart_format_add(
  ${dart_core_headers}
  ${dart_core_sources}
  ${PROJECT_SOURCE_DIR}/scripts/check_logging_float.cpp
)
