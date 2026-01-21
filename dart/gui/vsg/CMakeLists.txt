# VSG-based GUI module for DART
#
# This module provides VulkanSceneGraph (VSG) integration for collision
# visualization and future OSG replacement.

if(NOT DART_BUILD_GUI_VSG)
  message(STATUS "Skipping dart-gui-vsg (DART_BUILD_GUI_VSG == ${DART_BUILD_GUI_VSG})")
  return()
endif()

set(target_name ${PROJECT_NAME}-gui-vsg)
set(component_name gui-vsg)
set(component_dependencies)
set(component_dependency_packages)
set(link_libraries)

# VulkanSceneGraph - use find_package directly (VSG provides CMake config)
find_package(vsg CONFIG QUIET)
if(NOT vsg_FOUND)
  message(STATUS "Skipping dart-gui-vsg (VulkanSceneGraph not found)")
  return()
endif()
list(APPEND component_dependency_packages vsg)
list(APPEND link_libraries vsg::vsg)

# vsgImGui - fetch if not found (not on conda-forge)
option(DART_USE_SYSTEM_VSGIMGUI "Use system vsgImGui instead of FetchContent" OFF)
set(DART_VSGIMGUI_GIT_TAG "v0.7.0" CACHE STRING "vsgImGui git tag to fetch")

if(DART_USE_SYSTEM_VSGIMGUI)
  find_package(vsgImGui CONFIG QUIET)
endif()

if(DART_USE_SYSTEM_VSGIMGUI AND vsgImGui_FOUND)
  set(DART_HAS_VSGIMGUI TRUE)
  if(TARGET vsgImGui::vsgImGui)
    list(APPEND link_libraries vsgImGui::vsgImGui)
  elseif(TARGET vsgImGui)
    list(APPEND link_libraries vsgImGui)
  endif()
else()
  include(FetchContent)
  FetchContent_Declare(
    vsgImGui
    GIT_REPOSITORY https://github.com/vsg-dev/vsgImGui.git
    GIT_TAG ${DART_VSGIMGUI_GIT_TAG}
    GIT_SHALLOW TRUE
  )
  FetchContent_GetProperties(vsgImGui)
  if(NOT vsgimgui_POPULATED)
    cmake_policy(PUSH)
    if(POLICY CMP0169)
      cmake_policy(SET CMP0169 OLD)
    endif()
    FetchContent_Populate(vsgImGui)
    cmake_policy(POP)
  endif()
  
  if(NOT TARGET docs)
    add_custom_target(docs)
  endif()
  if(NOT TARGET clobber)
    add_custom_target(clobber)
  endif()
  if(NOT TARGET uninstall)
    add_custom_target(uninstall)
  endif()
  
  if(NOT TARGET vsgImGui)
    set(CMAKE_CXX_STANDARD_SAVED ${CMAKE_CXX_STANDARD})
    set(BUILD_SHARED_LIBS_SAVED ${BUILD_SHARED_LIBS})
    add_subdirectory(${vsgimgui_SOURCE_DIR} ${vsgimgui_BINARY_DIR} EXCLUDE_FROM_ALL)
    set(BUILD_SHARED_LIBS ${BUILD_SHARED_LIBS_SAVED})
    set(CMAKE_CXX_STANDARD ${CMAKE_CXX_STANDARD_SAVED})
  endif()
  
  set(DART_HAS_VSGIMGUI TRUE)
  if(TARGET vsgImGui::vsgImGui)
    message(STATUS "dart-gui-vsg: Found vsgImGui::vsgImGui alias target")
    list(APPEND link_libraries vsgImGui::vsgImGui)
  elseif(TARGET vsgImGui)
    message(STATUS "dart-gui-vsg: Found vsgImGui target (no alias)")
    list(APPEND link_libraries vsgImGui)
  else()
    message(WARNING "dart-gui-vsg: DART_HAS_VSGIMGUI=TRUE but no vsgImGui target found!")
  endif()
endif()

# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")

set(dart_gui_vsg_hdrs ${hdrs})
set(dart_gui_vsg_srcs ${srcs})

# Add target
dart_add_library(${target_name} ${dart_gui_vsg_hdrs} ${dart_gui_vsg_srcs})

# Explicitly set C++20 for dart-gui-vsg because vsgImGui's FetchContent
# may have polluted CMAKE_CXX_STANDARD to 17
target_compile_features(${target_name} PUBLIC cxx_std_20)

target_include_directories(
  ${target_name}
  PUBLIC
    $<BUILD_INTERFACE:${DART_SOURCE_DIR}>
    $<BUILD_INTERFACE:${DART_BINARY_DIR}>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${target_name}
  PUBLIC
    Eigen3::Eigen
    ${link_libraries}
)

# Link to collision-experimental for createFromShape()
# Use generator expression since collision-experimental is added after gui/vsg
if(DART_BUILD_COLLISION_EXPERIMENTAL)
  target_link_libraries(${target_name} PUBLIC ${PROJECT_NAME}-collision-experimental)
endif()

# Add vsgImGui compile definition
if(DART_HAS_VSGIMGUI)
  target_compile_definitions(${target_name} PUBLIC DART_HAS_VSGIMGUI)
endif()

if(MSVC AND BUILD_SHARED_LIBS)
  if(CMAKE_CONFIGURATION_TYPES)
    foreach(_cfg ${CMAKE_CONFIGURATION_TYPES})
      string(TOUPPER "${_cfg}" _cfg_upper)
      set_target_properties(
        ${target_name}
        PROPERTIES
          ARCHIVE_OUTPUT_DIRECTORY_${_cfg_upper}
            "${DART_BINARY_DIR}/lib/${_cfg}"
      )
    endforeach()
  else()
    set_target_properties(
      ${target_name}
      PROPERTIES
        ARCHIVE_OUTPUT_DIRECTORY "${DART_BINARY_DIR}/lib/${CMAKE_BUILD_TYPE}"
    )
  endif()
endif()

# Component
add_component(${PROJECT_NAME} ${component_name})
add_component_targets(${PROJECT_NAME} ${component_name} ${target_name})
add_component_dependencies(${PROJECT_NAME} ${component_name} ${component_dependencies})
add_component_dependency_packages(${PROJECT_NAME} ${component_name} ${component_dependency_packages})

# Generate header for this namespace
dart_get_filename_components(header_names "gui/vsg headers" ${hdrs})
dart_generate_component_headers(
  COMPONENT_NAME vsg
  TARGET_DIR "dart/gui/vsg/"
  OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}"
  HEADERS ${header_names}
  SOURCE_HEADERS ${hdrs}
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/vsg.hpp
  DESTINATION include/dart/gui/vsg
  COMPONENT headers
)

dart_format_add(${hdrs} ${srcs})
