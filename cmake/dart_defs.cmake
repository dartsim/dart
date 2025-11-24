# Copyright (c) The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/master/LICENSE
#
# This file is provided under the following "BSD-style" License:
#   Redistribution and use in source and binary forms, with or
#   without modification, are permitted provided that the following
#   conditions are met:
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
#   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
#   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
#   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
#   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
#   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
#   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#   POSSIBILITY OF SUCH DAMAGE.

#===============================================================================
# Utility Functions
#===============================================================================

#-------------------------------------------------------------------------------
# Get maximum value from a list of values
# Usage:
#   dart_get_max(var [value1 value2...])
#-------------------------------------------------------------------------------
function(dart_get_max var)
  if(NOT ARGN)
    set(${var} "" PARENT_SCOPE)
    return()
  endif()

  list(GET ARGN 0 choice)
  foreach(item IN LISTS ARGN)
    if(choice LESS item)
      set(choice ${item})
    endif()
  endforeach()
  set(${var} ${choice} PARENT_SCOPE)
endfunction()

#-------------------------------------------------------------------------------
# Get maximum string length from a list of strings
# Usage:
#   dart_get_max_string_length(var [value1 value2...])
#-------------------------------------------------------------------------------
function(dart_get_max_string_length var)
  set(lengths)
  foreach(item IN LISTS ARGN)
    string(LENGTH "${item}" length)
    list(APPEND lengths ${length})
  endforeach()
  dart_get_max(choice ${lengths})
  set(${var} ${choice} PARENT_SCOPE)
endfunction()

#-------------------------------------------------------------------------------
# Get list of subdirectories in a given directory
# Usage:
#   dart_get_subdir_list(var curdir)
#-------------------------------------------------------------------------------
function(dart_get_subdir_list var curdir)
  file(GLOB children RELATIVE "${curdir}" "${curdir}/*")
  set(dirlist)
  foreach(child IN LISTS children)
    if(IS_DIRECTORY "${curdir}/${child}")
      list(APPEND dirlist "${child}")
    endif()
  endforeach()
  set(${var} "${dirlist}" PARENT_SCOPE)
endfunction()

#-------------------------------------------------------------------------------
# Add or append property to a global property list
# Usage:
#   dart_property_add(property_name value1 [value2...])
#-------------------------------------------------------------------------------
function(dart_property_add property_name)
  get_property(is_defined GLOBAL PROPERTY ${property_name} DEFINED)

  if(NOT is_defined)
    define_property(GLOBAL PROPERTY ${property_name}
      BRIEF_DOCS "${property_name}"
      FULL_DOCS "Global properties for ${property_name}"
    )
  endif()

  foreach(item IN LISTS ARGN)
    set_property(GLOBAL APPEND PROPERTY ${property_name} "${item}")
  endforeach()
endfunction()

#===============================================================================
# Configuration and Options
#===============================================================================

#-------------------------------------------------------------------------------
# Define a DART option with tracking for display
# Usage:
#   dart_option(<variable> "<help_text>" <value>)
#-------------------------------------------------------------------------------
function(dart_option variable help_text default_value)
  set(option_category "")

  if(ARGC GREATER 3)
    cmake_parse_arguments(
      DART_OPTION
      ""
      ""
      "CATEGORY"
      ${ARGN}
    )

    if(DART_OPTION_UNPARSED_ARGUMENTS)
      message(
        FATAL_ERROR
        "dart_option(${variable} ...): unrecognized arguments: ${DART_OPTION_UNPARSED_ARGUMENTS}"
      )
    endif()

    if(DART_OPTION_CATEGORY)
      set(option_category "${DART_OPTION_CATEGORY}")
    endif()
  endif()

  set_property(
    GLOBAL PROPERTY DART_DETAIL_PROPERTY_OPTION_VARIABLE "${variable}" APPEND
  )
  set_property(
    GLOBAL PROPERTY DART_DETAIL_property_option_help_text "${help_text}" APPEND
  )
  set_property(
    GLOBAL PROPERTY DART_DETAIL_property_option_default_value "${default_value}"
    APPEND
  )
  if(option_category STREQUAL "")
    set(option_category "__DART_AUTO__")
  endif()
  set_property(
    GLOBAL PROPERTY DART_DETAIL_property_option_category "${option_category}" APPEND
  )

  # Add option
  option(${variable} ${help_text} ${default_value})

  # Normalize boolean value variants (e.g. 1/0, On/Off, TRUE/FALSE) to ON/OFF
  if(${variable})
    set(${variable} ON PARENT_SCOPE)
  else()
    set(${variable} OFF PARENT_SCOPE)
  endif()
endfunction()

#-------------------------------------------------------------------------------
# Print all DART options in a formatted table
# Usage:
#   dart_print_options()
#-------------------------------------------------------------------------------
function(dart_print_options)
  message(STATUS "[ Options ]")

  get_property(
    option_variables GLOBAL PROPERTY DART_DETAIL_PROPERTY_OPTION_VARIABLE
  )
  get_property(
    option_help_texts GLOBAL PROPERTY DART_DETAIL_property_option_help_text
  )
  get_property(
    option_default_values GLOBAL
    PROPERTY DART_DETAIL_property_option_default_value
  )
  get_property(
    option_categories GLOBAL PROPERTY DART_DETAIL_property_option_category
  )

  dart_get_max_string_length(option_variable_max_len ${option_variables})
  list(LENGTH option_variables option_count)
  if(option_count GREATER 0)
    math(EXPR option_count "${option_count} - 1")
  else()
    return()
  endif()

  set(group_order
    cmake
    build
    performance
    system
    msvc
    diagnostics
    other
  )
  set(group_label_build "Build Outputs")
  set(group_label_cmake "CMake Globals")
  set(group_label_performance "Performance & Debug")
  set(group_label_system "System Integrations")
  set(group_label_msvc "MSVC Toolchain")
  set(group_label_diagnostics "Developer Experience")
  set(group_label_other "Other")

  foreach(group IN LISTS group_order)
    set(group_${group})
  endforeach()

  foreach(val RANGE ${option_count})
    list(GET option_variables ${val} option_variable)
    list(GET option_default_values ${val} option_default_value)

    set(option_str "- ${option_variable}")
    set(spaces "")
    string(LENGTH ${option_variable} option_variable_len)
    math(EXPR space_count "${option_variable_max_len} - ${option_variable_len}")
    foreach(loop_var RANGE ${space_count})
      set(option_str "${option_str} ")
    endforeach()

    set(option_str "${option_str}: ${${option_variable}}")

    if(${option_variable} STREQUAL option_default_value)
      set(option_str "${option_str} [default]")
    endif()

    set(group_key "")
    list(GET option_categories ${val} option_category_raw)
    set(option_category "${option_category_raw}")
    if(option_category STREQUAL "__DART_AUTO__")
      set(option_category "")
    endif()
    string(STRIP "${option_category}" option_category)

    if(option_category)
      string(REGEX REPLACE "[^A-Za-z0-9]+" "_" option_category_key "${option_category}")
      string(TOLOWER "${option_category_key}" option_category_key)
      string(REGEX REPLACE "^_+|_+$" "" option_category_key "${option_category_key}")

      if(option_category_key STREQUAL "build" OR option_category_key STREQUAL "build_outputs")
        set(group_key build)
      elseif(option_category_key STREQUAL "performance" OR option_category_key STREQUAL "performance_debug" OR option_category_key STREQUAL "debug")
        set(group_key performance)
      elseif(option_category_key STREQUAL "system" OR option_category_key STREQUAL "system_integrations")
        set(group_key system)
      elseif(option_category_key STREQUAL "msvc" OR option_category_key STREQUAL "msvc_toolchain")
        set(group_key msvc)
      elseif(option_category_key STREQUAL "diagnostics" OR option_category_key STREQUAL "developer_experience")
        set(group_key diagnostics)
      elseif(option_category_key STREQUAL "other")
        set(group_key other)
      elseif(option_category_key)
        set(group_key "custom_${option_category_key}")
        if(NOT DEFINED group_label_${group_key})
          set(group_label_${group_key} "${option_category}")
        endif()
        list(FIND group_order ${group_key} _group_index)
        if(_group_index EQUAL -1)
          list(APPEND group_order ${group_key})
        endif()
      endif()
    endif()

    if(NOT group_key)
      if(option_variable MATCHES "^DART_USE_SYSTEM_")
        set(group_key system)
      elseif(option_variable MATCHES "^DART_MSVC_" OR option_variable STREQUAL "DART_RUNTIME_LIBRARY")
        set(group_key msvc)
      elseif(option_variable STREQUAL "BUILD_SHARED_LIBS"
          OR (option_variable MATCHES "^DART_BUILD_" AND NOT option_variable STREQUAL "DART_BUILD_PROFILE"))
        set(group_key build)
      elseif(option_variable MATCHES "^DART_(BUILD_PROFILE|FAST_DEBUG|ENABLE_SIMD|CODECOV|ENABLE_ASAN)$")
        set(group_key performance)
      elseif(option_variable MATCHES "^DART_(FORCE_COLORED_OUTPUT|VERBOSE)$")
        set(group_key diagnostics)
      else()
        set(group_key other)
      endif()
    endif()

    list(APPEND group_${group_key} "${option_str}")
  endforeach()

  if(BUILD_SHARED_LIBS)
    set(group_cmake "- BUILD_SHARED_LIBS : ON [default]")
  else()
    set(group_cmake "- BUILD_SHARED_LIBS : OFF")
  endif()

  foreach(group IN LISTS group_order)
    set(group_entries ${group_${group}})
    if(group_entries)
      set(group_label_var "group_label_${group}")
      set(group_label "${${group_label_var}}")
      message(STATUS "- ${group_label}")
      foreach(entry IN LISTS group_entries)
        message(STATUS "  ${entry}")
      endforeach()
    endif()
  endforeach()

  message(STATUS "")
endfunction()

function(dart_library)
  set(prefix _ARG)
  set(options
    GLOB_HEADERS
    GLOB_SOURCES
  )
  set(oneValueArgs
    NAME
  )
  set(multiValueArgs
    HEADERS
    SOURCES
    PUBLIC_LINK_LIBRARIES
    PRIVATE_LINK_LIBRARIES
    PUBLIC_COMPILE_DEFINITIONS
    PRIVATE_COMPILE_DEFINITIONS
  )
  cmake_parse_arguments(
    "${prefix}" "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN}
  )

  # Get the current directory relative to the root of the project
  # assuming that dart/* is the root of the source tree
  string(REPLACE "${CMAKE_SOURCE_DIR}/" "" relative_path ${CMAKE_CURRENT_SOURCE_DIR})

  if(${_ARG_GLOB_HEADERS})
    file(GLOB_RECURSE headers "*.hpp")
    list(APPEND _ARG_HEADERS ${headers})
  endif()

  if(${_ARG_GLOB_SOURCES})
    file(GLOB_RECURSE sources "*.cpp")
    list(APPEND _ARG_SOURCES ${sources})
  endif()

  add_library(${_ARG_NAME}
    ${_ARG_HEADERS}
    ${_ARG_SOURCES}
  )

  target_include_directories(${_ARG_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  )

  target_compile_features(${_ARG_NAME} PUBLIC cxx_std_20)

  target_link_libraries(${_ARG_NAME}
    PUBLIC
      ${_ARG_PUBLIC_LINK_LIBRARIES}
    PRIVATE
      ${_ARG_PRIVATE_LINK_LIBRARIES}
  )

  target_compile_definitions(${_ARG_NAME}
    PUBLIC
      ${_ARG_PUBLIC_COMPILE_DEFINITIONS}
    PRIVATE
      ${_ARG_PRIVATE_COMPILE_DEFINITIONS}
  )

  set_target_properties(${_ARG_NAME} PROPERTIES
    OUTPUT_NAME dart-${_ARG_NAME}
  )

  include(GNUInstallDirs)

  foreach(header IN LISTS _ARG_HEADERS)
    # Compute the relative path of each header from the root_dir
    file(RELATIVE_PATH rel_path "${CMAKE_SOURCE_DIR}" "${header}")
    get_filename_component(rel_dir "${rel_path}" DIRECTORY)

    # Install the file to the destination, preserving the directory structure
    install(
        FILES "${header}"
        DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${rel_dir}"
    )
  endforeach()

  install(
    TARGETS ${_ARG_NAME}
    EXPORT dart_${_ARG_NAME}Targets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  )

  install(
    EXPORT dart_${_ARG_NAME}Targets
    NAMESPACE DART::
    DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake
  )

  dart_format_add(${_ARG_HEADERS} ${_ARG_SOURCES})

endfunction()

#===============================================================================
function(dart_coverage)
  set(prefix _ARG)
  set(options
    REQUIRED
  )
  set(oneValueArgs
    INCLUDE_DIR
    SOURCE_DIR
    INSTALL_DIR
  )
  set(multiValueArgs
    INPUT        # optional
    EXCLUDE
  )
  cmake_parse_arguments(
    "${prefix}" "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN}
  )

  include(GNUInstallDirs)

  if(NOT _ARG_INCLUDE_DIR)
    message(FATAL_ERROR "INCLUDE_DIR is not set")
  endif()

  if(NOT _ARG_SOURCE_DIR)
    message(FATAL_ERROR "SOURCE_DIR is not set")
  endif()

  if(NOT _ARG_INSTALL_DIR)
    set(_ARG_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_DOXDIR}/${PROJECT_NAME}${DART_VERSION_MAJOR}/coverage)
  endif()

  # Find gcovr
  if(NOT GCOVR_EXECUTABLE)
    find_program(GCOVR_EXECUTABLE gcovr QUIET)
    if(NOT GCOVR_EXECUTABLE)
      if(_ARG_REQUIRED)
        message(FATAL_ERROR "Failed to find gcovr. Install gcovr or remove REQUIRED option.")
      else()
        return()
      endif()
    endif()
  endif()

  # Set variables
  get_filename_component(_ARG_INCLUDE_DIR ${_ARG_INCLUDE_DIR} ABSOLUTE)
  set(gcovr_include_dir ${_ARG_INCLUDE_DIR})
  get_filename_component(_ARG_SOURCE_DIR ${_ARG_SOURCE_DIR} ABSOLUTE)
  set(gcovr_source_dir ${_ARG_SOURCE_DIR})
  set(gcovr_html_dir ${CMAKE_CURRENT_BINARY_DIR}/__coverage__)
  set(gcovr_index_path ${gcovr_html_dir}/index.html)

  # Extract Gcovr version
  execute_process(COMMAND ${GCOVR_EXECUTABLE} --version OUTPUT_VARIABLE GCOVR_VERSION_RAW_OUTPUT)
  string(STRIP ${GCOVR_VERSION_RAW_OUTPUT} GCOVR_VERSION_RAW_OUTPUT)
  string(REPLACE "gcovr " "" GCOVR_VERSION_RAW_OUTPUT ${GCOVR_VERSION_RAW_OUTPUT})

  # Set options based on the Gcovr version
  if(${GCOVR_VERSION_RAW_OUTPUT} VERSION_GREATER_EQUAL 4.2)
    set(gcovr_options --exclude-throw-branches)
  endif()

  add_custom_target(coverage
    COMMAND
      ${GCOVR_EXECUTABLE}
        -r ${CMAKE_CURRENT_SOURCE_DIR}
        -f ${gcovr_include_dir}
        -f ${gcovr_source_dir}
    DEPENDS tests_and_run
    COMMENT "Generating line coverage report..."
  )

  add_custom_target(coverage_branch
    COMMAND ${GCOVR_EXECUTABLE}
        -b
        ${gcovr_options}
        --exclude-unreachable-branches
        -r ${CMAKE_CURRENT_SOURCE_DIR}
        -f ${gcovr_include_dir}
        -f ${gcovr_source_dir}
    DEPENDS tests_and_run
    COMMENT "Generating branch coverage report..."
  )

  add_custom_target(coverage_html
    COMMAND ${CMAKE_COMMAND} -E make_directory ${gcovr_html_dir}
    COMMAND ${GCOVR_EXECUTABLE}
      --html
      --html-details
      ${gcovr_options}
      --exclude-unreachable-branches
      -o "${gcovr_index_path}"
      -r ${CMAKE_CURRENT_SOURCE_DIR}
      -f ${gcovr_include_dir}
      -f ${gcovr_source_dir}
    DEPENDS coverage coverage_branch
    COMMENT "Generating a detailed HTML coverage report in ${gcovr_index_path}"
  )

  if(APPLE)
    set(open_command_name "open")
  else()
    set(open_command_name "xdg-open")
  endif()
  find_program(open_command
    NAMES ${open_command_name}
    DOC "Path to ${open_command_name}"
  )

  if(open_command)
    add_custom_target(coverage_view "${open_command}" "${gcovr_index_path}"
      DEPENDS coverage_html
      COMMENT "Opening documentation in a web browser."
    )
  else()
    message(WARNING "Failed to find ${open_command_name}, to enable "
      "'coverage_view', please install xdg-utils"
    )
  endif()

  # Remove all gcovr data and delete the html directory
  add_custom_target(coverage_cleanup
    COMMAND rm -rf ${CMAKE_CURRENT_BINARY_DIR}/**/*.gcno
    COMMAND rm -rf ${CMAKE_CURRENT_BINARY_DIR}/**/*.gcda
    COMMAND rm -rf ${gcovr_html_dir}
    COMMENT "Removing stored coverage files..."
  )

  # Create the working directory ahead so that make install doesn't complain even when
  # make coverage is run before.
  file(MAKE_DIRECTORY ${gcovr_html_dir})

  # Install
  if(EXISTS ${gcovr_html_dir})
    install(DIRECTORY ${gcovr_html_dir}/ DESTINATION ${_ARG_INSTALL_DIR})
  endif()
endfunction()

function(dart_benchmarks)
  set(prefix _ARG)
  set(options)
  set(oneValueArgs
    TYPE
    TARGET_PREFIX
    TEST_LIST
  )
  set(multiValueArgs
    SOURCES
    INCLUDE_DIRS
    LINK_LIBRARIES
    LINK_DART_LIBRARIES
    COMPILE_DEFINITIONS
  )
  cmake_parse_arguments(
    "${prefix}" "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN}
  )

  if(NOT _ARG_TYPE)
    message(
      FATAL_ERROR "DEVELOPER ERROR: You must specify a TYPE for your benchmarks!"
    )
  endif()

  if(NOT DEFINED _ARG_SOURCES)
    message(STATUS "No benchmarks have been specified for ${_ARG_TYPE}")
  else()
    list(LENGTH _ARG_SOURCES num_benchmarks)
    message(STATUS "Adding ${num_benchmarks} ${_ARG_TYPE} benchmarks")
  endif()

  if(_ARG_TEST_LIST)
    set(${_ARG_TEST_LIST} "")
  endif()

  foreach(source IN LISTS _ARG_SOURCES)

    # Set target name: <TYPE>[_<TARGET_PREFIX>]_<source>
    set(target_name ${_ARG_TYPE})
    if(_ARG_TARGET_PREFIX)
      set(target_name "${target_name}_${_ARG_TARGET_PREFIX}")
    endif()
    get_filename_component(source_name ${source} NAME_WE)
    string(REPLACE "bm_" "" source_name ${source_name})
    get_filename_component(source_dir ${source} DIRECTORY)
    if(source_dir)
      string(REPLACE "/" "_" source_prefix ${source_dir})
      set(target_name "${target_name}_${source_prefix}_${source_name}")
    else()
      set(target_name "${target_name}_${source_name}")
    endif()

    add_executable(${target_name} ${source})
    target_include_directories(
      ${target_name} PRIVATE ${_ARG_INCLUDE_DIRS}
    )

    target_link_libraries(${target_name} PRIVATE benchmark::benchmark benchmark::benchmark_main)

    if(UNIX)
      # gbenchmark requies pthread when compiled on a Unix machine
      target_link_libraries(${target_name} PRIVATE pthread)
    endif()

    target_link_libraries(
      ${target_name} PRIVATE ${_ARG_LINK_LIBRARIES}
    )

    target_compile_definitions(
      ${target_name} PRIVATE ${_ARG_COMPILE_DEFINITIONS}
    )

    foreach(dart_lib IN LISTS _ARG_LINK_DART_LIBRARIES)
      if(NOT TARGET ${dart_lib})
        message(FATAL_ERROR "Invalid target: ${dart_lib}")
      endif()
      target_link_libraries(${target_name} PRIVATE ${dart_lib})
    endforeach()

    set_target_properties(${target_name}
      PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${DART_BINARY_DIR}/bin"
    )

    if(_ARG_TEST_LIST)
      list(APPEND ${_ARG_TEST_LIST} ${target_name})
    endif()

    # Add executable target
    add_custom_target(
      RUN_${target_name}
      COMMAND ${target_name}
      COMMENT "Running benchmark ${target_name}..."
      VERBATIM
    )

    dart_property_add(DART_${_ARG_TYPE}_BENCHMARKS ${target_name})
  endforeach()

  if(_ARG_TEST_LIST)
    set(${_ARG_TEST_LIST} "${${_ARG_TEST_LIST}}"
        PARENT_SCOPE
    )
  endif()

  dart_format_add(${_ARG_SOURCES})

endfunction()

#===============================================================================
# Component Management
#===============================================================================
# Note: These functions were originally in Components.cmake and merged into
# dart_defs.cmake for better organization and consistency.

#-------------------------------------------------------------------------------
# Initialize component helpers
# Initializes global properties for component bookkeeping. `add_component()` will
# auto-initialize if this has not been called, but calling it once up front
# keeps the intent explicit.
# Usage:
#   initialize_component_helpers(package_name)
#-------------------------------------------------------------------------------
function(initialize_component_helpers package_name)
  define_property(
    GLOBAL
    PROPERTY "${package_name}_INCLUDE_DIRS"
    BRIEF_DOCS "Global include directories used by all components."
    FULL_DOCS "Global include directories used by all components."
  )
  define_property(
    GLOBAL
    PROPERTY "${package_name}_COMPONENTS"
    BRIEF_DOCS "List all known ${package_name} components."
    FULL_DOCS "List all known ${package_name} components."
  )
endfunction()

#-------------------------------------------------------------------------------
# Check if a target is a component
# Usage:
#   is_component(output_variable package_name component)
#-------------------------------------------------------------------------------
function(is_component output_variable package_name component)
  set(component_prefix "${package_name}_component_")
  set(target "${component_prefix}${component}")

  if(TARGET "${target}")
    get_property(
      output
      TARGET "${target}"
      PROPERTY "${component_prefix}COMPONENT"
    )
    set("${output_variable}" "${output}" PARENT_SCOPE)
  else()
    set("${output_variable}" FALSE PARENT_SCOPE)
  endif()
endfunction()

#-------------------------------------------------------------------------------
# Create a custom target for a component
# The target is named <package_name>_component_<component>. Afterwards, the
# dependency targets of the component can be added to the target using
# add_component_targets().
#
# Warning: At least one dependency target must be added to the component target,
# otherwise this function returns error.
# Usage:
#   add_component(package_name component)
#-------------------------------------------------------------------------------
function(add_component package_name component)
  get_property(_components_defined GLOBAL PROPERTY "${package_name}_COMPONENTS" DEFINED)
  if(NOT _components_defined)
    initialize_component_helpers(${package_name})
  endif()

  set(component_prefix "${package_name}_component_")
  set(target "${component_prefix}${component}")
  add_custom_target("${target}")
  # TODO(JS): It would be nice if we could check if ${target} has at least one
  # dependency target.

  set_property(
    TARGET "${target}"
    PROPERTY "${component_prefix}COMPONENT" TRUE
  )
  set_property(
    TARGET "${target}"
    PROPERTY "${component_prefix}DEPENDENCIES"
  )
  set_property(
    TARGET "${target}"
    PROPERTY "${component_prefix}INCLUDE_DIRS"
  )
  set_property(
    TARGET "${target}"
    PROPERTY "${component_prefix}LIBRARIES"
  )
  set_property(
    GLOBAL
    APPEND
    PROPERTY "${package_name}_COMPONENTS" "${component}"
  )
endfunction()

#-------------------------------------------------------------------------------
# Add include directories to a component
# Usage:
#   add_component_include_directories(package_name component dir1 [dir2...])
#-------------------------------------------------------------------------------
function(add_component_include_directories package_name component)
  set(component_prefix "${package_name}_component_")
  set(target "${component_prefix}${component}")
  set_property(
    TARGET "${target}"
    APPEND
    PROPERTY "${component_prefix}INCLUDE_DIRS" ${ARGN}
  )
endfunction()

#-------------------------------------------------------------------------------
# Add internal component dependencies
# Usage:
#   add_component_dependencies(package_name component dep1 [dep2...])
#-------------------------------------------------------------------------------
function(add_component_dependencies package_name component)
  set(component_prefix "${package_name}_component_")
  set(dependency_components ${ARGN})

  is_component(is_valid "${package_name}" "${component}")
  if(NOT is_valid)
    message(
      FATAL_ERROR
      "Target '${component}' is not a component of ${package_name}."
    )
  endif()

  set(target "${component_prefix}${component}")

  foreach(dependency_component IN LISTS dependency_components)
    is_component(is_valid "${package_name}" "${dependency_component}")
    if(NOT is_valid)
      message(
        FATAL_ERROR
        "Target '${dependency_component}' is not a component of ${package_name}."
      )
    endif()

    set(dependency_target "${component_prefix}${dependency_component}")
    add_dependencies("${target}" "${dependency_target}")
  endforeach()

  set_property(
    TARGET "${target}"
    APPEND
    PROPERTY "${component_prefix}DEPENDENCIES" ${dependency_components}
  )
endfunction()

#-------------------------------------------------------------------------------
# Add external package dependencies to a component
# Usage:
#   add_component_dependency_packages(package_name component pkg1 [pkg2...])
#-------------------------------------------------------------------------------
function(add_component_dependency_packages package_name component)
  set(component_prefix "${package_name}_component_")
  set(dependency_package ${ARGN})

  is_component(is_valid "${package_name}" "${component}")
  if(NOT is_valid)
    message(
      FATAL_ERROR
      "Target '${component}' is not a component of ${package_name}."
    )
  endif()

  set(target "${component_prefix}${component}")

  set_property(
    TARGET "${target}"
    APPEND
    PROPERTY "${component_prefix}dependency_package" ${dependency_package}
  )
endfunction()

#-------------------------------------------------------------------------------
# Add library targets to a component
# Usage:
#   add_component_targets(package_name component target1 [target2...])
#-------------------------------------------------------------------------------
function(add_component_targets package_name component)
  set(component_prefix "${package_name}_component_")
  set(dependency_targets ${ARGN})

  is_component(is_valid "${package_name}" "${component}")
  if(NOT is_valid)
    message(
      FATAL_ERROR
      "Target '${component}' is not a component of ${package_name}."
    )
  endif()

  set(target "${component_prefix}${component}")
  add_dependencies("${target}" ${ARGN})

  foreach(dependency_target IN LISTS dependency_targets)
    if(NOT TARGET "${dependency_target}")
      message(FATAL_ERROR "Target '${dependency_target}' does not exist.")
    endif()

    get_property(
      dependency_type
      TARGET "${dependency_target}"
      PROPERTY TYPE
    )
    if(NOT ("${dependency_type}" STREQUAL STATIC_LIBRARY
          OR "${dependency_type}" STREQUAL SHARED_LIBRARY))
      message(
        FATAL_ERROR
        "Target '${dependency_target}' has unsupported type"
        " '${dependency_type}'. Only 'STATIC_LIBRARY' and 'SHARED_LIBRARY'"
        " are supported."
      )
    endif()

    install(
      TARGETS "${dependency_target}"
      EXPORT "${target}"
      ARCHIVE DESTINATION "${LIBRARY_INSTALL_DIR}"
      LIBRARY DESTINATION "${LIBRARY_INSTALL_DIR}"
    )
  endforeach()

  if(dependency_targets)
    install(
      EXPORT "${target}"
      FILE "${package_name}_${component}Targets.cmake"
      DESTINATION "${CONFIG_INSTALL_DIR}"
    )
  endif()

  set_property(
    TARGET "${target}"
    APPEND
    PROPERTY "${component_prefix}LIBRARIES" ${dependency_targets}
  )
endfunction()

#-------------------------------------------------------------------------------
# Add a component with all properties in one call (modern, consolidated API)
# Usage:
#   dart_add_component(
#     NAME <component_name>
#     [PACKAGE <package_name>]  # Default: PROJECT_NAME
#     [TARGETS target1 target2...]
#     [DEPENDENCIES dep1 dep2...]  # Internal component dependencies
#     [DEPENDENCY_PACKAGES pkg1 pkg2...]  # External package dependencies
#     [INCLUDE_DIRS dir1 dir2...]
#     [COMPONENT_HEADERS header1.hpp header2.hpp...]  # full paths; names auto-derived
#     [COMPONENT_HEADER_EXTRAS extra1.hpp extra2.hpp...]  # names or relative paths to include in All.hpp
#     [COMPONENT_HEADER_SOURCES source_header1.hpp ...]  # for PascalCase compat generation
#     [COMPONENT_TARGET_DIR dir/]  # e.g., "dart/utils/"
#     [COMPONENT_OUTPUT_DIR dir]  # defaults to CMAKE_CURRENT_BINARY_DIR
#   )
#
# This function consolidates add_component(), add_component_targets(),
# add_component_dependencies(), and add_component_dependency_packages()
# into a single, modern API call.
#
# Example:
#   dart_add_component(
#     NAME utils
#     TARGETS dart-utils
#     DEPENDENCIES dart
#     DEPENDENCY_PACKAGES Eigen3 fmt
#   )
#-------------------------------------------------------------------------------
function(dart_add_component)
  set(prefix _ARG)
  set(options
    NO_REGISTER
  )
  set(oneValueArgs
    NAME
    PACKAGE
    COMPONENT_TARGET_DIR
    COMPONENT_OUTPUT_DIR
  )
  set(multiValueArgs
    TARGETS
    DEPENDENCIES
    DEPENDENCY_PACKAGES
    INCLUDE_DIRS
    COMPONENT_HEADERS
    COMPONENT_HEADER_EXTRAS
    COMPONENT_HEADER_SOURCES
  )
  cmake_parse_arguments(
    "${prefix}" "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN}
  )

  # Validate required arguments
  if(NOT _ARG_NAME)
    message(FATAL_ERROR "dart_add_component: NAME is required")
  endif()

  # Default package to PROJECT_NAME
  if(NOT _ARG_PACKAGE)
    set(_ARG_PACKAGE ${PROJECT_NAME})
  endif()

  # Step 1: Create the component (optional)
  set(_dart_register_component TRUE)
  if(_ARG_NO_REGISTER)
    set(_dart_register_component FALSE)
  endif()

  if(_dart_register_component)
    add_component(${_ARG_PACKAGE} ${_ARG_NAME})

    # Step 2: Add targets if specified
    if(_ARG_TARGETS)
      add_component_targets(${_ARG_PACKAGE} ${_ARG_NAME} ${_ARG_TARGETS})
    endif()

    # Step 3: Add internal dependencies if specified
    if(_ARG_DEPENDENCIES)
      add_component_dependencies(${_ARG_PACKAGE} ${_ARG_NAME} ${_ARG_DEPENDENCIES})
    endif()

    # Step 4: Add external package dependencies if specified
    if(_ARG_DEPENDENCY_PACKAGES)
      add_component_dependency_packages(${_ARG_PACKAGE} ${_ARG_NAME} ${_ARG_DEPENDENCY_PACKAGES})
    endif()

    # Step 5: Add include directories if specified
    if(_ARG_INCLUDE_DIRS)
      add_component_include_directories(${_ARG_PACKAGE} ${_ARG_NAME} ${_ARG_INCLUDE_DIRS})
    endif()
  endif()

  # Step 6: Generate component meta headers if requested
  if(_ARG_COMPONENT_HEADERS OR _ARG_COMPONENT_HEADER_EXTRAS
      OR _ARG_COMPONENT_HEADER_SOURCES OR _ARG_COMPONENT_TARGET_DIR
      OR _ARG_COMPONENT_OUTPUT_DIR)
    if(NOT _ARG_COMPONENT_TARGET_DIR)
      message(FATAL_ERROR "dart_add_component: COMPONENT_TARGET_DIR is required when generating component headers")
    endif()
    if(NOT _ARG_COMPONENT_OUTPUT_DIR)
      set(_ARG_COMPONENT_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}")
    endif()
    # Reset compat header list to avoid leaking from previous invocations
    set(DART_GENERATED_COMPAT_HEADERS "")

    set(_dart_component_header_names)
    foreach(_dart_header IN LISTS _ARG_COMPONENT_HEADERS)
      get_filename_component(_dart_header_name "${_dart_header}" NAME)
      list(APPEND _dart_component_header_names "${_dart_header_name}")
    endforeach()
    list(APPEND _dart_component_header_names ${_ARG_COMPONENT_HEADER_EXTRAS})

    if(_dart_component_header_names)
      dart_generate_component_headers(
        COMPONENT_NAME ${_ARG_NAME}
        TARGET_DIR "${_ARG_COMPONENT_TARGET_DIR}"
        OUTPUT_DIR "${_ARG_COMPONENT_OUTPUT_DIR}"
        HEADERS ${_dart_component_header_names}
        SOURCE_HEADERS ${_ARG_COMPONENT_HEADER_SOURCES}
      )

      # Install generated meta headers and compatibility headers when possible
      set(_dart_component_target_dir "${_ARG_COMPONENT_TARGET_DIR}")
      string(REGEX REPLACE "/$" "" _dart_component_target_dir "${_dart_component_target_dir}")
      set(_dart_component_install_dir "${CMAKE_INSTALL_INCLUDEDIR}/${_dart_component_target_dir}")

      set(_dart_component_all_header "${_ARG_COMPONENT_OUTPUT_DIR}/All.hpp")
      set(_dart_component_primary_header "${_ARG_COMPONENT_OUTPUT_DIR}/${_ARG_NAME}.hpp")

      if(EXISTS "${_dart_component_all_header}" OR EXISTS "${_dart_component_primary_header}")
        install(
          FILES ${_dart_component_all_header} ${_dart_component_primary_header}
          DESTINATION "${_dart_component_install_dir}"
          COMPONENT headers
        )
      endif()

      if(DART_GENERATED_COMPAT_HEADERS)
        dart_install_compat_headers(
          COMPAT_HEADERS ${DART_GENERATED_COMPAT_HEADERS}
          DESTINATION_PREFIX "${_dart_component_install_dir}"
        )
      endif()
    endif()
  endif()

  if(DART_VERBOSE AND _dart_register_component)
    message(STATUS "Component '${_ARG_NAME}' configured with ${_ARG_PACKAGE}")
  endif()
endfunction()

#-------------------------------------------------------------------------------
# Install component exports
# Usage:
#   install_component_exports(package_name)
#-------------------------------------------------------------------------------
function(install_component_exports package_name)
  set(component_prefix "${package_name}_component_")
  get_property(components GLOBAL PROPERTY "${package_name}_COMPONENTS")

  set(output_prefix "${CMAKE_CURRENT_BINARY_DIR}/${CONFIG_INSTALL_DIR}")

  foreach(component IN LISTS components)
    set(target "${component_prefix}${component}")

    # TODO: Replace this manual generation with a configure_file.
    set(output_path
      "${output_prefix}/${package_name}_${component}Component.cmake")

    get_property(
      internal_dependencies
      TARGET "${target}"
      PROPERTY "${component_prefix}DEPENDENCIES"
    )

    get_property(
      libraries
      TARGET "${target}"
      PROPERTY "${component_prefix}LIBRARIES"
    )

    get_property(
      dependency_package
      TARGET "${target}"
      PROPERTY "${component_prefix}dependency_package"
    )
    set(external_dependencies)
    foreach(dependent_package IN LISTS dependency_package)
      set(find_pkg_name "Find${dependent_package}.cmake")
      set(find_pkg_path "")
      set(dart_find_pkg_name "DARTFind${dependent_package}.cmake")
      set(dart_find_pkg_path "")
      foreach(module_path IN LISTS CMAKE_MODULE_PATH)
        set(find_pkg_path_candidate "${module_path}/${find_pkg_name}")
        set(dart_find_pkg_path_candidate "${module_path}/${dart_find_pkg_name}")
        if("${find_pkg_path}" STREQUAL "" AND EXISTS "${find_pkg_path_candidate}")
          set(find_pkg_path ${find_pkg_path_candidate})
        endif()
        if("${dart_find_pkg_path}" STREQUAL "" AND EXISTS "${dart_find_pkg_path_candidate}")
          set(dart_find_pkg_path ${dart_find_pkg_path_candidate})
        endif()
      endforeach()
      if(NOT "${find_pkg_path}" STREQUAL "")
        install(FILES "${find_pkg_path}" DESTINATION "${CONFIG_INSTALL_DIR}")
      endif()
      if("${dart_find_pkg_path}" STREQUAL "")
        message(FATAL_ERROR "Failed to find '${dart_find_pkg_path}'.")
      endif()
      list(APPEND external_dependencies ${dependent_package})
      install(FILES "${dart_find_pkg_path}" DESTINATION "${CONFIG_INSTALL_DIR}")
    endforeach()

    configure_file(
      "${DART_SOURCE_DIR}/cmake/dart_Component.cmake.in"
      "${output_path}"
      @ONLY)

    install(FILES "${output_path}" DESTINATION "${CONFIG_INSTALL_DIR}")
  endforeach()
endfunction()

#===============================================================================
# Header File Generation
#===============================================================================

#-------------------------------------------------------------------------------
# Convert snake_case to PascalCase
# Usage:
#   dart_snake_to_pascal(output_var "snake_case_string")
# Example:
#   dart_snake_to_pascal(result "my_class_name")  # result = "MyClassName"
#-------------------------------------------------------------------------------
function(dart_snake_to_pascal output_var input_string)
  # Remove file extension if present
  get_filename_component(name_without_ext "${input_string}" NAME_WE)
  get_filename_component(ext "${input_string}" EXT)

  # Split by underscores and capitalize each word
  string(REPLACE "_" ";" words "${name_without_ext}")
  set(pascal_result "")
  foreach(word IN LISTS words)
    # Capitalize first letter of each word
    string(SUBSTRING "${word}" 0 1 first_letter)
    string(TOUPPER "${first_letter}" first_letter_upper)
    string(LENGTH "${word}" word_length)
    if(word_length GREATER 1)
      string(SUBSTRING "${word}" 1 -1 rest_of_word)
      set(capitalized_word "${first_letter_upper}${rest_of_word}")
    else()
      set(capitalized_word "${first_letter_upper}")
    endif()
    string(APPEND pascal_result "${capitalized_word}")
  endforeach()

  # Add extension back
  set(${output_var} "${pascal_result}${ext}" PARENT_SCOPE)
endfunction()

#-------------------------------------------------------------------------------
# Check if a filename is in snake_case format
# Usage:
#   dart_is_snake_case(output_var "filename.hpp")
# Returns TRUE if filename contains underscores and no uppercase letters
#-------------------------------------------------------------------------------
function(dart_is_snake_case output_var filename)
  get_filename_component(name_without_ext "${filename}" NAME_WE)

  # Check if contains underscores
  string(FIND "${name_without_ext}" "_" underscore_pos)

  # Check if contains uppercase letters (excluding extension)
  string(REGEX MATCH "[A-Z]" has_uppercase "${name_without_ext}")

  if(NOT underscore_pos EQUAL -1 AND NOT has_uppercase)
    set(${output_var} TRUE PARENT_SCOPE)
  else()
    set(${output_var} FALSE PARENT_SCOPE)
  endif()
endfunction()

#-------------------------------------------------------------------------------
# Generate backward-compatible PascalCase headers for snake_case files
# Usage:
#   dart_generate_case_compat_headers(
#     HEADERS header1.hpp header2.hpp...
#     OUTPUT_DIR output_directory
#     COMPONENT_PATH dart/common
#     [RELATIVE_TO source_directory]
#     [SUPPRESS_WARNING]
#   )
#
# For each snake_case header, generates a PascalCase version that includes
# the snake_case file with a deprecation warning.
#
# Example:
#   my_class.hpp -> MyClass.hpp (generated)
#   MyClass.hpp includes "dart/common/my_class.hpp"
#-------------------------------------------------------------------------------
function(dart_generate_case_compat_headers)
  set(options SUPPRESS_WARNING)
  set(oneValueArgs OUTPUT_DIR RELATIVE_TO COMPONENT_PATH)
  set(multiValueArgs HEADERS)
  cmake_parse_arguments(DGCCH "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  if(NOT DGCCH_OUTPUT_DIR)
    message(FATAL_ERROR "OUTPUT_DIR is required for dart_generate_case_compat_headers")
  endif()

  if(NOT DGCCH_RELATIVE_TO)
    set(DGCCH_RELATIVE_TO "${CMAKE_CURRENT_SOURCE_DIR}")
  endif()

  set(generated_headers)

  foreach(header IN LISTS DGCCH_HEADERS)
    get_filename_component(header_name "${header}" NAME)

    # Check if this is a snake_case header
    dart_is_snake_case(is_snake "${header_name}")

    if(is_snake)
      # Convert to PascalCase
      dart_snake_to_pascal(pascal_name "${header_name}")

      # Get relative path from source to maintain directory structure
      file(RELATIVE_PATH rel_path "${DGCCH_RELATIVE_TO}" "${header}")
      get_filename_component(rel_dir "${rel_path}" DIRECTORY)

      # Output path for generated PascalCase header
      if(rel_dir)
        set(output_path "${DGCCH_OUTPUT_DIR}/${rel_dir}/${pascal_name}")
      else()
        set(output_path "${DGCCH_OUTPUT_DIR}/${pascal_name}")
      endif()

      # Create directory if needed
      get_filename_component(output_dir "${output_path}" DIRECTORY)
      file(MAKE_DIRECTORY "${output_dir}")

      # Generate the compatibility header
      file(WRITE "${output_path}" "// Automatically generated backward compatibility header\n")
      file(APPEND "${output_path}" "// This file is DEPRECATED and will be removed in a future release\n")
      file(APPEND "${output_path}" "//\n")
      file(APPEND "${output_path}" "// DEPRECATED: ${pascal_name} is deprecated.\n")
      file(APPEND "${output_path}" "// Please use ${header_name} instead.\n")
      file(APPEND "${output_path}" "//\n")
      file(APPEND "${output_path}" "// This header will be removed in DART 8.0.\n\n")

      if(NOT DGCCH_SUPPRESS_WARNING)
        file(APPEND "${output_path}" "#ifndef DART_SUPPRESS_DEPRECATED_HEADER_WARNING\n")
        file(APPEND "${output_path}" "#if defined(_MSC_VER)\n")
        file(APPEND "${output_path}" "#  pragma message(\"Warning: ${pascal_name} is deprecated. Use ${header_name} instead.\")\n")
        file(APPEND "${output_path}" "#elif defined(__GNUC__) || defined(__clang__)\n")
        file(APPEND "${output_path}" "#  warning \"${pascal_name} is deprecated. Use ${header_name} instead.\"\n")
        file(APPEND "${output_path}" "#endif\n")
        file(APPEND "${output_path}" "#endif // DART_SUPPRESS_DEPRECATED_HEADER_WARNING\n\n")
      endif()

      # Include the actual snake_case header using the full component path
      # Use COMPONENT_PATH if provided, otherwise construct from rel_path
      if(DGCCH_COMPONENT_PATH)
        if(rel_dir)
          set(include_path "${DGCCH_COMPONENT_PATH}/${rel_dir}/${header_name}")
        else()
          set(include_path "${DGCCH_COMPONENT_PATH}/${header_name}")
        endif()
      else()
        # Fallback to rel_path if COMPONENT_PATH not provided
        set(include_path "${rel_path}")
      endif()
      file(APPEND "${output_path}" "#include \"${include_path}\"\n")

      list(APPEND generated_headers "${output_path}")

      if(DART_VERBOSE)
        message(STATUS "Generated compatibility header: ${pascal_name} -> ${header_name}")
      endif()
    endif()
  endforeach()

  # Return generated headers to parent scope
  set(DART_GENERATED_COMPAT_HEADERS "${generated_headers}" PARENT_SCOPE)
endfunction()

#-------------------------------------------------------------------------------
# Generate and install component headers without registering a component
# Usage:
#   dart_generate_component_headers_only(
#     COMPONENT_NAME <name>
#     TARGET_DIR <dir/>
#     OUTPUT_DIR <dir>
#     HEADERS <names>
#     SOURCE_HEADERS <paths...>
#     [HEADER_EXTRAS <names...>]
#   )
#-------------------------------------------------------------------------------
function(dart_generate_component_headers_only)
  set(options)
  set(oneValueArgs COMPONENT_NAME TARGET_DIR OUTPUT_DIR)
  set(multiValueArgs HEADERS SOURCE_HEADERS HEADER_EXTRAS)
  cmake_parse_arguments(DGCHO "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  if(NOT DGCHO_COMPONENT_NAME OR NOT DGCHO_TARGET_DIR OR NOT DGCHO_OUTPUT_DIR)
    message(FATAL_ERROR "dart_generate_component_headers_only requires COMPONENT_NAME, TARGET_DIR, OUTPUT_DIR")
  endif()

  set(DART_GENERATED_COMPAT_HEADERS "")
  dart_generate_component_headers(
    COMPONENT_NAME ${DGCHO_COMPONENT_NAME}
    TARGET_DIR "${DGCHO_TARGET_DIR}"
    OUTPUT_DIR "${DGCHO_OUTPUT_DIR}"
    HEADERS ${DGCHO_HEADERS} ${DGCHO_HEADER_EXTRAS}
    SOURCE_HEADERS ${DGCHO_SOURCE_HEADERS}
  )

  set(_dgcho_target_dir "${DGCHO_TARGET_DIR}")
  string(REGEX REPLACE "/$" "" _dgcho_target_dir "${_dgcho_target_dir}")
  set(_dgcho_install_dir "${CMAKE_INSTALL_INCLUDEDIR}/${_dgcho_target_dir}")

  set(_dgcho_all_header "${DGCHO_OUTPUT_DIR}/All.hpp")
  set(_dgcho_primary_header "${DGCHO_OUTPUT_DIR}/${DGCHO_COMPONENT_NAME}.hpp")
  if(EXISTS "${_dgcho_all_header}" OR EXISTS "${_dgcho_primary_header}")
    install(
      FILES ${_dgcho_all_header} ${_dgcho_primary_header}
      DESTINATION "${_dgcho_install_dir}"
      COMPONENT headers
    )
  endif()

  if(DART_GENERATED_COMPAT_HEADERS)
    dart_install_compat_headers(
      COMPAT_HEADERS ${DART_GENERATED_COMPAT_HEADERS}
      DESTINATION_PREFIX "${_dgcho_install_dir}"
    )
  endif()
endfunction()

#-------------------------------------------------------------------------------
# Generate and install namespace headers for non-component subsystems
# Usage is internal to dart_add_subsystem_library below.
#-------------------------------------------------------------------------------
function(dart_add_subsystem_headers)
  set(options)
  set(oneValueArgs NAME TARGET_DIR OUTPUT_DIR)
  set(multiValueArgs HEADERS SOURCE_HEADERS HEADER_EXTRAS)
  cmake_parse_arguments(DASH "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  if(NOT DASH_NAME OR NOT DASH_TARGET_DIR)
    message(FATAL_ERROR "dart_add_subsystem_headers requires NAME and TARGET_DIR")
  endif()
  if(NOT DASH_OUTPUT_DIR)
    set(DASH_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}")
  endif()

  dart_generate_component_headers_only(
    COMPONENT_NAME ${DASH_NAME}
    TARGET_DIR "${DASH_TARGET_DIR}"
    OUTPUT_DIR "${DASH_OUTPUT_DIR}"
    HEADERS ${DASH_HEADERS}
    HEADER_EXTRAS ${DASH_HEADER_EXTRAS}
    SOURCE_HEADERS ${DASH_SOURCE_HEADERS}
  )
endfunction()

#-------------------------------------------------------------------------------
# Convenience helper for subsystems that are part of the main library (not standalone components)
# Usage:
#   dart_add_subsystem_library(
#     NAME <name>
#     TARGET_DIR <dir/>
#     [OUTPUT_DIR <dir>]
#     [HEADER_GLOBS <patterns...>]           # default "*.hpp"
#     [DETAIL_HEADER_GLOBS <patterns...>]    # default "detail/*.hpp"
#     [EXTRA_HEADER_GLOBS <patterns...>]     # optional additional headers
#     [SOURCE_GLOBS <patterns...>]           # default "*.cpp"
#     [DETAIL_SOURCE_GLOBS <patterns...>]    # default "detail/*.cpp"
#     [HEADER_EXTRAS <names...>]             # extra entries to include in All.hpp
#     [HEADER_EXCLUDE <names...>]            # file names to exclude from All.hpp
#     [SUBDIRECTORIES <dirs...>]             # add_subdirectory for each
#   )
#-------------------------------------------------------------------------------
function(dart_add_subsystem_library)
  set(options)
  set(oneValueArgs NAME TARGET_DIR OUTPUT_DIR)
  set(multiValueArgs
    HEADER_GLOBS
    DETAIL_HEADER_GLOBS
    EXTRA_HEADER_GLOBS
    SOURCE_GLOBS
    DETAIL_SOURCE_GLOBS
    HEADER_EXTRAS
    HEADER_EXCLUDE
    SUBDIRECTORIES
    SUB_DIRS
  )
  cmake_parse_arguments(DASL "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  if(NOT DASL_NAME OR NOT DASL_TARGET_DIR)
    message(FATAL_ERROR "dart_add_subsystem_library requires NAME and TARGET_DIR")
  endif()
  if(NOT DASL_OUTPUT_DIR)
    set(DASL_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}")
  endif()

  if(NOT DASL_HEADER_GLOBS)
    set(DASL_HEADER_GLOBS "*.hpp")
  endif()
  if(NOT DASL_DETAIL_HEADER_GLOBS)
    set(DASL_DETAIL_HEADER_GLOBS "detail/*.hpp")
  endif()
  if(NOT DASL_SOURCE_GLOBS)
    set(DASL_SOURCE_GLOBS "*.cpp")
  endif()
  if(NOT DASL_DETAIL_SOURCE_GLOBS)
    set(DASL_DETAIL_SOURCE_GLOBS "detail/*.cpp")
  endif()

  set(hdrs)
  foreach(pattern IN LISTS DASL_HEADER_GLOBS)
    file(GLOB _tmp "${pattern}")
    list(APPEND hdrs ${_tmp})
  endforeach()

  set(detail_hdrs)
  foreach(pattern IN LISTS DASL_DETAIL_HEADER_GLOBS)
    file(GLOB _tmp "${pattern}")
    list(APPEND detail_hdrs ${_tmp})
  endforeach()

  set(extra_hdrs)
  foreach(pattern IN LISTS DASL_EXTRA_HEADER_GLOBS)
    file(GLOB _tmp "${pattern}")
    list(APPEND extra_hdrs ${_tmp})
  endforeach()

  set(srcs)
  foreach(pattern IN LISTS DASL_SOURCE_GLOBS)
    file(GLOB _tmp "${pattern}")
    list(APPEND srcs ${_tmp})
  endforeach()

  set(detail_srcs)
  foreach(pattern IN LISTS DASL_DETAIL_SOURCE_GLOBS)
    file(GLOB _tmp "${pattern}")
    list(APPEND detail_srcs ${_tmp})
  endforeach()

  dart_add_core_headers(${hdrs} ${detail_hdrs})
  dart_add_core_sources(${srcs} ${detail_srcs})

  dart_get_filename_components(header_names "${DASL_NAME} headers" ${hdrs})
  if(DASL_HEADER_EXCLUDE)
    foreach(exclude_name IN LISTS DASL_HEADER_EXCLUDE)
      list(REMOVE_ITEM header_names ${exclude_name})
    endforeach()
  endif()
  list(APPEND header_names ${DASL_HEADER_EXTRAS})

  dart_add_subsystem_headers(
    NAME ${DASL_NAME}
    TARGET_DIR "${DASL_TARGET_DIR}"
    OUTPUT_DIR "${DASL_OUTPUT_DIR}"
    HEADERS ${header_names}
    SOURCE_HEADERS ${hdrs} ${detail_hdrs} ${extra_hdrs}
  )

  set(DASL_ALL_SUBDIRS ${DASL_SUBDIRECTORIES} ${DASL_SUB_DIRS})
  foreach(subdir IN LISTS DASL_ALL_SUBDIRS)
    add_subdirectory(${subdir})
  endforeach()
endfunction()

#-------------------------------------------------------------------------------
# Install generated compatibility headers while preserving subdirectory structure
# Usage:
#   dart_install_compat_headers(
#     COMPAT_HEADERS ${DART_GENERATED_COMPAT_HEADERS}
#     DESTINATION_PREFIX include/dart/common
#   )
#
# This function installs generated PascalCase compatibility headers to their
# appropriate subdirectories, preserving the directory structure from the build.
# For example, if a compat header is at build/dart/common/detail/MyHeader.hpp,
# it will be installed to include/dart/common/detail/MyHeader.hpp.
#-------------------------------------------------------------------------------
function(dart_install_compat_headers)
  set(options)
  set(oneValueArgs DESTINATION_PREFIX)
  set(multiValueArgs COMPAT_HEADERS)
  cmake_parse_arguments(DICH "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  if(NOT DICH_DESTINATION_PREFIX)
    message(FATAL_ERROR "DESTINATION_PREFIX is required for dart_install_compat_headers")
  endif()

  foreach(compat_header IN LISTS DICH_COMPAT_HEADERS)
    # Get path relative to the build directory to preserve subdirectory structure
    file(RELATIVE_PATH rel_path "${CMAKE_CURRENT_BINARY_DIR}" "${compat_header}")
    get_filename_component(rel_dir "${rel_path}" DIRECTORY)

    # Install to the appropriate subdirectory
    if(rel_dir)
      install(
        FILES "${compat_header}"
        DESTINATION "${DICH_DESTINATION_PREFIX}/${rel_dir}"
        COMPONENT headers
      )
    else()
      install(
        FILES "${compat_header}"
        DESTINATION "${DICH_DESTINATION_PREFIX}"
        COMPONENT headers
      )
    endif()
  endforeach()
endfunction()

#-------------------------------------------------------------------------------
# Append items to a cached list
# Usage:
#   dart_append_to_cached_string(_string _cacheDesc [items...])
#
# Note: Used internally by dart_get_filename_components
#-------------------------------------------------------------------------------
macro(dart_append_to_cached_string _string _cacheDesc)
  foreach(newItem ${ARGN})
    set(${_string} "${${_string}}${newItem}")
  endforeach()
endmacro()

#-------------------------------------------------------------------------------
# Get list of file names from list of full paths
# Usage:
#   dart_get_filename_components(_var _cacheDesc [items...])
#-------------------------------------------------------------------------------
macro(dart_get_filename_components _var _cacheDesc)
  set(${_var} "")
  foreach(header ${ARGN})
    get_filename_component(header ${header} NAME)
    dart_append_to_cached_string(
      ${_var}
      ${_cacheDesc}"_HEADER_NAMES"
      "${header}\;"
    )
  endforeach()
endmacro()

#-------------------------------------------------------------------------------
# Generate component headers with backward compatibility
# Usage:
#   dart_generate_component_headers(
#     COMPONENT_NAME component_name
#     TARGET_DIR target_dir
#     OUTPUT_DIR output_dir
#     HEADERS [headers...]
#     [SOURCE_HEADERS full_path_headers...]
#   )
#
# This macro generates:
#   1. All.hpp - The new meta header that includes all component headers
#   2. <component_name>.hpp - Deprecated header that includes All.hpp with warning
#   3. PascalCase compatibility headers for snake_case files (automatic)
#
# If SOURCE_HEADERS is provided, generates PascalCase compatibility headers
# for all snake_case files in SOURCE_HEADERS.
#-------------------------------------------------------------------------------
macro(dart_generate_component_headers)
  set(options)
  set(oneValueArgs COMPONENT_NAME TARGET_DIR OUTPUT_DIR)
  set(multiValueArgs HEADERS SOURCE_HEADERS)
  cmake_parse_arguments(DART_GCH "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  # Generate All.hpp (new header)
  set(all_hpp_path "${DART_GCH_OUTPUT_DIR}/All.hpp")
  file(WRITE ${all_hpp_path} "// Automatically generated file by cmake\n\n")
  foreach(header IN LISTS DART_GCH_HEADERS)
    file(APPEND ${all_hpp_path} "#include \"${DART_GCH_TARGET_DIR}${header}\"\n")
  endforeach()

  # Generate <component_name>.hpp (deprecated header with backward compatibility)
  set(deprecated_hpp_path "${DART_GCH_OUTPUT_DIR}/${DART_GCH_COMPONENT_NAME}.hpp")
  file(WRITE ${deprecated_hpp_path} "// Automatically generated file by cmake\n\n")
  file(APPEND ${deprecated_hpp_path} "// DEPRECATED: This header is deprecated and will be removed in the next major release.\n")
  file(APPEND ${deprecated_hpp_path} "// Please use <${DART_GCH_TARGET_DIR}All.hpp> instead of <${DART_GCH_TARGET_DIR}${DART_GCH_COMPONENT_NAME}.hpp>\n\n")
  file(APPEND ${deprecated_hpp_path} "#ifndef DART_SUPPRESS_DEPRECATED_HEADER_WARNING\n")
  file(APPEND ${deprecated_hpp_path} "#if defined(_MSC_VER)\n")
  file(APPEND ${deprecated_hpp_path} "#  pragma message(\"Warning: ${DART_GCH_TARGET_DIR}${DART_GCH_COMPONENT_NAME}.hpp is deprecated. Use ${DART_GCH_TARGET_DIR}All.hpp instead.\")\n")
  file(APPEND ${deprecated_hpp_path} "#elif defined(__GNUC__) || defined(__clang__)\n")
  file(APPEND ${deprecated_hpp_path} "#  warning \"${DART_GCH_TARGET_DIR}${DART_GCH_COMPONENT_NAME}.hpp is deprecated. Use ${DART_GCH_TARGET_DIR}All.hpp instead.\"\n")
  file(APPEND ${deprecated_hpp_path} "#endif\n")
  file(APPEND ${deprecated_hpp_path} "#endif // DART_SUPPRESS_DEPRECATED_HEADER_WARNING\n\n")
  file(APPEND ${deprecated_hpp_path} "#include \"${DART_GCH_TARGET_DIR}All.hpp\"\n")

  # Automatically generate PascalCase compatibility headers if SOURCE_HEADERS provided
  if(DART_GCH_SOURCE_HEADERS)
    dart_generate_case_compat_headers(
      HEADERS ${DART_GCH_SOURCE_HEADERS}
      OUTPUT_DIR "${DART_GCH_OUTPUT_DIR}"
      COMPONENT_PATH "${DART_GCH_TARGET_DIR}"
      RELATIVE_TO "${CMAKE_CURRENT_SOURCE_DIR}"
    )
  endif()
endmacro()

#===============================================================================
# Package Management and Dependency Finding
#===============================================================================

#-------------------------------------------------------------------------------
# Find a DART package using DARTFind<name>.cmake (legacy compatibility)
# OR find and track dependencies with enhanced features (new syntax)
#
# Legacy usage (backward compatible):
#   dart_find_package(package_name)
#   - Simply includes DARTFind${package_name}.cmake
#
# Enhanced usage (new syntax):
#   dart_find_package(
#     NAME <name>
#     PACKAGE <package_name>
#     [REQUIRED]
#     [QUIET]
#     [VERSION <version>]
#     [COMPONENTS <components...>]
#     [SET_VAR <variable_name>]
#   )
#   - Wraps find_package() and tracks found/missing dependencies
#   - Compatible with DART_VERBOSE for summary reporting
#
#-------------------------------------------------------------------------------
macro(dart_find_package)
  # Check if called with legacy syntax (single argument, no keywords)
  list(LENGTH ARGN arg_count)
  if(arg_count EQUAL 0)
    include(DARTFind${ARGV0})
  else()
    # Check if first argument is a keyword
    list(GET ARGN 0 first_arg)
    if(NOT first_arg MATCHES "^(NAME|PACKAGE|REQUIRED|QUIET|VERSION|COMPONENTS|SET_VAR)$")
      include(DARTFind${ARGV0})
    else()
      # New enhanced syntax - parse named arguments
      cmake_parse_arguments(
        ARG
        "REQUIRED;QUIET"
        "NAME;PACKAGE;VERSION;SET_VAR"
        "COMPONENTS"
        ${ARGN}
      )

      # Validate required arguments for new syntax
      if(NOT ARG_PACKAGE)
        message(FATAL_ERROR "dart_find_package: PACKAGE is required when using named arguments")
      endif()

      if(NOT ARG_NAME)
        set(ARG_NAME ${ARG_PACKAGE})
      endif()

      # Build find_package arguments
      set(find_args ${ARG_PACKAGE})
      if(ARG_VERSION)
        list(APPEND find_args ${ARG_VERSION})
      endif()
      if(ARG_COMPONENTS)
        list(APPEND find_args COMPONENTS ${ARG_COMPONENTS})
      endif()
      if(ARG_REQUIRED)
        list(APPEND find_args REQUIRED)
      elseif(ARG_QUIET)
        list(APPEND find_args QUIET)
      endif()

      # Find the package
      find_package(${find_args})

      # Check if found (using standard CMake convention)
      set(found_var "${ARG_PACKAGE}_FOUND")

      # Track dependency status
      if(${found_var})
        set(DART_DEPS_FOUND ${DART_DEPS_FOUND} ${ARG_NAME} CACHE INTERNAL "List of found dependencies")
        if(DART_VERBOSE)
          # Get version if available
          set(version_var "${ARG_PACKAGE}_VERSION")
          if(DEFINED ${version_var})
            message(STATUS "  Found ${ARG_NAME} ${${version_var}}")
          else()
            message(STATUS "  Found ${ARG_NAME}")
          endif()
        endif()

        # Set optional variable to TRUE if requested
        if(ARG_SET_VAR)
          set(${ARG_SET_VAR} TRUE CACHE BOOL "${ARG_NAME} available" FORCE)
        endif()
      else()
        set(DART_DEPS_MISSING ${DART_DEPS_MISSING} ${ARG_NAME} CACHE INTERNAL "List of missing dependencies")
        if(DART_VERBOSE)
          message(STATUS "  ${ARG_NAME} not found")
        endif()

        # Set optional variable to FALSE if requested
        if(ARG_SET_VAR)
          set(${ARG_SET_VAR} FALSE CACHE BOOL "${ARG_NAME} available" FORCE)
        endif()
      endif()
    endif()
  endif()
endmacro()

#===============================================================================
# Library and Target Building
#===============================================================================

#-------------------------------------------------------------------------------
# Add a DART library with modern CMake properties
# Usage:
#   dart_add_library(
#     NAME <library_name>
#     [SOURCES source1.cpp source2.cpp...]
#     [HEADERS header1.hpp header2.hpp...]
#     [GLOB_SOURCES]
#     [GLOB_HEADERS]
#     [PUBLIC_INCLUDE_DIRS dir1 dir2...]
#     [PRIVATE_INCLUDE_DIRS dir1 dir2...]
#     [PUBLIC_LINK_LIBRARIES lib1 lib2...]
#     [PRIVATE_LINK_LIBRARIES lib1 lib2...]
#     [PUBLIC_COMPILE_DEFINITIONS def1 def2...]
#     [PRIVATE_COMPILE_DEFINITIONS def1 def2...]
#     [PUBLIC_COMPILE_OPTIONS opt1 opt2...]
#     [PRIVATE_COMPILE_OPTIONS opt1 opt2...]
#     [COMPILE_FEATURES feature1 feature2...]
#     [VERSION version_string]
#     [OUTPUT_NAME output_name]
#     [NO_INSTALL]
#     [NO_EXPORT_MACRO]
#   )
#
# This function creates a library target with all modern CMake best practices:
# - Automatically handles both SHARED and STATIC builds based on BUILD_SHARED_LIBS
# - Sets up proper include directories with BUILD_INTERFACE and INSTALL_INTERFACE
# - Configures installation rules for targets and headers
# - Applies version properties
# - Supports globbing of sources and headers
# - Generates export macros for symbol visibility
#-------------------------------------------------------------------------------
function(dart_add_library)
  set(prefix _ARG)
  set(options
    GLOB_SOURCES
    GLOB_HEADERS
    NO_INSTALL
    NO_EXPORT_MACRO
    NO_FORMAT
    NO_EXPORT
  )
  set(oneValueArgs
    NAME
    VERSION
    OUTPUT_NAME
  )
  set(multiValueArgs
    SOURCES
    HEADERS
    FORMAT_EXCLUDE
    SUBDIRECTORIES
    SUB_DIRS
    PUBLIC_INCLUDE_DIRS
    PRIVATE_INCLUDE_DIRS
    PUBLIC_SYSTEM_INCLUDE_DIRS
    PRIVATE_SYSTEM_INCLUDE_DIRS
    PUBLIC_LINK_LIBRARIES
    PRIVATE_LINK_LIBRARIES
    PUBLIC_COMPILE_DEFINITIONS
    PRIVATE_COMPILE_DEFINITIONS
    PUBLIC_COMPILE_OPTIONS
    PRIVATE_COMPILE_OPTIONS
    COMPILE_FEATURES
  )
  cmake_parse_arguments(
    "${prefix}" "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN}
  )

  # Validate required argument
  if(NOT _ARG_NAME)
    message(FATAL_ERROR "dart_add_library: NAME is required")
  endif()

  # Handle backward compatibility: if called as dart_add_library(name source1 source2...)
  # then treat unparsed arguments as sources
  if(_ARG_UNPARSED_ARGUMENTS)
    list(APPEND _ARG_SOURCES ${_ARG_UNPARSED_ARGUMENTS})
  endif()

  # Glob sources if requested
  if(_ARG_GLOB_SOURCES)
    file(GLOB_RECURSE glob_sources "*.cpp")
    list(APPEND _ARG_SOURCES ${glob_sources})
  endif()

  # Glob headers if requested
  if(_ARG_GLOB_HEADERS)
    file(GLOB_RECURSE glob_headers "*.hpp")
    list(APPEND _ARG_HEADERS ${glob_headers})
  endif()

  # Expand glob patterns in SOURCES
  set(expanded_sources)
  foreach(source IN LISTS _ARG_SOURCES)
    if(source MATCHES ".*[*?].*")
      file(GLOB matched_files "${source}")
      list(APPEND expanded_sources ${matched_files})
    else()
      list(APPEND expanded_sources ${source})
    endif()
  endforeach()
  set(_ARG_SOURCES ${expanded_sources})

  # Expand glob patterns in HEADERS
  set(expanded_headers)
  foreach(header IN LISTS _ARG_HEADERS)
    if(header MATCHES ".*[*?].*")
      file(GLOB matched_files "${header}")
      list(APPEND expanded_headers ${matched_files})
    else()
      list(APPEND expanded_headers ${header})
    endif()
  endforeach()
  set(_ARG_HEADERS ${expanded_headers})

  # Add requested subdirectories before creating the target so they can
  # contribute sources/headers via global properties.
  set(_dart_all_subdirs ${_ARG_SUBDIRECTORIES} ${_ARG_SUB_DIRS})
  foreach(_dart_subdir IN LISTS _dart_all_subdirs)
    add_subdirectory(${_dart_subdir})
  endforeach()

  # If no explicit headers/sources were provided, fall back to accumulated
  # core lists so subsystem additions can populate the target.
  if(NOT _ARG_HEADERS)
    get_property(_dart_fallback_headers GLOBAL PROPERTY DART_CORE_HEADERS)
    set(_ARG_HEADERS ${_dart_fallback_headers})
  endif()
  if(NOT _ARG_SOURCES)
    get_property(_dart_fallback_sources GLOBAL PROPERTY DART_CORE_SOURCES)
    set(_ARG_SOURCES ${_dart_fallback_sources})
  endif()

  # Incorporate contributions from subdirectories via global properties
  get_property(_dart_extra_public_includes GLOBAL PROPERTY DART_PUBLIC_INCLUDE_DIRS)
  if(_dart_extra_public_includes)
    list(REMOVE_DUPLICATES _dart_extra_public_includes)
    list(APPEND _ARG_PUBLIC_INCLUDE_DIRS ${_dart_extra_public_includes})
  endif()

  get_property(_dart_extra_public_system_includes GLOBAL PROPERTY DART_PUBLIC_SYSTEM_INCLUDE_DIRS)
  if(_dart_extra_public_system_includes)
    list(REMOVE_DUPLICATES _dart_extra_public_system_includes)
    list(APPEND _ARG_PUBLIC_SYSTEM_INCLUDE_DIRS ${_dart_extra_public_system_includes})
  endif()

  get_property(_dart_extra_link_libs GLOBAL PROPERTY DART_EXTRA_LINK_LIBS)
  if(_dart_extra_link_libs)
    list(APPEND _ARG_PUBLIC_LINK_LIBRARIES ${_dart_extra_link_libs})
  endif()

  get_property(_dart_extra_compile_opts GLOBAL PROPERTY DART_EXTRA_COMPILE_OPTIONS)
  if(_dart_extra_compile_opts)
    list(APPEND _ARG_PRIVATE_COMPILE_OPTIONS ${_dart_extra_compile_opts})
  endif()

  # Create the library target
  add_library(${_ARG_NAME} ${_ARG_SOURCES} ${_ARG_HEADERS})
  # Clear any directory-scoped include directories to avoid unintended propagation
  set_target_properties(${_ARG_NAME} PROPERTIES INCLUDE_DIRECTORIES "")

  # Set output name if specified
  if(_ARG_OUTPUT_NAME)
    set_target_properties(${_ARG_NAME} PROPERTIES OUTPUT_NAME ${_ARG_OUTPUT_NAME})
  endif()

  # Generate export macro for symbol visibility (unless disabled)
  if(NOT _ARG_NO_EXPORT_MACRO)
    if(BUILD_SHARED_LIBS)
      set(_dart_target_build_shared 1)
    else()
      set(_dart_target_build_shared 0)
    endif()
    string(REPLACE "-" "_" _dart_export_macro "${_ARG_NAME}")
    string(REPLACE ":" "_" _dart_export_macro "${_dart_export_macro}")
    string(REPLACE "/" "_" _dart_export_macro "${_dart_export_macro}")
    string(TOUPPER "${_dart_export_macro}" _dart_export_macro)
    target_compile_definitions(
      ${_ARG_NAME}
      PUBLIC
        DART_BUILD_SHARED=${_dart_target_build_shared}
      PRIVATE
        DART_BUILDING_${_dart_export_macro}
    )
  endif()

  # Set up include directories
  # Default to parent of current source dir if no public include dirs specified
  if(NOT _ARG_PUBLIC_INCLUDE_DIRS)
    set(_ARG_PUBLIC_INCLUDE_DIRS
      ${CMAKE_SOURCE_DIR}
      ${CMAKE_BINARY_DIR}
    )
  endif()
  if(_ARG_PUBLIC_INCLUDE_DIRS)
    list(REMOVE_DUPLICATES _ARG_PUBLIC_INCLUDE_DIRS)
  endif()
  if(_ARG_PUBLIC_SYSTEM_INCLUDE_DIRS)
    list(REMOVE_DUPLICATES _ARG_PUBLIC_SYSTEM_INCLUDE_DIRS)
  endif()

  target_include_directories(${_ARG_NAME}
    PUBLIC
      "$<BUILD_INTERFACE:${_ARG_PUBLIC_INCLUDE_DIRS}>"
      "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}
      ${_ARG_PRIVATE_INCLUDE_DIRS}
  )
  if(_ARG_PUBLIC_SYSTEM_INCLUDE_DIRS)
    target_include_directories(${_ARG_NAME}
      SYSTEM PUBLIC
        "$<BUILD_INTERFACE:${_ARG_PUBLIC_SYSTEM_INCLUDE_DIRS}>"
    )
  endif()
  if(_ARG_PRIVATE_SYSTEM_INCLUDE_DIRS)
    target_include_directories(${_ARG_NAME}
      SYSTEM PRIVATE
        ${_ARG_PRIVATE_SYSTEM_INCLUDE_DIRS}
    )
  endif()

  # Set compile features (default to C++20 if not specified)
  if(_ARG_COMPILE_FEATURES)
    target_compile_features(${_ARG_NAME} PUBLIC ${_ARG_COMPILE_FEATURES})
  else()
    target_compile_features(${_ARG_NAME} PUBLIC cxx_std_20)
  endif()

  # Link libraries
  if(_ARG_PUBLIC_LINK_LIBRARIES)
    target_link_libraries(${_ARG_NAME} PUBLIC ${_ARG_PUBLIC_LINK_LIBRARIES})
  endif()
  if(_ARG_PRIVATE_LINK_LIBRARIES)
    target_link_libraries(${_ARG_NAME} PRIVATE ${_ARG_PRIVATE_LINK_LIBRARIES})
  endif()

  # Compile definitions
  if(_ARG_PUBLIC_COMPILE_DEFINITIONS)
    target_compile_definitions(${_ARG_NAME} PUBLIC ${_ARG_PUBLIC_COMPILE_DEFINITIONS})
  endif()
  if(_ARG_PRIVATE_COMPILE_DEFINITIONS)
    target_compile_definitions(${_ARG_NAME} PRIVATE ${_ARG_PRIVATE_COMPILE_DEFINITIONS})
  endif()

  # Compile options
  if(_ARG_PUBLIC_COMPILE_OPTIONS)
    target_compile_options(${_ARG_NAME} PUBLIC ${_ARG_PUBLIC_COMPILE_OPTIONS})
  endif()
  if(_ARG_PRIVATE_COMPILE_OPTIONS)
    target_compile_options(${_ARG_NAME} PRIVATE ${_ARG_PRIVATE_COMPILE_OPTIONS})
  endif()

  # Set output directories
  set(_dart_use_global_output_dirs TRUE)
  if(DEFINED DART_USE_GLOBAL_OUTPUT_DIRS)
    if(NOT DART_USE_GLOBAL_OUTPUT_DIRS)
      set(_dart_use_global_output_dirs FALSE)
    endif()
  endif()

  if(_dart_use_global_output_dirs)
    if(CMAKE_CONFIGURATION_TYPES)
      foreach(_dart_config IN LISTS CMAKE_CONFIGURATION_TYPES)
        string(TOUPPER "${_dart_config}" _dart_config_upper)
        set_target_properties(
          ${_ARG_NAME}
          PROPERTIES
            ARCHIVE_OUTPUT_DIRECTORY_${_dart_config_upper}
              "${DART_BINARY_DIR}/lib/${_dart_config}"
            LIBRARY_OUTPUT_DIRECTORY_${_dart_config_upper}
              "${DART_BINARY_DIR}/lib/${_dart_config}"
            RUNTIME_OUTPUT_DIRECTORY_${_dart_config_upper}
              "${DART_BINARY_DIR}/bin/${_dart_config}"
        )
      endforeach()
    else()
      set_target_properties(
        ${_ARG_NAME}
        PROPERTIES
          ARCHIVE_OUTPUT_DIRECTORY "${DART_BINARY_DIR}/lib"
          LIBRARY_OUTPUT_DIRECTORY "${DART_BINARY_DIR}/lib"
          RUNTIME_OUTPUT_DIRECTORY "${DART_BINARY_DIR}/bin"
      )
    endif()
  else()
    if(CMAKE_CONFIGURATION_TYPES)
      foreach(_dart_config IN LISTS CMAKE_CONFIGURATION_TYPES)
        string(TOUPPER "${_dart_config}" _dart_config_upper)
        set_target_properties(
          ${_ARG_NAME}
          PROPERTIES
            ARCHIVE_OUTPUT_DIRECTORY_${_dart_config_upper}
              "${CMAKE_CURRENT_BINARY_DIR}/${_dart_config}"
            LIBRARY_OUTPUT_DIRECTORY_${_dart_config_upper}
              "${CMAKE_CURRENT_BINARY_DIR}/${_dart_config}"
            RUNTIME_OUTPUT_DIRECTORY_${_dart_config_upper}
              "${CMAKE_CURRENT_BINARY_DIR}/${_dart_config}"
        )
      endforeach()
    else()
      set_target_properties(
        ${_ARG_NAME}
        PROPERTIES
          ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
          LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
          RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
      )
    endif()
  endif()

  # Set version properties
  if(_ARG_VERSION)
    set(lib_version ${_ARG_VERSION})
  elseif(DEFINED DART_VERSION)
    set(lib_version ${DART_VERSION})
  elseif(DEFINED PROJECT_VERSION)
    set(lib_version ${PROJECT_VERSION})
  endif()

  if(lib_version)
    # Extract major.minor for SOVERSION
    string(REGEX MATCH "^([0-9]+)\\.([0-9]+)" _ ${lib_version})
    if(CMAKE_MATCH_1 AND CMAKE_MATCH_2)
      set(lib_soversion "${CMAKE_MATCH_1}.${CMAKE_MATCH_2}")
    else()
      set(lib_soversion ${lib_version})
    endif()

    set_target_properties(
      ${_ARG_NAME}
      PROPERTIES
        VERSION ${lib_version}
        SOVERSION ${lib_soversion}
    )
  endif()

  # Installation rules (unless NO_INSTALL is set)
  if(NOT _ARG_NO_INSTALL)
    include(GNUInstallDirs)
    set(_dart_export_set "${_ARG_NAME}Targets")
    if(_ARG_NO_EXPORT)
      set(_dart_export_set "")
    endif()

    # Install the target
    if(_dart_export_set)
      install(
        TARGETS ${_ARG_NAME}
        EXPORT ${_dart_export_set}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
      )
    else()
      install(
        TARGETS ${_ARG_NAME}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
      )
    endif()

    # Install headers with directory structure preserved
    if(_ARG_HEADERS)
      foreach(header IN LISTS _ARG_HEADERS)
        # Compute the relative path of each header from the source dir
        get_filename_component(abs_header "${header}" ABSOLUTE
          BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}"
        )
        file(RELATIVE_PATH rel_path "${CMAKE_SOURCE_DIR}" "${abs_header}")
        get_filename_component(rel_dir "${rel_path}" DIRECTORY)

        # Install the file to the destination, preserving the directory structure
        install(
          FILES "${abs_header}"
          DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${rel_dir}"
        )
      endforeach()
    endif()

    # Install export targets
    if(_dart_export_set)
      install(
        EXPORT ${_dart_export_set}
        NAMESPACE DART::
        DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake
      )
    endif()
  endif()

  # Add to formatting list
  if(NOT _ARG_NO_FORMAT)
    dart_format_add(
      EXCLUDE_PATTERNS ${_ARG_FORMAT_EXCLUDE}
      FILES ${_ARG_SOURCES} ${_ARG_HEADERS}
    )
  endif()

endfunction()

#-------------------------------------------------------------------------------
# Check if a required package is found and print status
# Usage:
#   dart_check_required_package(variable dependency)
# Arguments:
#   variable: The package variable name (e.g., FOO for FOO_FOUND)
#   dependency: Human-readable dependency name
#-------------------------------------------------------------------------------
function(dart_check_required_package variable dependency)
  if(${${variable}_FOUND})
    if(DART_VERBOSE)
      message(STATUS "Looking for ${dependency} - version ${${variable}_VERSION} found")
    endif()
  endif()
endfunction()

#-------------------------------------------------------------------------------
# Check if an optional package is found with skip option
# Usage:
#   dart_check_optional_package(variable component dependency [version])
# Arguments:
#   variable: The package variable name (e.g., FOO for FOO_FOUND)
#   component: The component that depends on this package
#   dependency: Human-readable dependency name
#   version: Optional minimum version requirement (ARGV3)
#-------------------------------------------------------------------------------
macro(dart_check_optional_package variable component dependency)
  option(DART_SKIP_${variable} "If ON, do not use ${variable} even if it is found." OFF)
  mark_as_advanced(DART_SKIP_${variable})
  if(${${variable}_FOUND} AND NOT ${DART_SKIP_${variable}})
    set(HAVE_${variable} TRUE CACHE BOOL "Check if ${variable} found." FORCE)
    if(DART_VERBOSE)
      message(STATUS "Looking for ${dependency} - version ${${variable}_VERSION}"
                     " found")
    endif()
  else()
    set(HAVE_${variable} FALSE CACHE BOOL "Check if ${variable} found." FORCE)
    if(NOT ${${variable}_FOUND})
      if(ARGV3) # version
        message(WARNING "Looking for ${dependency} - NOT found, to use"
                        " ${component}, please install ${dependency} (>= ${ARGV3})")
      else()
        message(WARNING "Looking for ${dependency} - NOT found, to use"
                        " ${component}, please install ${dependency}")
      endif()
    elseif(${DART_SKIP_${variable}} AND DART_VERBOSE)
      message(STATUS "Not using ${dependency} - version ${${variable}_VERSION}"
                     " even if found because DART_SKIP_${variable} is ON.")
    endif()
    return()
  endif()
endmacro()

#-------------------------------------------------------------------------------
# Check if dependent targets exist before building
# Usage:
#   dart_check_dependent_target(target dependent_target1 [dependent_target2...])
#-------------------------------------------------------------------------------
macro(dart_check_dependent_target target)
  foreach(dependent_target ${ARGN})
    if(NOT TARGET ${dependent_target})
      message(WARNING "${target} is disabled because dependent target ${dependent_target} is not being built.")
      return()
    endif()
  endforeach()
endmacro()

#-------------------------------------------------------------------------------
# Add a custom target from a directory
# Usage:
#   dart_add_custom_target(rel_dir property_name [link_libraries...])
#-------------------------------------------------------------------------------
function(dart_add_custom_target rel_dir property_name)
  set(abs_dir "${CMAKE_CURRENT_LIST_DIR}/${rel_dir}")

  if(NOT IS_DIRECTORY ${abs_dir})
    message(SEND_ERROR "Failed to find directory: ${abs_dir}")
    return()
  endif()

  get_filename_component(target_name ${rel_dir} NAME)

  file(GLOB hdrs "${abs_dir}/*.hpp")
  file(GLOB srcs "${abs_dir}/*.cpp")
  if(srcs)
    add_executable(${target_name} EXCLUDE_FROM_ALL ${hdrs} ${srcs})
    target_link_libraries(${target_name} ${ARGN})
    dart_property_add(${property_name} ${target_name})
  endif()
endfunction()

#===============================================================================
# Examples and Tutorials
#===============================================================================

#-------------------------------------------------------------------------------
# Add an example to the global examples list
# Usage:
#   dart_add_example(target1 [target2...])
#-------------------------------------------------------------------------------
function(dart_add_example)
  dart_property_add(DART_EXAMPLES ${ARGN})
endfunction()

#-------------------------------------------------------------------------------
# Add a tutorial to the global tutorials list
# Usage:
#   dart_add_tutorial(target1 [target2...])
#-------------------------------------------------------------------------------
function(dart_add_tutorial)
  dart_property_add(DART_TUTORIALS ${ARGN})
endfunction()

#===============================================================================
# Code Formatting
#===============================================================================

#-------------------------------------------------------------------------------
# Add source files to the formatting list
# Usage:
#   dart_format_add(source1 [source2...])
#-------------------------------------------------------------------------------
function(dart_format_add)
  set(options NO_FORMAT)
  set(oneValueArgs)
  set(multiValueArgs EXCLUDE_PATTERNS FILES)
  cmake_parse_arguments(DFA "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  if(DFA_NO_FORMAT)
    return()
  endif()

  get_property(_dart_format_excludes GLOBAL PROPERTY DART_FORMAT_EXCLUDE_PATTERNS)
  list(APPEND _dart_format_excludes ${DFA_EXCLUDE_PATTERNS})

  set(_dart_format_files ${DFA_FILES} ${DFA_UNPARSED_ARGUMENTS})

  foreach(source IN LISTS _dart_format_files)
    set(_dart_skip FALSE)
    if(IS_ABSOLUTE "${source}")
      set(source_abs "${source}")
    else()
      get_filename_component(source_abs
        "${CMAKE_CURRENT_LIST_DIR}/${source}" ABSOLUTE)
    endif()
    foreach(_dart_exclude IN LISTS _dart_format_excludes)
      if(_dart_exclude AND "${source_abs}" MATCHES "${_dart_exclude}")
        set(_dart_skip TRUE)
        break()
      endif()
    endforeach()
    if(_dart_skip)
      continue()
    endif()
    if(EXISTS "${source_abs}")
      dart_property_add(DART_FORMAT_FILES "${source_abs}")
    else()
      message(FATAL_ERROR
        "Source file '${source}' does not exist at absolute path"
        " '${source_abs}'. This should never happen. Did you recently delete"
        " this file or modify 'CMAKE_CURRENT_LIST_DIR'")
    endif()
  endforeach()
endfunction()

#===============================================================================
# Building Targets, Examples, and Tutorials
#===============================================================================

#-------------------------------------------------------------------------------
# Build a target from source files in the current directory
# Usage:
#   dart_build_target_in_source(target
#     [LINK_LIBRARIES library1...]
#     [COMPILE_FEATURES feature1...]
#     [COMPILE_OPTIONS option1...]
#     [INCLUDE_DIRS dir1...]
#     [COMPILE_DEFINITIONS def1...]
#   )
#-------------------------------------------------------------------------------
function(dart_build_target_in_source target)
  set(prefix _ARG)
  set(options)
  set(oneValueArgs)
  set(multiValueArgs LINK_LIBRARIES COMPILE_FEATURES COMPILE_OPTIONS INCLUDE_DIRS COMPILE_DEFINITIONS)
  cmake_parse_arguments("${prefix}" "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  if(_ARG_LINK_LIBRARIES)
    foreach(dep_target ${_ARG_LINK_LIBRARIES})
      if(NOT TARGET ${dep_target})
        if(DART_VERBOSE)
          message(WARNING "Skipping ${target} because required target '${dep_target}' not found")
        endif()
        return()
      endif()
    endforeach()
  endif()

  file(GLOB srcs "*.cpp" "*.hpp")

  add_executable(${target} ${srcs})

  if(_ARG_LINK_LIBRARIES)
    target_link_libraries(${target} ${_ARG_LINK_LIBRARIES})
  endif()

  if(_ARG_COMPILE_FEATURES)
    target_compile_features(${target} PUBLIC ${_ARG_COMPILE_FEATURES})
  else()
    target_compile_features(${target} PUBLIC cxx_std_20)
  endif()

  if(_ARG_COMPILE_OPTIONS)
    target_compile_options(${target} PUBLIC ${_ARG_COMPILE_OPTIONS})
  endif()

  if(_ARG_INCLUDE_DIRS)
    target_include_directories(${target} PRIVATE ${_ARG_INCLUDE_DIRS})
  endif()

  if(_ARG_COMPILE_DEFINITIONS)
    target_compile_definitions(${target} PRIVATE ${_ARG_COMPILE_DEFINITIONS})
  endif()

  set_target_properties(${target}
    PROPERTIES
      RUNTIME_OUTPUT_DIRECTORY "${DART_BINARY_DIR}/bin"
  )

  dart_format_add(${srcs})
endfunction()

#-------------------------------------------------------------------------------
# Build an example from source files in the current directory
# Usage:
#   dart_build_example_in_source(target
#     [LINK_LIBRARIES library1...]
#     [COMPILE_FEATURES feature1...]
#     [COMPILE_OPTIONS option1...]
#   )
#-------------------------------------------------------------------------------
function(dart_build_example_in_source target)
  dart_build_target_in_source(${target} ${ARGN})
  dart_add_example(${target})
endfunction()

#-------------------------------------------------------------------------------
# Build a tutorial from source files in the current directory
# Usage:
#   dart_build_tutorial_in_source(target
#     [LINK_LIBRARIES library1...]
#     [COMPILE_FEATURES feature1...]
#     [COMPILE_OPTIONS option1...]
#   )
#-------------------------------------------------------------------------------
function(dart_build_tutorial_in_source target)
  dart_build_target_in_source(${target} ${ARGN})
  dart_add_tutorial(${target})
endfunction()


#===============================================================================
# Testing
#===============================================================================

#-------------------------------------------------------------------------------
# Build tests from source files
# Usage:
#   dart_build_tests(
#     TYPE <test_type>
#     [COMPONENT_NAME component_name]
#     [TARGET_PREFIX prefix]
#     [GLOB_SOURCES]
#     [SOURCES source1 source2...]
#     [INCLUDE_DIRS dir1 dir2...]
#     [LINK_LIBRARIES lib1 lib2...]
#   )
#-------------------------------------------------------------------------------
function(dart_build_tests)
  set(prefix _ARG)
  set(options
    GLOB_SOURCES
  )
  set(oneValueArgs
    COMPONENT_NAME
    TARGET_PREFIX
    TYPE
  )
  set(multiValueArgs
    INCLUDE_DIRS
    LINK_LIBRARIES
    SOURCES
  )
  cmake_parse_arguments(
    "${prefix}" "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN}
  )

  if(NOT _ARG_TARGET_PREFIX)
    set(_ARG_TARGET_PREFIX UNIT)
  endif()

  if(NOT _ARG_TYPE)
    message(FATAL_ERROR "TYPE is not set!")
  endif()

  foreach(dep IN LISTS _ARG_LINK_LIBRARIES)
    if(NOT TARGET ${dep})
      if(_ARG_COMPONENT_NAME)
        message(WARNING "Skipping tests for component [${_ARG_COMPONENT_NAME}] due to missing component target [${dep}]")
      else()
        message(WARNING "Skipping tests due to missing component target [${dep}]")
      endif()
      return()
    endif()
  endforeach()

  # Glob all the test files
  if(_ARG_GLOB_SOURCES)
    file(GLOB_RECURSE glob_test_files RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "test_*.cpp")
    list(APPEND test_files ${glob_test_files})
  endif()
  list(APPEND test_files ${_ARG_SOURCES})
  if(test_files)
    list(SORT test_files)
  endif()

  foreach(source IN LISTS test_files)
    # Set target name
    if(_ARG_TARGET_PREFIX)
      set(target_name ${_ARG_TARGET_PREFIX}_)
    else()
      set(target_name )
    endif()
    get_filename_component(source_name ${source} NAME_WE)
    string(REPLACE "test_" "" source_name ${source_name})
    get_filename_component(source_dir ${source} DIRECTORY)
    if(source_dir)
      string(REPLACE "/" "_" source_prefix ${source_dir})
      set(target_name "${target_name}${source_prefix}_${source_name}")
    else()
      set(target_name "${target_name}${source_name}")
    endif()

    if(MSVC)
      add_executable(${target_name} ${source})
    else()
      add_executable(${target_name} EXCLUDE_FROM_ALL ${source})
    endif()
    add_test(NAME ${target_name} COMMAND $<TARGET_FILE:${target_name}>)

    # Include directories
    target_include_directories(
      ${target_name} PRIVATE ${_ARG_INCLUDE_DIRS}
    )

    # Link libraries
    target_link_libraries(${target_name} PRIVATE GTest::gtest GTest::gtest_main)
    target_link_libraries(
      ${target_name} PRIVATE ${_ARG_LINK_LIBRARIES}
    )
    if(UNIX)
      # gtest requies pthread when compiled on a Unix machine
      target_link_libraries(${target_name} PRIVATE pthread)
    endif()

    # Add the test target to the list
    dart_property_add(DART_${_ARG_TYPE}_TESTS ${target_name})

  endforeach()

  dart_format_add(${test_files})

endfunction()

#-------------------------------------------------------------------------------
# Add a single test with CTest integration
# Usage:
#   dart_add_test(
#     NAME <test_name>
#     SOURCES <source_files...>
#     [LINK_LIBRARIES lib1 lib2...]
#     [LABELS label1 label2...]
#     [WORKING_DIRECTORY dir]
#     [INCLUDE_DIRS dir1 dir2...]
#     [COMPILE_DEFINITIONS def1 def2...]
#     [COMPILE_OPTIONS opt1 opt2...]
#     [COMPILE_FEATURES feature1 feature2...]
#     [RUNTIME_PATH_TARGET target]  # defaults to first linked target
#     [FOLDER folder_name]
#   )
#
# Automatically adds the test to CTest with the specified labels
#-------------------------------------------------------------------------------
function(dart_add_test)
  set(prefix _ARG)
  set(options)
  set(oneValueArgs
    NAME
    WORKING_DIRECTORY
    RUNTIME_PATH_TARGET
    FOLDER
  )
  set(multiValueArgs
    SOURCES
    LINK_LIBRARIES
    LABELS
    INCLUDE_DIRS
    COMPILE_DEFINITIONS
    COMPILE_OPTIONS
    COMPILE_FEATURES
  )
  cmake_parse_arguments(
    "${prefix}" "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN}
  )

  if(NOT _ARG_NAME)
    message(FATAL_ERROR "dart_add_test: NAME is required")
  endif()

  if(NOT _ARG_SOURCES)
    message(FATAL_ERROR "dart_add_test: SOURCES is required")
  endif()

  # Create the test executable
  add_executable(${_ARG_NAME} ${_ARG_SOURCES})

  if(_ARG_INCLUDE_DIRS)
    target_include_directories(${_ARG_NAME} PRIVATE ${_ARG_INCLUDE_DIRS})
  endif()

  if(_ARG_COMPILE_DEFINITIONS)
    target_compile_definitions(${_ARG_NAME} PRIVATE ${_ARG_COMPILE_DEFINITIONS})
  endif()

  if(_ARG_COMPILE_OPTIONS)
    target_compile_options(${_ARG_NAME} PRIVATE ${_ARG_COMPILE_OPTIONS})
  endif()

  if(_ARG_COMPILE_FEATURES)
    target_compile_features(${_ARG_NAME} PUBLIC ${_ARG_COMPILE_FEATURES})
  else()
    target_compile_features(${_ARG_NAME} PUBLIC cxx_std_20)
  endif()

  # Link default test libraries
  target_link_libraries(${_ARG_NAME}
    PRIVATE
      GTest::gtest
      GTest::gtest_main
      ${_ARG_LINK_LIBRARIES}
  )

  # Add to CTest with optional labels
  add_test(NAME ${_ARG_NAME} COMMAND $<TARGET_FILE:${_ARG_NAME}>)

  if(_ARG_LABELS)
    set_tests_properties(${_ARG_NAME} PROPERTIES LABELS "${_ARG_LABELS}")
  endif()

  if(_ARG_WORKING_DIRECTORY)
    set_tests_properties(${_ARG_NAME} PROPERTIES WORKING_DIRECTORY "${_ARG_WORKING_DIRECTORY}")
  endif()

  if(_ARG_FOLDER)
    set_target_properties(${_ARG_NAME} PROPERTIES FOLDER "${_ARG_FOLDER}")
  endif()

  set_target_properties(${_ARG_NAME}
    PROPERTIES
      RUNTIME_OUTPUT_DIRECTORY "${DART_BINARY_DIR}/bin"
  )

  # Propagate a dependency's runtime path to the test if requested or discoverable
  set(_dart_runtime_path_target "${_ARG_RUNTIME_PATH_TARGET}")
  if(NOT _dart_runtime_path_target)
    foreach(_dart_candidate IN LISTS _ARG_LINK_LIBRARIES)
      if(TARGET ${_dart_candidate})
        set(_dart_runtime_path_target ${_dart_candidate})
        break()
      endif()
    endforeach()
  endif()

  if(_dart_runtime_path_target)
    if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.22")
      set_tests_properties(${_ARG_NAME}
        PROPERTIES
          ENVIRONMENT_MODIFICATION
            "PATH=path_list_prepend:$<TARGET_FILE_DIR:${_dart_runtime_path_target}>"
      )
    else()
      if(WIN32)
        set(_dart_test_path_sep "\\;")
      else()
        set(_dart_test_path_sep ":")
      endif()
      set_property(
        TEST ${_ARG_NAME}
        PROPERTY ENVIRONMENT
          "PATH=$<TARGET_FILE_DIR:${_dart_runtime_path_target}>${_dart_test_path_sep}$ENV{PATH}"
      )
      unset(_dart_test_path_sep)
    endif()
  endif()

  # Add to global test list
  dart_property_add(DART_ALL_TESTS ${_ARG_NAME})

  # Add to formatting list
  dart_format_add(${_ARG_SOURCES})
endfunction()

#-------------------------------------------------------------------------------
# Discover and add all unit tests in a directory
# Usage:
#   dart_add_unit_test_dir(
#     MODULE_NAME <module_name>
#     TEST_DIR <directory>
#     [LINK_LIBRARIES lib1 lib2...]
#     [INCLUDE_DIRS dir1 dir2...]
#     [COMPILE_DEFINITIONS def1 def2...]
#     [COMPILE_OPTIONS opt1 opt2...]
#     [COMPILE_FEATURES feature1 feature2...]
#     [RUNTIME_PATH_TARGET target]
#   )
#
# Finds all test_*.cpp files in TEST_DIR and creates a test for each
#-------------------------------------------------------------------------------
function(dart_add_unit_test_dir)
  set(prefix _ARG)
  set(options)
  set(oneValueArgs
    MODULE_NAME
    TEST_DIR
    RUNTIME_PATH_TARGET
  )
  set(multiValueArgs
    LINK_LIBRARIES
    INCLUDE_DIRS
    COMPILE_DEFINITIONS
    COMPILE_OPTIONS
    COMPILE_FEATURES
  )
  cmake_parse_arguments(
    "${prefix}" "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN}
  )

  if(NOT _ARG_MODULE_NAME)
    message(FATAL_ERROR "dart_add_unit_test_dir: MODULE_NAME is required")
  endif()

  if(NOT _ARG_TEST_DIR)
    message(FATAL_ERROR "dart_add_unit_test_dir: TEST_DIR is required")
  endif()

  # Find all test files in the directory
  file(GLOB test_files "${_ARG_TEST_DIR}/test_*.cpp")

  if(NOT test_files)
    message(STATUS "  ${_ARG_MODULE_NAME}: no tests found in ${_ARG_TEST_DIR}")
    return()
  endif()

  # Register each test
  set(module_test_targets)
  foreach(test_file IN LISTS test_files)
    # Extract test name from filename (remove .cpp extension)
    get_filename_component(test_name ${test_file} NAME_WE)

    # Pick a runtime path target if one of the linked libraries is a target
    set(_dart_runtime_path_target "${_ARG_RUNTIME_PATH_TARGET}")
    if(NOT _dart_runtime_path_target)
      foreach(_dart_candidate IN LISTS _ARG_LINK_LIBRARIES)
        if(TARGET ${_dart_candidate})
          set(_dart_runtime_path_target ${_dart_candidate})
          break()
        endif()
      endforeach()
    endif()

    # Create the test
    set(_dart_add_test_args
      NAME ${test_name}
      SOURCES ${test_file}
      LINK_LIBRARIES ${_ARG_LINK_LIBRARIES}
      LABELS ${_ARG_MODULE_NAME}
      FOLDER "tests/${_ARG_MODULE_NAME}"
    )

    if(_ARG_INCLUDE_DIRS)
      list(APPEND _dart_add_test_args INCLUDE_DIRS ${_ARG_INCLUDE_DIRS})
    endif()

    if(_ARG_COMPILE_DEFINITIONS)
      list(APPEND _dart_add_test_args COMPILE_DEFINITIONS ${_ARG_COMPILE_DEFINITIONS})
    endif()

    if(_ARG_COMPILE_OPTIONS)
      list(APPEND _dart_add_test_args COMPILE_OPTIONS ${_ARG_COMPILE_OPTIONS})
    endif()

    if(_ARG_COMPILE_FEATURES)
      list(APPEND _dart_add_test_args COMPILE_FEATURES ${_ARG_COMPILE_FEATURES})
    endif()

    if(_dart_runtime_path_target)
      list(APPEND _dart_add_test_args RUNTIME_PATH_TARGET ${_dart_runtime_path_target})
    endif()

    dart_add_test(${_dart_add_test_args})

    # Add to this module's test list
    list(APPEND module_test_targets ${test_name})
  endforeach()

  # Create meta target for this module
  add_custom_target(tests_${_ARG_MODULE_NAME}
    DEPENDS ${module_test_targets}
    COMMENT "Building ${_ARG_MODULE_NAME} tests"
  )

  # Report
  list(LENGTH test_files num_tests)
  message(STATUS "  ${_ARG_MODULE_NAME}: ${num_tests} test(s)")
endfunction()

#===============================================================================
# Python Bindings
#===============================================================================

#-------------------------------------------------------------------------------
# Add a Python binding module with nanobind
# Usage:
#   dart_add_python_module(
#     NAME <module_name>
#     SOURCES <source_files...>
#     [LINK_LIBRARIES lib1 lib2...]
#     [VERSION <version>]
#   )
#
# Creates a Python extension module using nanobind.
# Automatically finds Python and nanobind.
#-------------------------------------------------------------------------------
function(dart_add_python_module)
  set(prefix _ARG)
  set(options)
  set(oneValueArgs
    NAME
    VERSION
  )
  set(multiValueArgs
    SOURCES
    LINK_LIBRARIES
  )
  cmake_parse_arguments(
    "${prefix}" "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN}
  )

  # Validate required arguments
  if(NOT _ARG_NAME)
    message(FATAL_ERROR "dart_add_python_module: NAME is required")
  endif()

  if(NOT _ARG_SOURCES)
    message(FATAL_ERROR "dart_add_python_module: SOURCES is required")
  endif()

  # Expand glob patterns in SOURCES
  set(expanded_sources)
  foreach(source IN LISTS _ARG_SOURCES)
    if(source MATCHES ".*[*?].*")
      file(GLOB matched_files "${source}")
      list(APPEND expanded_sources ${matched_files})
    else()
      list(APPEND expanded_sources ${source})
    endif()
  endforeach()
  set(_ARG_SOURCES ${expanded_sources})

  # Use PROJECT_VERSION if VERSION not specified
  if(NOT _ARG_VERSION)
    if(PROJECT_VERSION)
      set(_ARG_VERSION ${PROJECT_VERSION})
    else()
      message(FATAL_ERROR "dart_add_python_module: VERSION is required (or set project VERSION)")
    endif()
  endif()

  # Find Python
  find_package(Python COMPONENTS Interpreter Development REQUIRED)

  # Try to find nanobind using Python
  execute_process(
    COMMAND ${Python_EXECUTABLE} -m nanobind --cmake_dir
    OUTPUT_VARIABLE nanobind_ROOT
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
    RESULT_VARIABLE nanobind_NOTFOUND
  )

  if(nanobind_NOTFOUND)
    message(STATUS "nanobind not found. Skipping ${_ARG_NAME}.")
    message(STATUS "To enable ${_ARG_NAME}, install nanobind: pip install nanobind")
    return()
  endif()

  # Find nanobind
  list(APPEND CMAKE_PREFIX_PATH "${nanobind_ROOT}")
  find_package(nanobind CONFIG REQUIRED)

  message(STATUS "Building Python module ${_ARG_NAME} with nanobind at ${nanobind_ROOT}")

  # Build Python extension module using nanobind
  nanobind_add_module(${_ARG_NAME}
    NB_STATIC  # Build a static extension module
    ${_ARG_SOURCES}
  )

  # Include directories
  target_include_directories(${_ARG_NAME}
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}
      ${CMAKE_SOURCE_DIR}
      ${CMAKE_BINARY_DIR}
  )

  # Link libraries
  if(_ARG_LINK_LIBRARIES)
    target_link_libraries(${_ARG_NAME} PRIVATE ${_ARG_LINK_LIBRARIES})
  endif()

  # Set version info
  target_compile_definitions(${_ARG_NAME}
    PRIVATE ${_ARG_NAME}_VERSION_INFO="${_ARG_VERSION}"
  )

  # Remove debug postfix
  set_target_properties(${_ARG_NAME} PROPERTIES DEBUG_POSTFIX "")

  # Set output directory to match expected PYTHONPATH
  set_target_properties(${_ARG_NAME} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/python"
    LIBRARY_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/Debug/python"
    LIBRARY_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/Release/python"
  )

  # Add to formatting list
  dart_format_add(${_ARG_SOURCES})

  message(STATUS "${_ARG_NAME}: Configured Python module")
endfunction()

#===============================================================================
# Benchmarking
#===============================================================================

#-------------------------------------------------------------------------------
# Add a single benchmark
# Usage:
#   dart_add_benchmark(
#     NAME <benchmark_name>
#     SOURCES <source_files...>
#     [LINK_LIBRARIES lib1 lib2...]
#     [INCLUDE_DIRS dir1 dir2...]
#     [COMPILE_DEFINITIONS def1 def2...]
#     [COMPILE_OPTIONS opt1 opt2...]
#     [COMPILE_FEATURES feature1 feature2...]
#     [FOLDER folder_name]
#   )
#
# Creates a benchmark executable using Google Benchmark.
#-------------------------------------------------------------------------------
function(dart_add_benchmark)
  set(prefix _ARG)
  set(options)
  set(oneValueArgs NAME FOLDER)
  set(multiValueArgs
    SOURCES
    LINK_LIBRARIES
    INCLUDE_DIRS
    COMPILE_DEFINITIONS
    COMPILE_OPTIONS
    COMPILE_FEATURES
  )
  cmake_parse_arguments(
    "${prefix}" "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN}
  )

  if(NOT _ARG_NAME)
    message(FATAL_ERROR "dart_add_benchmark: NAME is required")
  endif()

  if(NOT _ARG_SOURCES)
    message(FATAL_ERROR "dart_add_benchmark: SOURCES is required")
  endif()

  # Create the benchmark executable
  add_executable(${_ARG_NAME} ${_ARG_SOURCES})

  if(_ARG_INCLUDE_DIRS)
    target_include_directories(${_ARG_NAME} PRIVATE ${_ARG_INCLUDE_DIRS})
  endif()

  if(_ARG_COMPILE_DEFINITIONS)
    target_compile_definitions(${_ARG_NAME} PRIVATE ${_ARG_COMPILE_DEFINITIONS})
  endif()

  if(_ARG_COMPILE_OPTIONS)
    target_compile_options(${_ARG_NAME} PRIVATE ${_ARG_COMPILE_OPTIONS})
  endif()

  if(_ARG_COMPILE_FEATURES)
    target_compile_features(${_ARG_NAME} PUBLIC ${_ARG_COMPILE_FEATURES})
  else()
    target_compile_features(${_ARG_NAME} PUBLIC cxx_std_20)
  endif()

  # Link benchmark libraries
  target_link_libraries(${_ARG_NAME}
    PRIVATE
      benchmark::benchmark
      benchmark::benchmark_main
      ${_ARG_LINK_LIBRARIES}
  )

  # Set benchmark properties
  if(NOT _ARG_FOLDER)
    set(_ARG_FOLDER "benchmarks")
  endif()
  set_target_properties(${_ARG_NAME} PROPERTIES
    FOLDER "${_ARG_FOLDER}"
    RUNTIME_OUTPUT_DIRECTORY "${DART_BINARY_DIR}/bin"
  )

  # Add to global benchmark list
  dart_property_add(DART_ALL_BENCHMARKS ${_ARG_NAME})

  # Add to formatting list
  dart_format_add(${_ARG_SOURCES})
endfunction()
