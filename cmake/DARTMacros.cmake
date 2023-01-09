# Copyright (c) 2011-2023, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/master/LICENSE
#
# This file is provided under the "BSD-style" License

# ==============================================================================
# dart_get_max(var [value1 value2...])
function(dart_get_max var)
  set(first YES)
  set(choice NO)
  foreach(item ${ARGN})
    if(first)
      set(choice ${item})
      set(first NO)
    elseif(choice LESS ${item})
      set(choice ${item})
    endif()
  endforeach(item)
  set(${var} ${choice} PARENT_SCOPE)
endfunction()

# ==============================================================================
# dart_get_max_string_length(var [value1 value2...])
function(dart_get_max_string_length var)
  foreach(item ${ARGN})
    string(LENGTH ${item} length)
    list(APPEND list ${length})
  endforeach()
  dart_get_max(choice ${list})
  set(${var} ${choice} PARENT_SCOPE)
endfunction()

# ==============================================================================
# cmake-format: off
# dart_option(<variable> "<help_text>" <value>)
# cmake-format: on
function(dart_option variable help_text default_value)
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

  # Add option
  option(${variable} ${help_text} ${default_value})

  # Normalize boolean value variants (e.g. 1/0, On/Off, TRUE/FALSE) to ON/OFF
  if(${variable})
    set(${variable} ON PARENT_SCOPE)
  else()
    set(${variable} OFF PARENT_SCOPE)
  endif()

endfunction()

# ==============================================================================
# cmake-format: off
# dart_print_options()
# cmake-format: on
function(dart_print_options)
  # Print the header
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

  dart_get_max_string_length(option_variable_max_len ${option_variables})
  list(LENGTH option_variables option_count)
  math(EXPR option_count "${option_count} - 1")
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

    message(STATUS "${option_str}")
  endforeach()

  message(STATUS "")
endfunction()

#===============================================================================
# Appends items to a cached list.
# Usage:
#   dart_append_to_cached_string(_string _cacheDesc [items...])
#===============================================================================
macro(dart_append_to_cached_string _string _cacheDesc)
  foreach(newItem ${ARGN})
    set(${_string} "${${_string}}${newItem}")
  endforeach()
endmacro()

#===============================================================================
# Get list of file names give list of full paths.
# Usage:
#   dart_get_filename_components(_var _cacheDesc [items...])
#===============================================================================
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

#===============================================================================
# Generate directory list of ${curdir}
# Usage:
#   dart_get_subdir_list(var curdir)
#===============================================================================
macro(dart_get_subdir_list var curdir)
    file(GLOB children RELATIVE ${curdir} "${curdir}/*")
    set(dirlist "")
    foreach(child ${children})
        if(IS_DIRECTORY ${curdir}/${child})
            LIST(APPEND dirlist ${child})
        endif()
    endforeach()
    set(${var} ${dirlist})
endmacro()

# ==============================================================================
# cmake-format: off
# dart_check_compiler_visibility(<output_variable>)
#
# Macro to check for visibility capability in compiler
# cmake-format: on
macro(dart_check_compiler_visibility variable)
  include(CheckCXXCompilerFlag)
  check_cxx_compiler_flag(-fvisibility=hidden ${variable})
endmacro()

# ==============================================================================
# dart_generate_export_header(
#   TARGET_NAME <target_name>
#   DESTINATION <dir_name>
#   INCLUDE_DIR <dir_name>
#   EXPORT_FILE_NAME <file_name>
#   [BASE_NAME <base_name>]
#   [EXPORT_ALL_SYMBOLS_BY_DEFAULT]
# )
#
# Function to create an export header for control of binary symbols visibility
#
function(dart_generate_export_header)
  set(prefix _ARG)
  set(options EXPORT_ALL_SYMBOLS_BY_DEFAULT)
  set(oneValueArgs
    TARGET_NAME
    DESTINATION
    EXPORT_FILE_NAME
    BASE_NAME
    BASE_DIR
  )
  set(multiValueArgs)
  cmake_parse_arguments(
    "${prefix}"
    "${options}"
    "${oneValueArgs}"
    "${multiValueArgs}"
    ${ARGN}
  )

  # Check required argument
  if(NOT _ARG_TARGET_NAME)
    message(FATAL_ERROR "DEVELOPER ERROR: You must specify TARGET_NAME!")
    return()
  endif()
  if(NOT _ARG_DESTINATION)
    message(FATAL_ERROR "DEVELOPER ERROR: You must specify DESTINATION!")
    return()
  endif()
  if(NOT _ARG_EXPORT_FILE_NAME)
    message(FATAL_ERROR "DEVELOPER ERROR: You must specify EXPORT_FILE_NAME!")
    return()
  endif()

  # Check if target is valid
  if(NOT TARGET ${_ARG_TARGET_NAME})
    message(
      FATAL_ERROR
        "DEVELOPER ERROR: Invalid target "
        "\"${_ARG_TARGET_NAME}\" is passed! "
        "Make sure this function is called after the target is defined by "
        "add_library(<target> ...).")
    return()
  endif()

  # Hide symbols by default
  if(UNIX AND NOT _ARG_EXPORT_ALL_SYMBOLS_BY_DEFAULT)
    dart_check_compiler_visibility(compiler_supports_visibility)
    if(compiler_supports_visibility)
      target_compile_options(${_ARG_TARGET_NAME} PRIVATE -fvisibility=hidden)
    endif()
  endif()

  # Base name
  if(_ARG_BASE_NAME)
    set(base_name ${_ARG_BASE_NAME})
  else()
    set(base_name "${_ARG_TARGET_NAME}")
    string(REPLACE "-" "_" base_name ${base_name})
  endif()
  string(TOUPPER ${base_name} base_name)

  # Set up paths
  set(export_file_path "${_ARG_DESTINATION}/${_ARG_EXPORT_FILE_NAME}")
  set(export_detail_file_path "${_ARG_DESTINATION}/detail/${_ARG_EXPORT_FILE_NAME}")

  # Generate CMake's default export header
  include(GenerateExportHeader)
  generate_export_header(
    ${_ARG_TARGET_NAME}
    EXPORT_MACRO_NAME DETAIL_${base_name}_API
    EXPORT_FILE_NAME ${export_detail_file_path}
  )

  # Generate final export header
  file(
    WRITE ${export_file_path}
    "// This file is automatically generated by ${PROJECT_NAME}.\n"
    "\n"
    "#pragma once\n"
    "\n"
    "/**\n"
    " * @brief Apply this macro to classes and functions that will need to be exposed\n"
    "   to the consumer libraries or programs.\n"
    " */\n"
    "#define ${base_name}_API \\\n"
    "    DETAIL_${base_name}_API\n"
    "\n"
    "#ifdef _MSC_VER\n"
    "  #define ${base_name}_TEMPL_INST_DECL_API\n"
    "#else\n"
    "  #define ${base_name}_TEMPL_INST_DECL_API ${base_name}_API\n"
    "#endif\n"
    "\n"
    "#ifdef _MSC_VER\n"
    "  #define ${base_name}_TEMPL_INST_DEF_API ${base_name}_API\n"
    "#else\n"
    "  #define ${base_name}_TEMPL_INST_DEF_API\n"
    "#endif\n"
    "\n"
    "#include \"detail/${_ARG_EXPORT_FILE_NAME}\"\n"
  )

  # Install generated export files
  set(include_base_path ${CMAKE_INSTALL_INCLUDEDIR}/${org_name}/${project_name}${project_version_major})
  set(export_install_path "${include_base_path}/${_ARG_BASE_DIR}")
  set(detail_export_install_path "${export_install_path}/detail/")
  install(FILES "${export_file_path}"
    DESTINATION "${export_install_path}"
  )
  install(FILES "${export_detail_file_path}"
    DESTINATION "${detail_export_install_path}"
  )
endfunction()


#===============================================================================
# Generate header file.
# Usage:
#   dart_generate_meta_header(file_path target_dir [headers...])
#===============================================================================
function(dart_generate_meta_header file_path target_dir)
  file(WRITE ${file_path} "// Automatically generated file by cmake\n\n")
  foreach(header ${ARGN})
    file(APPEND ${file_path} "#include \<${target_dir}${header}\>\n")
  endforeach()
endfunction()

#===============================================================================
function(dart_generate_meta_header_from_abs_paths output_filepath base_path)
  file(WRITE ${output_filepath} "// Automatically generated file by cmake\n\n")
  foreach(header ${ARGN})
    file(RELATIVE_PATH path_rel ${base_path}/ ${header})
    file(APPEND ${output_filepath} "#include \<${path_rel}\>\n")
  endforeach()
endfunction()

#===============================================================================
# Add library and set target properties
# Usage:
#   dart_add_library(_libname source1 [source2 ...])
#===============================================================================
macro(dart_find_package _name)
  include(DARTFind${_name})
endmacro()

#===============================================================================
# Add library and set target properties
# Usage:
#   dart_add_library(_libname source1 [source2 ...])
#===============================================================================
macro(dart_add_library _name)
  add_library(${_name} ${ARGN})
  set_target_properties(
    ${_name} PROPERTIES
    SOVERSION "${DART_MAJOR_VERSION}.${DART_MINOR_VERSION}"
    VERSION "${DART_VERSION}"
  )
endmacro()

#===============================================================================
function(dart_property_add property_name)

  get_property(is_defined GLOBAL PROPERTY ${property_name} DEFINED)

  if(NOT is_defined)
    define_property(GLOBAL PROPERTY ${property_name}
      BRIEF_DOCS "${property_name}"
      FULL_DOCS "Global properties for ${property_name}"
    )
  endif()

  foreach(item ${ARGN})
    set_property(GLOBAL APPEND PROPERTY ${property_name} "${item}")
  endforeach()

endfunction()

#===============================================================================
function(dart_check_required_package variable dependency)
  # TODO: Take version for the case that the version variable is not
  # <package>_VERSION
  if(${${variable}_FOUND})
    if(DART_VERBOSE)
      message(STATUS "Looking for ${dependency} - version ${${variable}_VERSION}"
                     " found")
    endif()
  endif()
endfunction()

#===============================================================================
macro(dart_check_optional_package variable component dependency)
  # Use upper case for the package names
  string(TOUPPER ${variable} variable_upper)

  option(DART_SKIP_${variable_upper} "If ON, do not use ${variable_upper} even if it is found." OFF)
  mark_as_advanced(DART_SKIP_${variable_upper})
  if(${${variable}_FOUND} AND NOT ${DART_SKIP_${variable_upper}})
    set(DART_HAS_${variable_upper} TRUE CACHE BOOL "Check if ${variable} found." FORCE)
    if(DART_VERBOSE)
      message(STATUS "Building ${component} with ${dependency} ${${variable}_VERSION}")
    endif()
  else()
    set(DART_HAS_${variable_upper} FALSE CACHE BOOL "Check if ${variable_upper} found." FORCE)
    if(NOT ${${variable}_FOUND})
      if(DART_VERBOSE)
        message(WARNING "Skipping building ${component} with ${dependency} since it's not found")
      else()
        message(STATUS "Skipping building ${component} with ${dependency} since it's not found")
      endif()
    elseif(${DART_SKIP_${variable_upper}} AND DART_VERBOSE)
      message(STATUS "Skipping building ${component} with ${dependency} ${${variable}_VERSION} since DART_SKIP_${variable_upper} is ON.")
    endif()
    return()
  endif()
endmacro()

#===============================================================================
macro(dart_check_dependent_target target)
  foreach(dependent_target ${ARGN})
    if (NOT TARGET ${dependent_target})
      message(WARNING "${target} is disabled because dependent target ${dependent_target} is not being built.")
      return()
    endif()
  endforeach()
endmacro()

#===============================================================================
function(dart_add_custom_target rel_dir property_name)
  set(abs_dir "${CMAKE_CURRENT_LIST_DIR}/${rel_dir}")

  if(NOT IS_DIRECTORY ${abs_dir})
    message(SEND_ERROR "Failed to find directory: ${abs_dir}")
  endif()

  # Use the directory name as the executable name
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
function(dart_add_example)
  dart_property_add(DART_EXAMPLES ${ARGN})
endfunction(dart_add_example)

#===============================================================================
function(dart_add_tutorial)
  dart_property_add(DART_TUTORIALS ${ARGN})
endfunction(dart_add_tutorial)

#===============================================================================
function(dart_format_add)
  foreach(source ${ARGN})
    if(IS_ABSOLUTE "${source}")
      set(source_abs "${source}")
    else()
      get_filename_component(source_abs
        "${CMAKE_CURRENT_LIST_DIR}/${source}" ABSOLUTE)
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
# dart_build_target_in_source(target
#   [LINK_LIBRARIES library1 ...])
#   [COMPILE_FEATURES feature1 ...]
#   [COMPILE_OPTIONS option1 ...]
# )
function(dart_build_target_in_source target)
  set(prefix example)
  set(options )
  set(oneValueArgs )
  set(multiValueArgs LINK_LIBRARIES COMPILE_FEATURES COMPILE_OPTIONS)
  cmake_parse_arguments("${prefix}" "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  if(example_LINK_LIBRARIES)
    foreach(dep_target ${example_LINK_LIBRARIES})
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

  if(example_LINK_LIBRARIES)
    foreach(dep_target ${example_LINK_LIBRARIES})
      target_link_libraries(${target} ${dep_target})
    endforeach()
  endif()

  if(example_COMPILE_FEATURES)
    foreach(comple_feature ${example_COMPILE_FEATURES})
      target_compile_features(${target} PUBLIC ${comple_feature})
    endforeach()
  endif()

  if(example_COMPILE_OPTIONS)
    foreach(comple_option ${example_COMPILE_OPTIONS})
      target_compile_options(${target} PUBLIC ${comple_option})
    endforeach()
  endif()

  set_target_properties(${target}
    PROPERTIES
      RUNTIME_OUTPUT_DIRECTORY "${DART_BINARY_DIR}/bin"
  )

  dart_format_add(${srcs})
endfunction()

#===============================================================================
# dart_build_example_in_source(target
#   [LINK_LIBRARIES library1 ...])
#   [COMPILE_FEATURES feature1 ...]
#   [COMPILE_OPTIONS option1 ...]
# )
function(dart_build_example_in_source target)
  dart_build_target_in_source(${target} ${ARGN})
  dart_add_example(${target})
endfunction()

#===============================================================================
# dart_build_tutorial_in_source(target
#   [LINK_LIBRARIES library1 ...])
#   [COMPILE_FEATURES feature1 ...]
#   [COMPILE_OPTIONS option1 ...]
# )
function(dart_build_tutorial_in_source target)
  dart_build_target_in_source(${target} ${ARGN})
  dart_add_tutorial(${target})
endfunction()

#===============================================================================
function(dart_add_project)
  set(prefix dart_add_project)
  set(options
    GENERATE_META_HEADER
  )
  set(oneValueArgs
    PROJECT_NAME
    PROJECT_VERSION_MAJOR
  )
  set(multiValueArgs
    COMPONENTS
  )
  cmake_parse_arguments(
    "${prefix}" "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN}
  )

  # Shorter variable names for readability
  set(org_name dartsim)
  set(project_name ${${prefix}_PROJECT_NAME})
  set(project_version_major ${${prefix}_PROJECT_VERSION_MAJOR})
  set(components ${${prefix}_COMPONENTS})

  # Include path
  set(include_base_path include/${org_name}/${project_name}${project_version_major})
  set(include_path ${include_base_path}/${project_name})

  # Add components
  foreach(component ${components})
    add_subdirectory(${component})
  endforeach()

  # Generate project meta header
  set(component_meta_headers)
  # TODO(JS): Improve the way of adding config.hpp
  list(APPEND component_meta_headers config.hpp)
  get_property(enabled_components GLOBAL PROPERTY ${PROJECT_NAME}_COMPONENTS)
  # TODO(JS): Remove this
  list(REMOVE_ITEM enabled_components external-imgui)
  foreach(component ${enabled_components})
    list(APPEND component_meta_headers "${component}/${component}.hpp")
  endforeach()
  if(${prefix}_GENERATE_META_HEADER)
    dart_get_filename_components(header_names "${project_name} headers" ${hdrs})
    dart_generate_meta_header(
      "${CMAKE_CURRENT_BINARY_DIR}/${project_name}.hpp"
      "${project_name}/"
      ${component_meta_headers}
    )
    install(
      FILES ${CMAKE_CURRENT_BINARY_DIR}/${project_name}.hpp
      DESTINATION ${include_path}
      COMPONENT headers
    )
  endif()
endfunction()

#===============================================================================
function(dart_add_component)
  set(prefix _ARG)
  set(options
    GENERATE_EXPORT_HEADER
    GENERATE_META_HEADER
    FORMAT_CODE
  )
  set(oneValueArgs
    COMPONENT_NAME
    PROJECT_NAME
    PROJECT_VERSION_MAJOR
    PROJECT_SOURCE_DIR
    PROJECT_BINARY_DIR
  )
  set(multiValueArgs
    DEPENDENT_COMPONENTS
    DEPENDENT_PACKAGES_REQUIRED
    DEPENDENT_PACKAGES_OPTIONAL
    TARGET_LINK_LIBRARIES_PUBLIC
    TARGET_LINK_LIBRARIES_PUBLIC_SKIP_CHECKING
    TARGET_LINK_LIBRARIES_PUBLIC_OPTIONAL
    TARGET_LINK_LIBRARIES_PRIVATE
    TARGET_LINK_OPTIONS_PUBLIC
    TARGET_COMPILE_FEATURES_PUBLIC
    TARGET_COMPILE_OPTIONS_PUBLIC
    TARGET_COMPILE_OPTIONS_PRIVATE
    TARGET_COMPILE_DEFINITIONS_PUBLIC
    SUB_DIRECTORIES
  )
  cmake_parse_arguments(
    "${prefix}" "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN}
  )

  # Shorter variable names for readability
  set(project_source_dir ${_ARG_PROJECT_SOURCE_DIR})
  set(project_binary_dir ${_ARG_PROJECT_BINARY_DIR})
  set(component_name ${_ARG_COMPONENT_NAME})
  set(org_name dartsim)
  set(project_name ${_ARG_PROJECT_NAME})
  set(project_version_major ${_ARG_PROJECT_VERSION_MAJOR})
  set(link_libraries_public ${_ARG_TARGET_LINK_LIBRARIES_PUBLIC})
  set(link_libraries_public_skip_checking ${_ARG_TARGET_LINK_LIBRARIES_PUBLIC_SKIP_CHECKING})
  set(link_libraries_public_optional ${_ARG_TARGET_LINK_LIBRARIES_PUBLIC_OPTIONAL})
  set(link_libraries_private ${_ARG_TARGET_LINK_LIBRARIES_PRIVATE})
  set(link_options_public ${_ARG_TARGET_LINK_OPTIONS_PUBLIC})
  set(compile_features_public ${_ARG_TARGET_COMPILE_FEATURES_PUBLIC})
  set(compile_options_public ${_ARG_TARGET_COMPILE_OPTIONS_PUBLIC})
  set(compile_options_private ${_ARG_TARGET_COMPILE_OPTIONS_PRIVATE})
  set(dependent_components ${_ARG_DEPENDENT_COMPONENTS})
  set(sub_directories ${_ARG_SUB_DIRECTORIES})

  # Check if the component is enabled
  if(NOT DART_BUILD_COMP_${component_name})
    message(WARNING "Skipping component <${component_name}> because DART_BUILD_COMP_${component_name} is not set or set to OFF")
    return()
  endif()

  # Check component dependencies
  get_property(enabled_components GLOBAL PROPERTY ${PROJECT_NAME}_COMPONENTS)
  foreach(dep_comp ${dependent_components})
    if(NOT ${dep_comp} IN_LIST enabled_components)
      message(WARNING "Skipping component <${component_name}> because of missing component <${dep_comp}>")
      return()
    endif()
  endforeach()
  foreach(dep ${link_libraries_public})
    if(NOT TARGET ${dep})
      message(WARNING "Skipping component <${component_name}> because of missing library <${dep}>")
      return()
    endif()
  endforeach()
  foreach(dep ${link_libraries_private})
    if(NOT TARGET ${dep})
      message(WARNING "Skipping component <${component_name}> because of missing library <${dep}>")
      return()
    endif()
  endforeach()

  # Check required dependencies
  foreach(package ${_ARG_DEPENDENT_PACKAGES_REQUIRED})
    string(TOUPPER ${package} package_upper)
    if(${package}_FOUND OR ${package_upper}_FOUND)
      if(NOT DART_SKIP_${package_upper})
        list(APPEND _ARG_DEPENDENT_PACKAGES_REQUIRED ${package})
      else()
        message("[WARN] Skipped component [${_ARG_COMPONENT_NAME}] as [DART_SKIP_${package_upper}=ON] is set")
        return()
      endif()
    else()
      message("[WARN] Skipped component [${_ARG_COMPONENT_NAME}] due to missing [${package}]")
      return()
    endif()
  endforeach()

  # Check optional dependencies
  foreach(package ${_ARG_DEPENDENT_PACKAGES_OPTIONAL})
    string(TOUPPER ${package} package_upper)
    if(${package}_FOUND)
      if(NOT DART_SKIP_${package_upper})
        list(APPEND _ARG_DEPENDENT_PACKAGES_REQUIRED ${package})
        list(APPEND _ARG_TARGET_COMPILE_DEFINITIONS_PUBLIC -DDART_HAS_${package_upper}=1)
      else()
        list(APPEND _ARG_TARGET_COMPILE_DEFINITIONS_PUBLIC -DDART_HAS_${package_upper}=0)
        message("[WARN] Building component [${_ARG_COMPONENT_NAME}] without [${package}] as [DART_SKIP_${package_upper}=ON] is set")
      endif()
    else()
      list(APPEND _ARG_TARGET_COMPILE_DEFINITIONS_PUBLIC -DDART_HAS_${package_upper}=0)
      message("[WARN] Building component [${_ARG_COMPONENT_NAME}] without [${package}] as it's not found")
    endif()
  endforeach()

  # Target name
  set(target_base ${project_name}${project_version_major})
  set(target_name ${target_base}-${component_name})

  # Include path
  set(include_base_path include/${org_name}/${project_name}${project_version_major})
  set(include_path ${include_base_path}/${project_name})

  # Current paths
  set(current_src_source_dir ${CMAKE_CURRENT_SOURCE_DIR})
  set(current_src_binary_dir ${CMAKE_CURRENT_BINARY_DIR})
  file(RELATIVE_PATH relative_src_source_dir "${PROJECT_SOURCE_DIR}" "${current_src_source_dir}")
  set(current_include_source_dir ${PROJECT_SOURCE_DIR}/${relative_src_source_dir})  # TODO: make "dart" configurable
  set(current_include_binary_dir ${PROJECT_BINARY_DIR}/${relative_src_source_dir})
  file(RELATIVE_PATH relative_include_source_dir "${PROJECT_SOURCE_DIR}" "${current_include_source_dir}")

  # Set context properties
  set_property(GLOBAL PROPERTY _DART_CURRENT_COMPONENT_HEADERS "")
  set_property(GLOBAL PROPERTY _DART_CURRENT_COMPONENT_SOURCES "")
  set_property(GLOBAL PROPERTY _DART_CURRENT_COMPONENT_BASE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
  set_property(GLOBAL PROPERTY _DART_CURRENT_COMPONENT_INCLUDE_BASE_PATH ${include_base_path})
  set_property(GLOBAL PROPERTY _DART_CURRENT_COMPONENT_INCLUDE_PATH ${include_path})
  set_property(GLOBAL PROPERTY _DART_CURRENT_DEPENDENT_PACKAGES_REQUIRED "")
  set_property(GLOBAL PROPERTY _DART_CURRENT_TARGET_NAME ${target_name})
  set_property(GLOBAL PROPERTY _DART_CURRENT_TARGET_LINK_LIBRARIES_PUBLIC "")
  set_property(GLOBAL PROPERTY _DART_CURRENT_TARGET_LINK_LIBRARIES_PUBLIC_SKIP_CHECKING "")
  set_property(GLOBAL PROPERTY _DART_CURRENT_TARGET_LINK_LIBRARIES_PRIVATE "")
  set_property(GLOBAL PROPERTY _DART_CURRENT_TARGET_COMPILE_DEFINITIONS_PUBLIC "")
  set_property(GLOBAL PROPERTY _DART_CURRENT_TARGET_COMPILE_OPTIONS_PUBLIC "")

  # Add library
  add_library(${target_name})

  # Collect sources
  file(GLOB headers "*.hpp")
  file(GLOB sources "*.cpp")

  # Collect sources of sub-directories
  foreach(sub_directory ${sub_directories})
    add_subdirectory(${sub_directory})
  endforeach()
  get_property(current_component_headers                          GLOBAL PROPERTY _DART_CURRENT_COMPONENT_HEADERS)
  get_property(current_component_sources                          GLOBAL PROPERTY _DART_CURRENT_COMPONENT_SOURCES)
  get_property(current_component_dependency_packages              GLOBAL PROPERTY _DART_CURRENT_DEPENDENT_PACKAGES_REQUIRED)
  get_property(current_target_link_libraries_public               GLOBAL PROPERTY _DART_CURRENT_TARGET_LINK_LIBRARIES_PUBLIC)
  get_property(current_target_link_libraries_public_skip_checking GLOBAL PROPERTY _DART_CURRENT_TARGET_LINK_LIBRARIES_PUBLIC_SKIP_CHECKING)
  get_property(current_target_link_libraries_private              GLOBAL PROPERTY _DART_CURRENT_TARGET_LINK_LIBRARIES_PRIVATE)
  get_property(current_target_compile_definitions_public          GLOBAL PROPERTY _DART_CURRENT_TARGET_COMPILE_DEFINITIONS_PUBLIC)
  get_property(current_target_compile_options_public              GLOBAL PROPERTY _DART_CURRENT_TARGET_COMPILE_OPTIONS_PUBLIC)

  # Add sources
  target_sources(${target_name}
    PRIVATE
      ${headers}
      ${sources}
      ${current_component_headers}
      ${current_component_sources}
  )

  # Set include directory
  target_include_directories(
    ${target_name}
    PUBLIC
      $<BUILD_INTERFACE:${project_source_dir}>
      $<BUILD_INTERFACE:${project_binary_dir}>
      $<INSTALL_INTERFACE:${include_base_path}>
  )

  # Set link libraries
  foreach(dependent_component ${dependent_components})
    set(dependent_component_target_name ${target_base}-${dependent_component})
    target_link_libraries(${target_name} PUBLIC ${dependent_component_target_name})
  endforeach()
  target_link_libraries(${target_name} PUBLIC ${link_libraries_public})
  target_link_libraries(${target_name} PUBLIC ${current_target_link_libraries_public})
  target_link_libraries(${target_name} PUBLIC ${link_libraries_public_skip_checking})
  target_link_libraries(${target_name} PUBLIC ${current_target_link_libraries_public_skip_checking})
  target_link_libraries(${target_name} PRIVATE ${link_libraries_private})
  target_link_libraries(${target_name} PRIVATE ${current_target_link_libraries_private})

  # Set link options
  target_link_options(${target_name} PUBLIC ${link_options})

  # Set compile features
  target_compile_features(${target_name} PUBLIC ${compile_features_public})

  # Set compile options
  target_compile_options(${target_name} PUBLIC ${compile_options_public})
  target_compile_options(${target_name} PUBLIC ${current_target_compile_options_public})
  target_compile_options(${target_name} PRIVATE ${compile_options_private})

  # Set compile definitions
  target_compile_definitions(
    ${target_name} PUBLIC ${_ARG_TARGET_COMPILE_DEFINITIONS_PUBLIC}
  )
  target_compile_definitions(
    ${target_name} PUBLIC ${current_target_compile_definitions_public}
  )

  # Format files
  if(_ARG_FORMAT_CODE)
    dart_format_add(${headers} ${sources})
  endif()

  # Component settings
  add_component(${project_name} ${component_name})
  add_component_targets(${project_name} ${component_name} ${target_name})
  add_component_dependencies(${project_name} ${component_name}
    ${dependent_components}
  )
  add_component_dependency_packages(
    ${project_name}
    ${component_name}
    ${_ARG_DEPENDENT_PACKAGES_REQUIRED} ${current_component_dependency_packages}
  )

  # Generate export header for the component
  if (_ARG_GENERATE_EXPORT_HEADER)
    dart_generate_export_header(
      TARGET_NAME ${target_name}
      DESTINATION ${current_include_binary_dir}
      EXPORT_FILE_NAME Export.hpp
      BASE_NAME DART_${_ARG_COMPONENT_NAME}
      BASE_DIR ${relative_include_source_dir}
    )
  endif()

  # Generate a meta header for the component
  if(_ARG_GENERATE_META_HEADER)
    dart_generate_meta_header_from_abs_paths(
      "${CMAKE_CURRENT_BINARY_DIR}/${component_name}.hpp"
      ${project_source_dir}
      ${headers}
      ${current_component_headers}
    )
    install(
      FILES ${CMAKE_CURRENT_BINARY_DIR}/${component_name}.hpp
      DESTINATION ${include_path}/${component_name}
      COMPONENT headers
    )
  endif()

  # Install headers
  install(
    FILES ${headers}
    DESTINATION ${include_path}/${component_name}
    COMPONENT headers
  )

endfunction()

#===============================================================================
function(dart_add_component_sub_directory)
  set(prefix _ARG)
  set(options
    DISABLE_FORMAT
  )
  set(oneValueArgs
  )
  set(multiValueArgs
    TARGET_LINK_LIBRARIES_PUBLIC
    TARGET_LINK_LIBRARIES_PUBLIC_SKIP_CHECKING
    TARGET_LINK_LIBRARIES_PRIVATE
    DEPENDENT_PACKAGES_REQUIRED
    TARGET_COMPILE_DEFINITIONS_PUBLIC
    TARGET_COMPILE_OPTIONS_PUBLIC
    SUB_DIRECTORIES
  )
  cmake_parse_arguments(
    "${prefix}" "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN}
  )

  # Shorter variable names for readability
  set(link_libraries_public ${_ARG_TARGET_LINK_LIBRARIES_PUBLIC})
  set(link_libraries_private ${_ARG_TARGET_LINK_LIBRARIES_PRIVATE})
  set(dependent_packages ${_ARG_DEPENDENT_PACKAGES_REQUIRED})
  set(target_compile_definitions_public ${_ARG_TARGET_COMPILE_DEFINITIONS_PUBLIC})
  set(target_compile_options_public ${_ARG_TARGET_COMPILE_OPTIONS_PUBLIC})
  set(sub_directories ${_ARG_SUB_DIRECTORIES})

  # Context variables
  get_property(component_base_path    GLOBAL PROPERTY _DART_CURRENT_COMPONENT_BASE_PATH)
  get_property(include_base_path      GLOBAL PROPERTY _DART_CURRENT_COMPONENT_INCLUDE_BASE_PATH)
  get_property(include_path           GLOBAL PROPERTY _DART_CURRENT_COMPONENT_INCLUDE_PATH)
  get_property(current_target_name    GLOBAL PROPERTY _DART_CURRENT_TARGET_NAME)

  # Check dependencies
  foreach(package ${dependent_packages})
    string(TOUPPER ${package} package_upper)
    dart_check_optional_package(${package} ${current_target_name} ${package})
    if(NOT DART_HAS_${package_upper})
      message(STATUS "Skipping <todo> because of missing dependency package ${package}")
      return()
    endif()
    list(APPEND target_compile_definitions_public -DDART_HAS_${package_upper}=1)
  endforeach()

  # Add sub-directories
  foreach(sub_directory ${sub_directories})
    add_subdirectory(${sub_directory})
  endforeach()

  # Paths
  file(RELATIVE_PATH component_rel_path ${component_base_path} ${CMAKE_CURRENT_SOURCE_DIR})

  # Add sources
  file(GLOB headers "*.hpp")
  file(GLOB sources "*.cpp")
  set_property(GLOBAL APPEND PROPERTY _DART_CURRENT_COMPONENT_HEADERS ${headers})
  set_property(GLOBAL APPEND PROPERTY _DART_CURRENT_COMPONENT_SOURCES ${sources})

  # Set link libraries
  set_property(GLOBAL APPEND PROPERTY _DART_CURRENT_TARGET_LINK_LIBRARIES_PUBLIC ${link_libraries_public})
  set_property(GLOBAL APPEND PROPERTY _DART_CURRENT_TARGET_LINK_LIBRARIES_PUBLIC_SKIP_CHECKING ${_ARG_TARGET_LINK_LIBRARIES_PUBLIC_SKIP_CHECKING})
  set_property(GLOBAL APPEND PROPERTY _DART_CURRENT_TARGET_LINK_LIBRARIES_PRIVATE ${link_libraries_private})

  # Set compile definitions
  set_property(GLOBAL APPEND PROPERTY _DART_CURRENT_TARGET_COMPILE_DEFINITIONS_PUBLIC ${target_compile_definitions_public})

  # Set compile options
  set_property(GLOBAL APPEND PROPERTY _DART_CURRENT_TARGET_COMPILE_OPTIONS_PUBLIC ${target_compile_options_public})

  # Component settings
  set_property(GLOBAL APPEND PROPERTY _DART_CURRENT_DEPENDENT_PACKAGES_REQUIRED ${dependent_packages})

  # Install headers
  install(
    FILES ${headers}
    DESTINATION ${include_path}/${component_name}/${component_rel_path}
    COMPONENT headers
  )

  # Format files
  if(NOT _ARG_DISABLE_FORMAT)
    dart_format_add(${headers} ${sources})
  endif()
endfunction()

# ==============================================================================
# cmake-format: off
# dart_build_tests(
#   TYPE <test_type>
#   SOURCES <sources>
#   [INCLUDE_DIRS <include_dependencies>]
#   [LINK_LIBRARIES <library_dependencies>]
#   [LINK_DART_LIBRARIES <library_dependencies>]
#   [TEST_LIST <output_var>]
# )
#
# Build multiple tests. Arguments are as follows:
#
# - TYPE           : Required. Preferably UNIT or INTEGRATION.
#
# - TARGET_PREFIX  : Optional. Prefix of the target name.
#
# - SOURCES        : Required. The list of source files for your tests. Each file
#                    will turn into a test.
#
# - INCLUDE_DIRS   : Optional. Additional include directories that should be
#                    visible to all the tests.
#
# - LINK_LIBRARIES : Optional. Additional library dependencies that every test
#                    should link to including the library built by this project.
#                    'gtest' and 'gtest_main' will be automatically linked.
#
# - LINK_DART_LIBRARIES:
#                    Optional. DART library dependencies.
#
# - COMPILE_DEFINITIONS:
#
# - TEST_LIST      : Optional. Provide a variable which will be given the list of the
#                    target names of the tests generated by this function.
# cmake-format: on
function(dart_build_tests)
  set(prefix dart_build_tests)
  set(options)
  set(oneValueArgs TYPE TARGET_PREFIX TEST_LIST)
  set(multiValueArgs SOURCES INCLUDE_DIRS LINK_LIBRARIES LINK_DART_LIBRARIES
                     COMPILE_DEFINITIONS
  )
  cmake_parse_arguments(
    "${prefix}" "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN}
  )

  if(NOT dart_build_tests_TYPE)
    message(
      FATAL_ERROR "DEVELOPER ERROR: You must specify a TYPE for your tests!"
    )
  endif()

  set(test_type ${dart_build_tests_TYPE})

  if(NOT DEFINED dart_build_tests_SOURCES)
    message(STATUS "No tests have been specified for ${test_type}")
  else()
    list(LENGTH dart_build_tests_SOURCES num_tests)
  endif()

  if(dart_build_tests_TEST_LIST)
    set(${dart_build_tests_TEST_LIST} "")
  endif()

  foreach(source ${dart_build_tests_SOURCES})

    # Set target name: <TYPE>[_<TARGET_PREFIX>]_<source>
    set(target_name ${test_type})
    if(dart_build_tests_TARGET_PREFIX)
      set(target_name "${target_name}_${dart_build_tests_TARGET_PREFIX}")
    endif()
    get_filename_component(source_name ${source} NAME_WE)
    string(REPLACE "test_" "" source_name ${source_name})
    get_filename_component(source_dir ${source} DIRECTORY)
    if(source_dir)
      string(REPLACE "/" "_" source_prefix ${source_dir})
      set(target_name "${target_name}_${source_prefix}_${source_name}")
    else()
      set(target_name "${target_name}_${source_name}")
    endif()

    add_executable(${target_name} ${source})
    add_test(NAME ${target_name} COMMAND $<TARGET_FILE:${target_name}>)
    target_include_directories(
      ${target_name} PRIVATE ${dart_build_tests_INCLUDE_DIRS}
    )

    target_link_libraries(${target_name} PRIVATE gtest gtest_main)

    if(UNIX)
      # gtest requies pthread when compiled on a Unix machine
      target_link_libraries(${target_name} PRIVATE pthread)
    endif()

    target_link_libraries(
      ${target_name} PRIVATE ${dart_build_tests_LINK_LIBRARIES}
    )

    if(dart_build_tests_COMPILE_DEFINITIONS)
      target_compile_definitions(
        ${target_name} PRIVATE ${dart_build_tests_COMPILE_DEFINITIONS}
      )
    endif()

    foreach(dart_lib ${dart_build_tests_LINK_DART_LIBRARIES})
      if(NOT TARGET ${dart_lib})
        message(FATAL_ERROR "Invalid target: ${dart_lib}")
      endif()
      target_link_libraries(${target_name} PRIVATE ${dart_lib})
    endforeach()

    set_target_properties (${target_name} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${DART_BINARY_DIR}/bin)

    if(dart_build_tests_TEST_LIST)
      list(APPEND ${dart_build_tests_TEST_LIST} ${target_name})
    endif()

    # Add executable target
    add_custom_target(
      RUN_${target_name}
      COMMAND ${target_name}
      COMMENT "Running ${target_name}..."
      VERBATIM
    )

    dart_property_add(DART_${test_type}_TESTS ${target_name})
  endforeach()

  if(dart_build_tests_TEST_LIST)
    set(${dart_build_tests_TEST_LIST} "${${dart_build_tests_TEST_LIST}}"
        PARENT_SCOPE
    )
  endif()

  dart_format_add(${dart_build_tests_SOURCES})

endfunction()


# ==============================================================================
# cmake-format: off
# dart_build_benchmarks(
#   TYPE <benchmark_type>
#   SOURCES <sources>
#   [INCLUDE_DIRS <include_dependencies>]
#   [LINK_LIBRARIES <library_dependencies>]
#   [LINK_DART_LIBRARIES <library_dependencies>]
#   [TEST_LIST <output_var>]
# )
#
# Build multiple benchmarks. Arguments are as follows:
#
# - TYPE           : Required. Preferably UNIT or INTEGRATION.
#
# - TARGET_PREFIX  : Optional. Prefix of the target name.
#
# - SOURCES        : Required. The list of source files for your benchmarks. Each file
#                    will turn into a benchmark.
#
# - INCLUDE_DIRS   : Optional. Additional include directories that should be
#                    visible to all the benchmarks.
#
# - LINK_LIBRARIES : Optional. Additional library dependencies that every benchmark
#                    should link to including the library built by this project.
#                    'benchmark' and 'benchmark_main' will be automatically linked.
#
# - LINK_DART_LIBRARIES:
#                    Optional. DART library dependencies.
#
# - COMPILE_DEFINITIONS:
#
# - TEST_LIST      : Optional. Provide a variable which will be given the list of the
#                    target names of the benchmarks generated by this function.
# cmake-format: on
function(dart_build_benchmarks)
  set(prefix _)
  set(options)
  set(oneValueArgs TYPE TARGET_PREFIX TEST_LIST)
  set(multiValueArgs SOURCES INCLUDE_DIRS LINK_LIBRARIES LINK_DART_LIBRARIES
                     COMPILE_DEFINITIONS
  )
  cmake_parse_arguments(
    "${prefix}" "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN}
  )

  if(NOT __TYPE)
    message(
      FATAL_ERROR "DEVELOPER ERROR: You must specify a TYPE for your benchmarks!"
    )
  endif()

  set(benchmark_type ${__TYPE})

  if(NOT DEFINED __SOURCES)
    message(STATUS "No benchmarks have been specified for ${benchmark_type}")
  else()
    list(LENGTH __SOURCES num_benchmarks)
    message(STATUS "Adding ${num_benchmarks} ${benchmark_type} benchmarks")
  endif()

  if(__TEST_LIST)
    set(${__TEST_LIST} "")
  endif()

  foreach(source ${__SOURCES})

    # Set target name: <TYPE>[_<TARGET_PREFIX>]_<source>
    set(target_name ${benchmark_type})
    if(__TARGET_PREFIX)
      set(target_name "${target_name}_${__TARGET_PREFIX}")
    endif()
    get_filename_component(source_name ${source} NAME_WE)
    string(REPLACE "benchmark_" "" source_name ${source_name})
    get_filename_component(source_dir ${source} DIRECTORY)
    if(source_dir)
      string(REPLACE "/" "_" source_prefix ${source_dir})
      set(target_name "${target_name}_${source_prefix}_${source_name}")
    else()
      set(target_name "${target_name}_${source_name}")
    endif()

    add_executable(${target_name} ${source})
    target_include_directories(
      ${target_name} PRIVATE ${__INCLUDE_DIRS}
    )

    target_link_libraries(${target_name} PRIVATE benchmark benchmark_main)

    if(UNIX)
      # gbenchmark requies pthread when compiled on a Unix machine
      target_link_libraries(${target_name} PRIVATE pthread)
    endif()

    target_link_libraries(
      ${target_name} PRIVATE ${__LINK_LIBRARIES}
    )

    if(__COMPILE_DEFINITIONS)
      target_compile_definitions(
        ${target_name} PRIVATE ${__COMPILE_DEFINITIONS}
      )
    endif()

    foreach(dart_lib ${__LINK_DART_LIBRARIES})
      if(NOT TARGET ${dart_lib})
        message(FATAL_ERROR "Invalid target: ${dart_lib}")
      endif()
      target_link_libraries(${target_name} PRIVATE ${dart_lib})
    endforeach()

    if(__TEST_LIST)
      list(APPEND ${__TEST_LIST} ${target_name})
    endif()

    dart_property_add(DART_${benchmark_type}_BENCHMARKS ${target_name})
  endforeach()

  if(__TEST_LIST)
    set(${__TEST_LIST} "${${__TEST_LIST}}"
        PARENT_SCOPE
    )
  endif()

  dart_format_add(${__SOURCES})

endfunction()

# ==============================================================================
macro(dart_fetch_git_repo)
  set(prefix _ARG)
  set(options
  )
  set(oneValueArgs
    PROJECT_NAME
    WORKING_DIR
    GIT_URL
    GIT_TAG
  )
  set(multiValueArgs
  )
  cmake_parse_arguments(
    "${prefix}" "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN}
  )

  get_property(
    project_base_dir GLOBAL PROPERTY DART_GLOBAL_PROPERTY_PROJECT_BASE_DIR
  )

  set(${_ARG_PROJECT_NAME}_SOURCE_DIR ${_ARG_WORKING_DIR}/${_ARG_PROJECT_NAME}-src)
  set(${_ARG_PROJECT_NAME}_BINARY_DIR ${_ARG_WORKING_DIR}/${_ARG_PROJECT_NAME}-build)

  # Variables used configuring dart_fetch_git_repo_sub.cmake.in
  set(FETCH_PROJECT_NAME ${_ARG_PROJECT_NAME})
  set(FETCH_SOURCE_DIR ${${_ARG_PROJECT_NAME}_SOURCE_DIR})
  set(FETCH_BINARY_DIR ${${_ARG_PROJECT_NAME}_BINARY_DIR})
  set(FETCH_GIT_REPOSITORY ${_ARG_GIT_URL})
  set(FETCH_GIT_TAG ${_ARG_GIT_TAG})

  configure_file(
    ${DART_SOURCE_DIR}/cmake/dart_fetch_at_configure_step.cmake.in
    ${_ARG_WORKING_DIR}/${_ARG_PROJECT_NAME}/CMakeLists.txt
    @ONLY
  )

  # Unset them again
  unset(FETCH_PROJECT_NAME)
  unset(FETCH_SOURCE_DIR)
  unset(FETCH_BINARY_DIR)
  unset(FETCH_GIT_REPOSITORY)
  unset(FETCH_GIT_TAG)

  # Configure sub-project
  execute_process(
    COMMAND "${CMAKE_COMMAND}" -G "${CMAKE_GENERATOR}" .
    WORKING_DIRECTORY ${_ARG_WORKING_DIR}/${_ARG_PROJECT_NAME}
  )

  # Build sub-project which triggers ExternalProject_Add
  execute_process(
    COMMAND "${CMAKE_COMMAND}" --build .
    WORKING_DIRECTORY ${_ARG_WORKING_DIR}/${_ARG_PROJECT_NAME}
  )
endmacro()

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
    set(_ARG_INSTALL_DIR ${CMAKE_INSTALL_PREFIXX}/${CMAKE_INSTALL_DOXDIR}/${PROJECT_NAME}${DART_VERSION_MAJOR}/coverage)
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
    if(_ARG_VERBOSE)
      message(STATUS "Failed to find ${open_command_name}, to enable "
        "'coverage_view', please install xdg-utils"
      )
    endif()
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

#=========================================================================================
function(dart_build_dartpy_submodule)
  set(prefix _ARG)
  set(options)
  set(oneValueArgs
    TARGET_NAME
  )
  set(multiValueArgs
    SOURCES
    TARGET_LINK_LIBRARIES
  )
  cmake_parse_arguments(
    "${prefix}" "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN}
  )

  if(NOT DEFINED _ARG_TARGET_NAME)
    message(FATAL_ERROR "TARGET_NAME not provided for dartpy submodule")
  endif()

  if(NOT DEFINED _ARG_SOURCES)
    message(FATAL_ERROR "SOURCES not provided for ${_ARG_TARGET_NAME}")
  endif()

  foreach(lib ${_ARG_TARGET_LINK_LIBRARIES})
    if(NOT TARGET ${lib})
      message(WARNING "Skipping <${_ARG_TARGET_NAME}> because of missing target <${lib}>")
      return()
    endif()
  endforeach()

  pybind11_add_module(${_ARG_TARGET_NAME} MODULE ${_ARG_SOURCES})

  target_compile_definitions(${_ARG_TARGET_NAME}
    PRIVATE DARTPY_VERSION_INFO="${DARTPY_VERSION_INFO}"
  )

  target_include_directories(${_ARG_TARGET_NAME}
    SYSTEM PRIVATE
      ${PYTHON_INCLUDE_DIRS}
  )

  foreach(lib ${_ARG_TARGET_LINK_LIBRARIES})
    target_link_libraries(${_ARG_TARGET_NAME} PUBLIC ${lib})
  endforeach()

  # Remove debug postfix for dartpy
  set_target_properties(${_ARG_TARGET_NAME}
    PROPERTIES
      DEBUG_POSTFIX ""
  )

  # Copy target for pytest
  # Create a custom target to copy the shared library
  add_custom_target(_copy_${_ARG_TARGET_NAME}
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${_ARG_TARGET_NAME}> ${DART_DARTPY_BUILD_DIR}
    DEPENDS ${_ARG_TARGET_NAME}
    COMMENT "Copying ${_ARG_TARGET_NAME} to ${DART_DARTPY_BUILD_DIR}"
  )

  # Add the custom target as a dependency of the main target
  add_dependencies(_copy_submodules _copy_${_ARG_TARGET_NAME})

  # TODO: Fix installing dartpy using CMake
  # Install the pybind module to site-packages directory
  # install(TARGETS ${_ARG_TARGET_NAME}
  #   LIBRARY DESTINATION "${PYTHON_SITE_PACKAGES}"
  # )

  dart_format_add(${_ARG_SOURCES})

  set_property(GLOBAL APPEND PROPERTY _DART_DARTPY_SUBMODULES ${_ARG_TARGET_NAME})

endfunction()
