# Copyright (c) 2011-2021, The DART development contributors

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
function(dart_property_get property_name output_var)
  get_property(is_defined GLOBAL PROPERTY ${property_name} DEFINED)
  if(NOT is_defined)
    define_property(GLOBAL PROPERTY ${property_name}
      BRIEF_DOCS "${property_name}"
      FULL_DOCS "Global properties for ${property_name}"
    )
  endif()
  get_property(property GLOBAL PROPERTY ${property_name})
  set(${output_var} ${property} PARENT_SCOPE)
endfunction()

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
# dart_check_compiler_visibility(<output_variable>)
#
# Macro to check for visibility capability in compiler
# cmake-format: on
macro(dart_check_compiler_visibility variable)
  include(CheckCXXCompilerFlag)
  check_cxx_compiler_flag(-fvisibility=hidden ${variable})
endmacro()

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
    ${project_base_dir}/cmake/dart_fetch_at_configure_step.cmake.in
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
function(dart_generate_meta_header_from_abs_paths output_filepath base_path)
  file(WRITE ${output_filepath} "// Automatically generated file by cmake\n\n")
  foreach(header ${ARGN})
    file(RELATIVE_PATH path_rel ${base_path}/ ${header})
    file(APPEND ${output_filepath} "#include \"${path_rel}\"\n")
  endforeach()
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

# ==============================================================================
# cmake-format: off
# dart_set_project()
# cmake-format: on

function(dart_set_project)
  set(prefix _ARG)
  set(options
  )
  set(oneValueArgs
    PROJECT_BASE_DIR
    INCLUDE_SOURCE_BASE_DIR
    INCLUDE_BINARY_BASE_DIR
    SRC_SOURCE_BASE_DIR
  )
  set(multiValueArgs
  )
  cmake_parse_arguments(
    "${prefix}" "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN}
  )

  if(NOT _ARG_INCLUDE_SOURCE_BASE_DIR)
    message(FATAL_ERROR "INCLUDE_SOURCE_BASE_DIR is not set")
  endif()

  if(NOT _ARG_INCLUDE_BINARY_BASE_DIR)
    message(FATAL_ERROR "INCLUDE_BINARY_BASE_DIR is not set")
  endif()

  if(NOT _ARG_SRC_SOURCE_BASE_DIR)
    message(FATAL_ERROR "SRC_SOURCE_BASE_DIR is not set")
  endif()
  
  set_property(
    GLOBAL PROPERTY DART_GLOBAL_PROPERTY_PROJECT_BASE_DIR "${_ARG_PROJECT_BASE_DIR}"
  )
  set_property(
    GLOBAL PROPERTY DART_GLOBAL_PROPERTY_PROJECT_INCLUDE_SOURCE_BASE_DIR "${_ARG_INCLUDE_SOURCE_BASE_DIR}"
  )
  set_property(
    GLOBAL PROPERTY DART_GLOBAL_PROPERTY_PROJECT_INCLUDE_BINARY_BASE_DIR "${_ARG_INCLUDE_BINARY_BASE_DIR}"
  )
  set_property(
    GLOBAL PROPERTY DART_GLOBAL_PROPERTY_PROJECT_SRC_SOURCE_BASE_DIR "${_ARG_SRC_SOURCE_BASE_DIR}"
  )

  if(DART_TRACE)
    message("[TRACE] PROJECT INCLUDE SOURCE DIR: ${_ARG_INCLUDE_SOURCE_BASE_DIR}")
    message("[TRACE] PROJECT INCLUDE BINARY DIR: ${_ARG_INCLUDE_BINARY_BASE_DIR}")
    message("[TRACE] PROJECT SRC SOURCE DIR : ${_ARG_SRC_SOURCE_BASE_DIR}\n")
  endif()

  add_compile_options(
    -Wall
    $<$<NOT:$<CONFIG:DEBUG>>:-Werror>
    -Wextra
    $<$<CONFIG:DEBUG>:-g>
    $<$<CONFIG:RELEASE>:-O3>
    $<$<CONFIG:RELWITHDEBINFO>:-g>
    $<$<CONFIG:RELWITHDEBINFO>:-O3>
    $<$<CONFIG:MINSIZEREL>:-O3>
  )
endfunction()

# ==============================================================================
# dart_generate_export_header(
#   TARGET_NAME <target_name>
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
    "#include \"detail/${_ARG_EXPORT_FILE_NAME}\"\n"
  )

  # Install generated export files
  # set(include_base_path ${CMAKE_INSTALL_INCLUDEDIR}/${org_name}/${project_name}${project_version_major})
  # set(export_install_path "${include_base_path}/${_ARG_DESTINATION}/")
  # set(detail_export_install_path "${export_install_path}/detail/")
  # install(FILES "${binary_export_file_path}"
  #   DESTINATION "${export_install_path}"
  # )
  # install(FILES "${binary_detail_export_file_path}"
  #   DESTINATION "${detail_export_install_path}"
  # )
endfunction()

# ==============================================================================
# cmake-format: off
# dart_get_component_target_name()
# cmake-format: on

function(dart_get_component_target_name)
  set(prefix _ARG)
  set(options
  )
  set(oneValueArgs
    COMPONENT_NAME
    OUTPUT_VAR
  )
  set(multiValueArgs
  )
  cmake_parse_arguments(
    "${prefix}" "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN}
  )

  if(NOT _ARG_COMPONENT_NAME)
    message(FATAL_ERROR "[ERROR] COMPONENT_NAME is not set!")
  endif()

  if(NOT _ARG_OUTPUT_VAR)
    message(FATAL_ERROR "[ERROR] OUTPUT_VAR is not set!")
  endif()

  set(target_name ${PROJECT_NAME}${DART_VERSION_MAJOR}-${_ARG_COMPONENT_NAME})

  set(${_ARG_OUTPUT_VAR} ${target_name} PARENT_SCOPE)
endfunction()

# ==============================================================================
# cmake-format: off
# dart_add_component()
# cmake-format: on

function(dart_add_component)
  set(prefix _ARG)
  set(options
  )
  set(oneValueArgs
    COMPONENT_NAME
  )
  set(multiValueArgs
  )
  cmake_parse_arguments(
    "${prefix}" "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN}
  )

  if(NOT _ARG_COMPONENT_NAME)
    message(FATAL_ERROR "[ERROR] COMPONENT_NAME is not set!")
  endif()

  set_property(
    GLOBAL PROPERTY DART_GLOBAL_PROPERTY_COMPONENT_LIST "${_ARG_COMPONENT_NAME}" APPEND
  )
endfunction()

# ==============================================================================
# cmake-format: off
# dart_add_component_dependent_components()
# cmake-format: on

function(dart_add_component_dependent_components)
  set(prefix _ARG)
  set(options
  )
  set(oneValueArgs
    COMPONENT_NAME
  )
  set(multiValueArgs
    DEPENDENT_COMPONENTS
  )
  cmake_parse_arguments(
    "${prefix}" "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN}
  )

  if(NOT _ARG_COMPONENT_NAME)
    message(FATAL_ERROR "[ERROR] COMPONENT_NAME is not set!")
  endif()

  set_property(
    GLOBAL
    PROPERTY
    DART_GLOBAL_PROPERTY_COMPONENT_${_ARG_COMPONENT_NAME}_DEPENDENT_COMPONENTS
    "${_ARG_DEPENDENT_COMPONENTS}"
    APPEND
  )
endfunction()

# ==============================================================================
# cmake-format: off
# dart_get_component_dependent_components()
# cmake-format: on

function(dart_get_component_dependent_components)
  set(prefix _ARG)
  set(options
  )
  set(oneValueArgs
    COMPONENT_NAME
    OUTPUT_VAR
  )
  set(multiValueArgs
  )
  cmake_parse_arguments(
    "${prefix}" "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN}
  )

  if(NOT _ARG_COMPONENT_NAME)
    message(FATAL_ERROR "[ERROR] COMPONENT_NAME is not set!")
  endif()

  if(NOT _ARG_OUTPUT_VAR)
    message(FATAL_ERROR "[ERROR] OUTPUT_VAR is not set!")
  endif()

  get_property(
    component_dependent_components
    GLOBAL
    PROPERTY
    DART_GLOBAL_PROPERTY_COMPONENT_${_ARG_COMPONENT_NAME}_DEPENDENT_COMPONENTS
  )

  set(${_ARG_OUTPUT_VAR} ${component_dependent_components} PARENT_SCOPE)
endfunction()

# ==============================================================================
# cmake-format: off
# dart_get_component_dependent_component_targets()
# cmake-format: on

function(dart_get_component_dependent_component_targets)
  set(prefix _ARG)
  set(options
  )
  set(oneValueArgs
    COMPONENT_NAME
    OUTPUT_VAR
  )
  set(multiValueArgs
  )
  cmake_parse_arguments(
    "${prefix}" "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN}
  )

  if(NOT _ARG_COMPONENT_NAME)
    message(FATAL_ERROR "[ERROR] COMPONENT_NAME is not set!")
  endif()

  if(NOT _ARG_OUTPUT_VAR)
    message(FATAL_ERROR "[ERROR] OUTPUT_VAR is not set!")
  endif()

  dart_get_component_dependent_components(
    COMPONENT_NAME ${_ARG_COMPONENT_NAME}
    OUTPUT_VAR component_dependent_components
  )

  foreach(comp ${component_dependent_components})
    dart_get_component_target_name(
      COMPONENT_NAME ${comp}
      OUTPUT_VAR component_target
    )
    list(APPEND component_dependent_component_targets ${component_target})
  endforeach()

  set(${_ARG_OUTPUT_VAR} ${component_dependent_component_targets} PARENT_SCOPE)
endfunction()

# ==============================================================================
# cmake-format: off
# dart_component_setup()
# cmake-format: on

function(dart_component_setup)
  set(prefix _ARG)
  set(options
    GENERATE_EXPORT_HEADER
    GENERATE_COMPONENT_ALL_HEADER
  )
  set(oneValueArgs
    COMPONENT_NAME
  )
  set(multiValueArgs
    DEPENDENT_COMPONENTS
    DEPENDENT_PACKAGES_REQUIRED
    DEPENDENT_PACKAGES_OPTIONAL
    TARGET_LINK_LIBRARIES_PUBLIC
    TARGET_LINK_LIBRARIES_PRIVATE
    TARGET_LINK_OPTIONS_PUBLIC
    TARGET_COMPILE_FEATURES_PUBLIC
    TARGET_COMPILE_FEATURES_PRIVATE
    TARGET_COMPILE_OPTIONS_PUBLIC
    TARGET_COMPILE_OPTIONS_PRIVATE
    TARGET_COMPILE_DEFINITIONS_PUBLIC
    TARGET_COMPILE_DEFINITIONS_PRIVATE
    SUB_DIRECTORIES
  )
  cmake_parse_arguments(
    "${prefix}" "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN}
  )

  if(NOT _ARG_COMPONENT_NAME)
    message(FATAL_ERROR "[ERROR] COMPONENT_NAME is not set!")
  endif()

  if(DART_DEBUG OR DART_TRACE)
    message("[DEBUG] ================================================")
    message("[DEBUG]  Component: ${_ARG_COMPONENT_NAME}")
    message("[DEBUG] =================================================")
    message("[DEBUG]")
  endif()

  dart_add_component(COMPONENT_NAME ${_ARG_COMPONENT_NAME})
  dart_add_component_dependent_components(
    COMPONENT_NAME ${_ARG_COMPONENT_NAME}
    DEPENDENT_COMPONENTS ${_ARG_DEPENDENT_COMPONENTS}
  )

  # Get global properties
  get_property(
    project_include_source_base_dir GLOBAL PROPERTY DART_GLOBAL_PROPERTY_PROJECT_INCLUDE_SOURCE_BASE_DIR
  )
  get_property(
    project_include_binary_base_dir GLOBAL PROPERTY DART_GLOBAL_PROPERTY_PROJECT_INCLUDE_BINARY_BASE_DIR
  )
  get_property(
    project_src_source_base_dir GLOBAL PROPERTY DART_GLOBAL_PROPERTY_PROJECT_SRC_SOURCE_BASE_DIR
  )

  # Set target name
  set(target_name ${PROJECT_NAME}${DART_VERSION_MAJOR}-${_ARG_COMPONENT_NAME})

  # Check dependencies
  foreach(package ${_ARG_DEPENDENT_PACKAGES_OPTIONAL})
    if(${package}_FOUND)
      if(NOT DART_SKIP_${package})
        list(APPEND _ARG_TARGET_COMPILE_DEFINITIONS_PUBLIC -DDART_HAVE_${package}=1)
        list(APPEND _ARG_DEPENDENT_PACKAGES_REQUIRED ${package})
      else()
        list(APPEND _ARG_TARGET_COMPILE_DEFINITIONS_PUBLIC -DDART_HAVE_${package}=0)
        message("[WARN] Building component [${_ARG_COMPONENT_NAME}] without [${package}] as [DART_SKIP_${package}=ON] is set")
      endif()
    else()
      list(APPEND _ARG_TARGET_COMPILE_DEFINITIONS_PUBLIC -DDART_HAVE_${package}=0)
      message("[WARN] Building component [${_ARG_COMPONENT_NAME}] without [${package}] as it's not found")
    endif()
  endforeach()

  # Current paths
  set(current_src_source_dir ${CMAKE_CURRENT_SOURCE_DIR})
  set(current_src_binary_dir ${CMAKE_CURRENT_BINARY_DIR})
  file(RELATIVE_PATH relative_src_source_dir "${project_src_source_base_dir}" "${current_src_source_dir}")
  set(current_include_source_dir ${project_include_source_base_dir}/dart/${relative_src_source_dir})  # TODO: make "dart" configurable
  set(current_include_binary_dir ${project_include_binary_base_dir}/dart/${relative_src_source_dir})
  file(RELATIVE_PATH relative_include_source_dir "${project_include_source_base_dir}" "${current_include_source_dir}")

  # Collect sources
  file(GLOB include_headers "${current_include_source_dir}/*.hpp")
  file(GLOB include_headers_relative RELATIVE ${project_include_source_base_dir} "${current_include_source_dir}/*.hpp")
  file(GLOB include_detail_headers "${current_include_source_dir}/detail/*.hpp")
  file(GLOB src_headers "*.hpp")
  file(GLOB src_sources "*.cpp")

  # Set context properties
  set_property(GLOBAL PROPERTY _DART_CURRENT_COMPONENT_INCLUDE_HEADERS ${include_headers})
  set_property(GLOBAL PROPERTY _DART_CURRENT_COMPONENT_INCLUDE_DETAIL_HEADERS ${include_detail_headers})
  set_property(GLOBAL PROPERTY _DART_CURRENT_COMPONENT_SRC_HEADERS ${src_headers})
  set_property(GLOBAL PROPERTY _DART_CURRENT_COMPONENT_SRC_SOURCES ${src_sources})
  set_property(GLOBAL PROPERTY _DART_CURRENT_TARGET_NAME ${target_name})
  set_property(GLOBAL PROPERTY _DART_CURRENT_TARGET_LINK_LIBRARIES_PUBLIC ${_ARG_TARGET_LINK_LIBRARIES_PUBLIC})
  set_property(GLOBAL PROPERTY _DART_CURRENT_TARGET_LINK_LIBRARIES_PRIVATE ${_ARG_TARGET_LINK_LIBRARIES_PRIVATE})
  set_property(GLOBAL PROPERTY _DART_CURRENT_TARGET_COMPILE_FEATURES_PUBLIC ${_ARG_TARGET_COMPILE_FEATURES_PUBLIC})
  set_property(GLOBAL PROPERTY _DART_CURRENT_TARGET_COMPILE_OPTIONS_PUBLIC ${_ARG_TARGET_COMPILE_OPTIONS_PUBLIC})
  set_property(GLOBAL PROPERTY _DART_CURRENT_TARGET_COMPILE_DEFINITIONS_PUBLIC ${_ARG_TARGET_COMPILE_DEFINITIONS_PUBLIC})

  # Collect sources from sub-directories
  foreach(sub_directory ${_ARG_SUB_DIRECTORIES})
    add_subdirectory(${sub_directory})
  endforeach()

  # Get context properties
  get_property(current_component_include_headers         GLOBAL PROPERTY _DART_CURRENT_COMPONENT_INCLUDE_HEADERS)
  get_property(current_component_include_detail_headers  GLOBAL PROPERTY _DART_CURRENT_COMPONENT_INCLUDE_DETAIL_HEADERS)
  get_property(current_component_src_headers             GLOBAL PROPERTY _DART_CURRENT_COMPONENT_SRC_SOURCES)
  get_property(current_component_src_sources             GLOBAL PROPERTY _DART_CURRENT_COMPONENT_SRC_SOURCES)
  get_property(current_component_dependency_packages     GLOBAL PROPERTY _DART_CURRENT_COMPONENT_DEPENDENT_PACKAGES)
  get_property(current_target_link_libraries_public      GLOBAL PROPERTY _DART_CURRENT_TARGET_LINK_LIBRARIES_PUBLIC)
  get_property(current_target_link_libraries_private     GLOBAL PROPERTY _DART_CURRENT_TARGET_LINK_LIBRARIES_PRIVATE)
  get_property(current_target_compile_features_public    GLOBAL PROPERTY _DART_CURRENT_TARGET_COMPILE_FEATURES_PUBLIC)
  get_property(current_target_compile_options_public     GLOBAL PROPERTY _DART_CURRENT_TARGET_COMPILE_OPTIONS_PUBLIC)
  get_property(current_target_compile_definitions_public GLOBAL PROPERTY _DART_CURRENT_TARGET_COMPILE_DEFINITIONS_PUBLIC)

  if(DART_TRACE)
    message("[TRACE] current_include_source_dir: ${current_include_source_dir}")
    message("[TRACE] current_include_binary_dir: ${current_include_binary_dir}")
    message("[TRACE] current_src_source_dir    : ${current_src_source_dir}")
    message("[TRACE] current_src_binary_dir    : ${current_src_binary_dir}\n")

    message("[TRACE] relative_include_source_dir: ${relative_include_source_dir}")
    message("[TRACE] relative_src_source_dir    : ${relative_src_source_dir}\n")

    message("[TRACE] current_component_include_headers       : ${current_component_include_headers}")
    message("[TRACE] current_component_include_detail_headers: ${current_component_include_detail_headers}")
    message("[TRACE] current_component_src_headers: ${current_component_src_headers}")
    message("[TRACE] current_component_src_sources: ${current_component_src_sources}")
    message("[TRACE] current_component_dependency_packages: ${current_component_dependency_packages}")
    message("[TRACE] current_target_link_libraries_public : ${current_target_link_libraries_public}")
    message("[TRACE] current_target_link_libraries_private: ${current_target_link_libraries_private}")
    message("[TRACE] current_target_compile_definitions_public: ${current_target_compile_definitions_public}")
    message("[TRACE] current_target_compile_options_public    : ${current_target_compile_options_public}\n")

    message("[TRACE] target_include_directories: ${project_include_source_base_dir}\n")
  endif()

  # Add library
  add_library(${target_name})

  if(DART_DEBUG OR DART_TRACE)
    message("[DEBUG] target_name: ${target_name}")
  endif()

  # Add sources
  target_sources(${target_name}
    PRIVATE
      ${current_component_include_headers}
      ${current_component_include_detail_headers}
      ${current_component_src_headers}
      ${current_component_src_sources}
  )
  dart_clang_format_add(
    ${current_component_include_headers}
    ${current_component_include_detail_headers}
    ${include_detail_headers}
    ${current_component_src_headers}
    ${current_component_src_sources}
  )

  # Set include directory
  target_include_directories(
    ${target_name}
    PUBLIC
      $<BUILD_INTERFACE:${project_include_source_base_dir}>
      $<BUILD_INTERFACE:${project_include_binary_base_dir}>
      $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}${DART_VERSION_MAJOR}>
  )

  # Set link libraries
  dart_get_component_dependent_components(
    COMPONENT_NAME ${_ARG_COMPONENT_NAME}
    OUTPUT_VAR dependent_components
  )
  dart_get_component_dependent_component_targets(
    COMPONENT_NAME ${_ARG_COMPONENT_NAME}
    OUTPUT_VAR dependent_component_targets
  )
  target_link_libraries(${target_name} PUBLIC ${dependent_component_targets})
  target_link_libraries(${target_name} PUBLIC ${current_target_link_libraries_public})
  target_link_libraries(${target_name} PRIVATE ${current_target_link_libraries_private})

  # Set target link options
  if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.13)
    target_link_options(${target_name} PUBLIC ${_ARG_TARGET_LINK_OPTIONS_PUBLIC})
  endif()

  # Set compile features
  target_compile_features(
    ${target_name} PUBLIC ${current_target_compile_features_public}
  )
  target_compile_features(
    ${target_name} PRIVATE ${current_target_compile_features_private}
  )

  # Set compile options
  target_compile_options(
    ${target_name} PUBLIC ${current_target_compile_options_public}
  )
  target_compile_options(
    ${target_name} PRIVATE ${current_target_compile_options_private}
  )
  
  # Set compile definitions
  target_compile_definitions(
    ${target_name} PUBLIC ${current_target_compile_definitions_public}
  )
  target_compile_definitions(
    ${target_name} PRIVATE ${current_target_compile_definitions_private}
  )

  if(DART_TRACE)
    message("[TRACE] Target compile features PUBLIC: ${current_target_compile_features_public}")
    message("[TRACE] Target compile features PRIVATE: ${current_target_compile_features_private}")
    message("[TRACE] Dependent component targets: ${dependent_component_targets}\n")
  endif()

  # Generate export header for the component
  if (_ARG_GENERATE_EXPORT_HEADER)
    dart_generate_export_header(
      TARGET_NAME ${target_name}
      DESTINATION ${current_include_binary_dir}
      EXPORT_FILE_NAME export.hpp
      BASE_NAME DART_${_ARG_COMPONENT_NAME}
    )
  endif()

  # Generate component "all" header
  if(_ARG_GENERATE_COMPONENT_ALL_HEADER)
    dart_generate_meta_header_from_abs_paths(
      ${current_include_binary_dir}/all.hpp
      ${project_include_source_base_dir}
      ${current_component_include_headers}
    )
    install(
      FILES ${current_include_binary_dir}/all.hpp
      DESTINATION
        ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}${DART_VERSION_MAJOR}/${relative_include_source_dir}
    )
  endif()

  # Install static headers
  install(
    FILES
      ${include_headers}
    DESTINATION
      ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}${DART_VERSION_MAJOR}/${relative_include_source_dir}
  )
  install(
    FILES
      ${include_detail_headers}
    DESTINATION
      ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}${DART_VERSION_MAJOR}/${relative_include_source_dir}/detail
  )

  # Install targets
  # cmake-format: off
  install(
    TARGETS ${target_name}
    EXPORT ${target_name}-targets
    PUBLIC_HEADER  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    PRIVATE_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    INCLUDES       DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    RUNTIME        DESTINATION ${CMAKE_INSTALL_BINDIR}
    ARCHIVE        DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY        DESTINATION ${CMAKE_INSTALL_LIBDIR}
  )
  # cmake-format: on
  install(
    EXPORT ${target_name}-targets
    FILE ${target_name}-targets.cmake
    DESTINATION
      ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}${DART_VERSION_MAJOR}
  )

  # Register module
  set_property(
    GLOBAL APPEND PROPERTY DART_CPP_BUILDING_MODULES "${module_name}"
  )

  if(DART_DEBUG)
    message("[DEBUG] Dependent components: ${dependent_components}")
    message("[DEBUG] Dependent component targets: ${dependent_component_targets}")
    message("[DEBUG]")
    message("[DEBUG] =====================================================\n")
  endif()
endfunction()

#===============================================================================
function(dart_component_setup_subdirectory)
  set(prefix _ARG)
  set(options
  )
  set(oneValueArgs
    SUB_COMPONENT_NAME
  )
  set(multiValueArgs
    DEPENDENT_PACKAGES_REQUIRED
    TARGET_LINK_LIBRARIES_PUBLIC
    TARGET_LINK_LIBRARIES_PRIVATE
    COMPONENT_DEPENDENT_PACKAGES
    TARGET_COMPILE_DEFINITIONS_PUBLIC
    TARGET_COMPILE_OPTIONS_PUBLIC
    SUB_DIRECTORIES
  )
  cmake_parse_arguments(
    "${prefix}" "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN}
  )

  # Get global properties
  get_property(
    project_include_source_base_dir GLOBAL PROPERTY DART_GLOBAL_PROPERTY_PROJECT_INCLUDE_SOURCE_BASE_DIR
  )
  get_property(
    project_include_binary_base_dir GLOBAL PROPERTY DART_GLOBAL_PROPERTY_PROJECT_INCLUDE_BINARY_BASE_DIR
  )
  get_property(
    project_src_source_base_dir GLOBAL PROPERTY DART_GLOBAL_PROPERTY_PROJECT_SRC_SOURCE_BASE_DIR
  )

  # Current paths
  set(current_src_source_dir ${CMAKE_CURRENT_SOURCE_DIR})
  set(current_src_binary_dir ${CMAKE_CURRENT_BINARY_DIR})
  file(RELATIVE_PATH relative_src_source_dir "${project_src_source_base_dir}" "${current_src_source_dir}")
  set(current_include_source_dir ${project_include_source_base_dir}/dart/${relative_src_source_dir})  # TODO: make "dart" configurable
  set(current_include_binary_dir ${project_include_binary_base_dir}/dart/${relative_src_source_dir})
  file(RELATIVE_PATH relative_include_source_dir "${project_include_source_base_dir}" "${current_include_source_dir}")

  # Context variables
  get_property(current_target_name    GLOBAL PROPERTY _DART_CURRENT_TARGET_NAME)

  # Check dependencies
  foreach(package ${_ARG_DEPENDENT_PACKAGES_REQUIRED})
    if(${package}_FOUND)
      if(NOT DART_SKIP_${package})
        list(APPEND _ARG_TARGET_COMPILE_DEFINITIONS_PUBLIC -DDART_HAVE_${package}=1)
      else()
        list(APPEND _ARG_TARGET_COMPILE_DEFINITIONS_PUBLIC -DDART_HAVE_${package}=0)
        message(STATUS "Building target [${current_target_name}] without sub component [${sub_component_name}] because DART_SKIP_${package} is set")
        return()
      endif()
    else()
      list(APPEND _ARG_TARGET_COMPILE_DEFINITIONS_PUBLIC -DDART_HAVE_${package}=0)
      message(STATUS "Building target [${current_target_name}] without sub component [${sub_component_name}] because of missing dependency package ${package}")
      return()
    endif()
  endforeach()

  # Check optional dependencies
  foreach(package ${_ARG_DEPENDENT_PACKAGES_OPTIONAL})
    if(${package}_FOUND)
      if(NOT DART_SKIP_${package})
        list(APPEND _ARG_TARGET_COMPILE_DEFINITIONS_PUBLIC -DDART_HAVE_${package}=1)
      else()
        list(APPEND _ARG_TARGET_COMPILE_DEFINITIONS_PUBLIC -DDART_HAVE_${package}=0)
        message(STATUS "Building target [${current_target_name}] without [${package}] because [DART_SKIP_${package}] is set")
      endif()
    else()
      list(APPEND _ARG_TARGET_COMPILE_DEFINITIONS_PUBLIC -DDART_HAVE_${package}=0)
      message(STATUS "Building target [${current_target_name}] without [${package}] because the package is not found")
    endif()
  endforeach()

  # Add sub-directories
  foreach(sub_directory ${sub_directories})
    add_subdirectory(${sub_directory})
  endforeach()

  # Paths
  #file(RELATIVE_PATH component_rel_path ${component_base_path} ${CMAKE_CURRENT_SOURCE_DIR})

  # Collect sources
  file(GLOB include_headers "${current_include_source_dir}/*.hpp")
  file(GLOB include_detail_headers "${current_include_source_dir}/detail/*.hpp")
  file(GLOB src_headers "*.hpp")
  file(GLOB src_sources "*.cpp")

  set_property(GLOBAL APPEND PROPERTY _DART_CURRENT_COMPONENT_INCLUDE_HEADERS ${include_headers})
  set_property(GLOBAL APPEND PROPERTY _DART_CURRENT_COMPONENT_INCLUDE_DETAIL_HEADERS ${include_detail_headers})
  set_property(GLOBAL APPEND PROPERTY _DART_CURRENT_COMPONENT_SRC_HEADERS ${src_headers})
  set_property(GLOBAL APPEND PROPERTY _DART_CURRENT_COMPONENT_SRC_SOURCES ${src_sources})

  # Set link libraries
  set_property(GLOBAL APPEND PROPERTY _DART_CURRENT_TARGET_LINK_LIBRARIES_PUBLIC ${_ARG_TARGET_LINK_LIBRARIES_PUBLIC})
  set_property(GLOBAL APPEND PROPERTY _DART_CURRENT_TARGET_LINK_LIBRARIES_PRIVATE ${_ARG_TARGET_LINK_LIBRARIES_PRIVATE})

  # Set compile features
  set_property(GLOBAL APPEND PROPERTY _DART_CURRENT_TARGET_COMPILE_FEATURES_PUBLIC${_ARG_TARGET_COMPILE_FEATURES_PUBLIC})

  # Set compile options
  set_property(GLOBAL APPEND PROPERTY _DART_CURRENT_TARGET_COMPILE_OPTIONS_PUBLIC ${_ARG_TARGET_COMPILE_OPTIONS_PUBLIC})

  # Set compile definitions
  set_property(GLOBAL APPEND PROPERTY _DART_CURRENT_TARGET_COMPILE_DEFINITIONS_PUBLIC ${_ARG_TARGET_COMPILE_DEFINITIONS_PUBLIC})

  # Component settings
  set_property(GLOBAL APPEND PROPERTY _DART_CURRENT_COMPONENT_DEPENDENT_PACKAGES ${_ARG_COMPONENT_DEPENDENT_PACKAGES})

  # Install static headers
  install(
    FILES ${include_headers}
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}${DART_VERSION_MAJOR}/${relative_include_source_dir}
  )
  install(
    FILES ${include_detail_headers}
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}${DART_VERSION_MAJOR}/${relative_include_source_dir}/detail
  )

  # Format files
  #dart_format_add(${headers} ${sources})
endfunction()

# ==============================================================================
# cmake-format: off
# dart_build_tests()
# cmake-format: on

function(dart_build_tests)
  set(prefix _ARG)
  set(options
  )
  set(oneValueArgs
    COMPONENT_NAME
    TARGET_PREFIX
  )
  set(multiValueArgs
    INCLUDE_DIRS
  )
  cmake_parse_arguments(
    "${prefix}" "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN}
  )

  if(NOT _ARG_COMPONENT_NAME)
    message(FATAL_ERROR "[ERROR] COMPONENT_NAME is not set!")
  endif()

  if(NOT _ARG_TARGET_PREFIX)
    set(_ARG_TARGET_PREFIX TEST)
  endif()

  # Get global properties
  get_property(
    project_include_source_base_dir GLOBAL PROPERTY DART_GLOBAL_PROPERTY_PROJECT_INCLUDE_SOURCE_BASE_DIR
  )
  get_property(
    project_include_binary_base_dir GLOBAL PROPERTY DART_GLOBAL_PROPERTY_PROJECT_INCLUDE_BINARY_BASE_DIR
  )
  get_property(
    project_src_source_base_dir GLOBAL PROPERTY DART_GLOBAL_PROPERTY_PROJECT_SRC_SOURCE_BASE_DIR
  )

  # Get component targets
  dart_get_component_target_name(
    COMPONENT_NAME ${_ARG_COMPONENT_NAME}
    OUTPUT_VAR component_target
  )
  dart_get_component_dependent_component_targets(
    COMPONENT_NAME ${_ARG_COMPONENT_NAME}
    OUTPUT_VAR dependent_component_targets
  )

  foreach(dep ${dependent_component_targets})
    if(NOT TARGET ${dep})
      message(WARNING "[WARN] Skipping tests for component [${_ARG_COMPONENT_NAME}] due to missing component target [${dep}]")
      return()
    endif()
  endforeach()

  # Glob all the test files
  file(GLOB_RECURSE test_files RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "test_*.cpp")
  if(test_files)
    list(SORT test_files)
  endif()

  foreach(source ${test_files})
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

    add_executable(${target_name} EXCLUDE_FROM_ALL ${source})
    add_test(NAME ${target_name} COMMAND $<TARGET_FILE:${target_name}>)

    # Include directories
    target_include_directories(
      ${target_name} PRIVATE ${_ARG_INCLUDE_DIRS}
    )

    # Link libraries
    target_link_libraries(${target_name} PRIVATE gtest gtest_main)
    target_link_libraries(
      ${target_name} PRIVATE ${component_target} ${dependent_component_targets}
    )
    target_link_libraries(
      ${target_name} PRIVATE ${_ARG_LINK_LIBRARIES}
    )
    if(UNIX)
      # gtest requies pthread when compiled on a Unix machine
      target_link_libraries(${target_name} PRIVATE pthread)
    endif()

    # Add the test target to the list
    dart_property_add(DART_GLOBAL_PROPERTY_TEST_LIST ${target_name})

  endforeach()

  dart_clang_format_add(${test_files})

endfunction()


# ==============================================================================
# cmake-format: off
# dart_build_executable()
# cmake-format: on

function(dart_build_executable)
  set(prefix _ARG)
  set(options
  )
  set(oneValueArgs
    TARGET_NAME
  )
  set(multiValueArgs
    DEPENDENT_COMPONENTS
    INCLUDE_DIRS
  )
  cmake_parse_arguments(
    "${prefix}" "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN}
  )

  file(GLOB_RECURSE headers "*.hpp")
  file(GLOB_RECURSE sources "*.cpp")

  add_executable(${_ARG_TARGET_NAME} EXCLUDE_FROM_ALL ${headers} ${sources})

  foreach(comp ${_ARG_DEPENDENT_COMPONENTS})
    dart_get_component_target_name(
      COMPONENT_NAME ${comp}
      OUTPUT_VAR component_target
    )
    list(APPEND component_dependent_component_targets ${component_target})
  endforeach()

  target_link_libraries(${_ARG_TARGET_NAME} PRIVATE ${component_dependent_component_targets})

  dart_clang_format_add(${header} ${sources})

endfunction()

# ==============================================================================
function(dart_get_test_list output_var)
  dart_property_get(DART_GLOBAL_PROPERTY_TEST_LIST test_list)
  set(${output_var} ${test_list} PARENT_SCOPE)
endfunction()

#===============================================================================
function(dart_clang_format_add)
  set(prefix _ARG)
  set(options
  )
  set(oneValueArgs
    VERSION
  )
  set(multiValueArgs
  )
  cmake_parse_arguments(
    "${prefix}" "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN}
  )

  foreach(source ${ARGN})
    if(IS_ABSOLUTE ${source})
      set(source_abs ${source})
    else()
      get_filename_component(
        source_abs ${CMAKE_CURRENT_LIST_DIR}/${source} ABSOLUTE
      )
    endif()
    if(EXISTS ${source_abs})
      dart_property_add(DART_GLOBAL_PROPERTY_CLANG_FORMAT_FILES ${source_abs})
    else()
      message(FATAL_ERROR "Cannot format not existing file: ${source_abs}")
    endif()
  endforeach()
endfunction()

#===============================================================================
function(dart_clang_format_setup)
  set(prefix _ARG)
  set(options
  )
  set(oneValueArgs
    VERSION
  )
  set(multiValueArgs
  )
  cmake_parse_arguments(
    "${prefix}" "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN}
  )

  if(_ARG_VERSION)
    set(clang_format_name clang-format-${_ARG_VERSION})
  else()
    set(clang_format_name clang-format)
  endif()

  find_program(
    clang_format_executable
    NAMES ${clang_format_name}
  )

  if(DART_TRACE)
    message("[TRACE] clang format: ${clang_format_executable}")
  endif()

  if(clang_format_executable)
    dart_property_get(DART_GLOBAL_PROPERTY_CLANG_FORMAT_FILES files)
    list(LENGTH files files_length)

    if(files)
      add_custom_target(
        clang-format
        COMMAND ${CMAKE_COMMAND} -E echo "Formatting ${files_length} files..."
        COMMAND ${clang_format_executable} -style=file -i ${files}
        COMMAND ${CMAKE_COMMAND} -E echo "Formatting done"
      )
    else()
      add_custom_target(
        clang-format
        COMMAND ${CMAKE_COMMAND} -E echo "No files to format"
      )
    endif()
  else()
    add_custom_target(
      clang-format
      COMMAND ${CMAKE_COMMAND} -E echo "clang-format not found"
    )
  endif()
endfunction()

