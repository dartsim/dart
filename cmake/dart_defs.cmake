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

# dart_get_max_string_length(var [value1 value2...])
function(dart_get_max_string_length var)
  foreach(item ${ARGN})
    string(LENGTH ${item} length)
    list(APPEND list ${length})
  endforeach()
  dart_get_max(choice ${list})
  set(${var} ${choice} PARENT_SCOPE)
endfunction()

# cmake-format: off
# dart_option(<variable> "<help_text>" <value>)
# cmake-format: on
function(dart_option variable help_text default_value)
  set_property(
    GLOBAL
    PROPERTY DART_DETAIL_PROPERTY_OPTION_VARIABLE "${variable}"
    APPEND
  )
  set_property(
    GLOBAL
    PROPERTY DART_DETAIL_property_option_help_text "${help_text}"
    APPEND
  )
  set_property(
    GLOBAL
    PROPERTY DART_DETAIL_property_option_default_value "${default_value}"
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

# cmake-format: off
# dart_print_options()
# cmake-format: on
function(dart_print_options)
  # Print the header
  message(STATUS "[ Options ]")

  get_property(
    option_variables
    GLOBAL
    PROPERTY DART_DETAIL_PROPERTY_OPTION_VARIABLE
  )
  get_property(
    option_help_texts
    GLOBAL
    PROPERTY DART_DETAIL_property_option_help_text
  )
  get_property(
    option_default_values
    GLOBAL
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

function(_dart_msvc_runtime_flag_for_config out runtime_library config)
  string(TOUPPER "${config}" config_upper)

  if(runtime_library STREQUAL "MultiThreadedDLL")
    set(runtime_flag "/MD")
  elseif(runtime_library STREQUAL "MultiThreadedDebugDLL")
    set(runtime_flag "/MDd")
  elseif(runtime_library STREQUAL "MultiThreaded")
    set(runtime_flag "/MT")
  elseif(runtime_library STREQUAL "MultiThreadedDebug")
    set(runtime_flag "/MTd")
  elseif(runtime_library STREQUAL "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
    if(config_upper STREQUAL "DEBUG")
      set(runtime_flag "/MDd")
    else()
      set(runtime_flag "/MD")
    endif()
  elseif(runtime_library STREQUAL "MultiThreaded$<$<CONFIG:Debug>:Debug>")
    if(config_upper STREQUAL "DEBUG")
      set(runtime_flag "/MTd")
    else()
      set(runtime_flag "/MT")
    endif()
  else()
    message(FATAL_ERROR "Unsupported MSVC runtime library: ${runtime_library}")
  endif()

  set(${out} "${runtime_flag}" PARENT_SCOPE)
endfunction()

macro(_dart_replace_msvc_runtime_flag flags_variable runtime_flag)
  if(DEFINED ${flags_variable})
    set(flags_value "${${flags_variable}}")
  else()
    set(flags_value "")
  endif()

  string(
    REGEX REPLACE
    "(^|[ \t\r\n])[-/]M[DT]d?([ \t\r\n]|$)"
    " "
    flags_value
    "${flags_value}"
  )
  string(STRIP "${flags_value}" flags_value)

  if(flags_value)
    set(flags_value "${flags_value} ${runtime_flag}")
  else()
    set(flags_value "${runtime_flag}")
  endif()

  set(${flags_variable} "${flags_value}")
  set(${flags_variable} "${flags_value}" PARENT_SCOPE)
endmacro()

function(_dart_msvc_runtime_flag_fallback_needed out)
  set(needs_fallback OFF)
  set(configs Debug Release RelWithDebInfo MinSizeRel)

  if(CMAKE_CONFIGURATION_TYPES)
    list(APPEND configs ${CMAKE_CONFIGURATION_TYPES})
  endif()
  if(CMAKE_BUILD_TYPE)
    list(APPEND configs ${CMAKE_BUILD_TYPE})
  endif()
  list(REMOVE_DUPLICATES configs)

  foreach(lang C CXX)
    if(DEFINED CMAKE_${lang}_FLAGS)
      if(
        "${CMAKE_${lang}_FLAGS}" MATCHES "(^|[ \t\r\n])[-/]M[DT]d?([ \t\r\n]|$)"
      )
        set(needs_fallback ON)
      endif()
    endif()

    foreach(config IN LISTS configs)
      string(TOUPPER "${config}" config_upper)
      set(flags_variable CMAKE_${lang}_FLAGS_${config_upper})

      if(DEFINED ${flags_variable})
        if(
          "${${flags_variable}}" MATCHES "(^|[ \t\r\n])[-/]M[DT]d?([ \t\r\n]|$)"
        )
          set(needs_fallback ON)
        endif()
      endif()
    endforeach()
  endforeach()

  if(POLICY CMP0091)
    cmake_policy(GET CMP0091 cmp0091)
    if(NOT cmp0091 STREQUAL "NEW")
      set(needs_fallback ON)
    endif()
  endif()

  set(${out} ${needs_fallback} PARENT_SCOPE)
endfunction()

macro(_dart_apply_msvc_runtime_flag_fallback runtime_library)
  set(configs Debug Release RelWithDebInfo MinSizeRel)

  if(CMAKE_CONFIGURATION_TYPES)
    list(APPEND configs ${CMAKE_CONFIGURATION_TYPES})
  endif()
  if(CMAKE_BUILD_TYPE)
    list(APPEND configs ${CMAKE_BUILD_TYPE})
  endif()
  list(REMOVE_DUPLICATES configs)

  _dart_msvc_runtime_flag_for_config(
    default_runtime_flag
    "${runtime_library}"
    ""
  )

  foreach(lang C CXX)
    if(DEFINED CMAKE_${lang}_FLAGS)
      _dart_replace_msvc_runtime_flag(
        CMAKE_${lang}_FLAGS
        "${default_runtime_flag}"
      )
    endif()

    foreach(config IN LISTS configs)
      string(TOUPPER "${config}" config_upper)
      set(flags_variable CMAKE_${lang}_FLAGS_${config_upper})

      if(DEFINED ${flags_variable})
        _dart_msvc_runtime_flag_for_config(
          config_runtime_flag
          "${runtime_library}"
          "${config}"
        )
        _dart_replace_msvc_runtime_flag(
          ${flags_variable}
          "${config_runtime_flag}"
        )
      endif()
    endforeach()
  endforeach()
endmacro()

#-------------------------------------------------------------------------------
# Configure MSVC runtime-library policy before any targets are created.
#-------------------------------------------------------------------------------
function(dart_configure_msvc_runtime_library)
  if(NOT MSVC)
    return()
  endif()

  set(_dart_msvc_runtime_preserved OFF)
  if(DEFINED CMAKE_MSVC_RUNTIME_LIBRARY)
    set(_dart_msvc_runtime_library "${CMAKE_MSVC_RUNTIME_LIBRARY}")
    set(_dart_msvc_runtime_preserved ON)
  else()
    if(DART_MSVC_FORCE_RELEASE_RUNTIME)
      set(_dart_msvc_runtime_library "MultiThreadedDLL")
    elseif(DART_RUNTIME_LIBRARY STREQUAL "/MT")
      set(_dart_msvc_runtime_library "MultiThreaded$<$<CONFIG:Debug>:Debug>")
    else()
      set(_dart_msvc_runtime_library "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
    endif()
    set(
      CMAKE_MSVC_RUNTIME_LIBRARY
      "${_dart_msvc_runtime_library}"
      CACHE STRING
      "MSVC runtime library for DART targets"
      FORCE
    )
  endif()

  if(DEFINED CACHE{CMAKE_MSVC_RUNTIME_LIBRARY})
    set_property(
      CACHE CMAKE_MSVC_RUNTIME_LIBRARY
      PROPERTY
        STRINGS
          "MultiThreadedDLL"
          ""
          "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL"
          "MultiThreaded$<$<CONFIG:Debug>:Debug>"
    )
  endif()

  set(_dart_msvc_runtime_fallback_needed OFF)
  if(
    NOT _dart_msvc_runtime_preserved
    OR NOT _dart_msvc_runtime_library STREQUAL ""
  )
    _dart_msvc_runtime_flag_fallback_needed(_dart_msvc_runtime_fallback_needed)
    if(_dart_msvc_runtime_fallback_needed)
      _dart_apply_msvc_runtime_flag_fallback("${_dart_msvc_runtime_library}")
    endif()
  endif()

  if(_dart_msvc_runtime_preserved)
    if(_dart_msvc_runtime_library STREQUAL "")
      message(STATUS "MSVC runtime library: <empty> (preserved from caller)")
    else()
      message(
        STATUS
        "MSVC runtime library: ${_dart_msvc_runtime_library} (preserved from caller)"
      )
    endif()
  elseif(DART_MSVC_FORCE_RELEASE_RUNTIME)
    message(
      STATUS
      "DART_MSVC_FORCE_RELEASE_RUNTIME=ON: using /MD for all configurations"
    )
  else()
    message(STATUS "MSVC runtime library: ${CMAKE_MSVC_RUNTIME_LIBRARY}")
  endif()

  if(_dart_msvc_runtime_fallback_needed)
    message(STATUS "Updated MSVC runtime flags for CMP0091 OLD compatibility")
  endif()
endfunction()

#-------------------------------------------------------------------------------
# Configure MSVC-specific compiler and linker policy.
#-------------------------------------------------------------------------------
function(dart_configure_msvc_toolchain)
  if(NOT MSVC)
    return()
  endif()

  set(options)
  set(oneValueArgs REQUIRED_VERSION REQUIRED_LABEL)
  set(multiValueArgs)
  cmake_parse_arguments(
    ARG
    "${options}"
    "${oneValueArgs}"
    "${multiValueArgs}"
    ${ARGN}
  )

  if(
    ARG_REQUIRED_VERSION
    AND MSVC_VERSION VERSION_LESS "${ARG_REQUIRED_VERSION}"
    AND CMAKE_CXX_COMPILER_ID STREQUAL "MSVC"
  )
    if(ARG_REQUIRED_LABEL)
      set(_dart_msvc_required_label " (${ARG_REQUIRED_LABEL})")
    else()
      set(_dart_msvc_required_label "")
    endif()
    message(
      FATAL_ERROR
      "MSVC ${MSVC_VERSION} is detected, but ${PROJECT_NAME_UPPERCASE} "
      "requires MSVC ${ARG_REQUIRED_VERSION} or greater"
      "${_dart_msvc_required_label}."
    )
  endif()

  add_compile_definitions(
    _CRT_SECURE_NO_WARNINGS
    _ENABLE_EXTENDED_ALIGNED_STORAGE
  )

  add_compile_options(
    $<$<COMPILE_LANGUAGE:CXX>:/EHsc>
    $<$<COMPILE_LANGUAGE:CXX>:/permissive->
    $<$<COMPILE_LANGUAGE:CXX>:/Zc:twoPhase->
    $<$<COMPILE_LANGUAGE:CXX>:/utf-8>
    $<$<COMPILE_LANGUAGE:CXX>:/wd4005>
    $<$<COMPILE_LANGUAGE:CXX>:/wd4099>
    $<$<COMPILE_LANGUAGE:CXX>:/wd4146>
    $<$<COMPILE_LANGUAGE:CXX>:/wd4244>
    $<$<COMPILE_LANGUAGE:CXX>:/wd4250>
    $<$<COMPILE_LANGUAGE:CXX>:/wd4267>
    $<$<COMPILE_LANGUAGE:CXX>:/wd4305>
    $<$<COMPILE_LANGUAGE:CXX>:/wd4334>
    $<$<COMPILE_LANGUAGE:CXX>:/wd4838>
    $<$<COMPILE_LANGUAGE:CXX>:/wd4996>
    $<$<COMPILE_LANGUAGE:CXX>:/bigobj>
  )

  if(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    add_compile_options(
      $<$<COMPILE_LANGUAGE:CXX>:/MP>
      $<$<COMPILE_LANGUAGE:CXX>:/FS>
    )
  endif()

  if(NOT DART_MSVC_DEFAULT_OPTIONS)
    add_compile_options(
      $<$<COMPILE_LANGUAGE:CXX>:/W1>
      $<$<COMPILE_LANGUAGE:CXX>:/Zi>
      $<$<COMPILE_LANGUAGE:CXX>:/Gy>
      $<$<AND:$<COMPILE_LANGUAGE:CXX>,$<CONFIG:Release>>:/GL>
    )
    add_link_options($<$<CONFIG:Release>:/INCREMENTAL:NO>)
  endif()
endfunction()

function(dart_library)
  set(prefix _ARG)
  set(options GLOB_HEADERS GLOB_SOURCES)
  set(oneValueArgs NAME)
  set(
    multiValueArgs
    HEADERS
    SOURCES
    PUBLIC_LINK_LIBRARIES
    PRIVATE_LINK_LIBRARIES
    PUBLIC_COMPILE_DEFINITIONS
    PRIVATE_COMPILE_DEFINITIONS
  )
  cmake_parse_arguments(
    "${prefix}"
    "${options}"
    "${oneValueArgs}"
    "${multiValueArgs}"
    ${ARGN}
  )

  # Get the current directory relative to the root of the project
  # assuming that dart/* is the root of the source tree
  string(
    REPLACE
    "${CMAKE_SOURCE_DIR}/"
    ""
    relative_path
    ${CMAKE_CURRENT_SOURCE_DIR}
  )

  if(${_ARG_GLOB_HEADERS})
    file(GLOB_RECURSE headers "*.hpp")
    list(APPEND _ARG_HEADERS ${headers})
  endif()

  if(${_ARG_GLOB_SOURCES})
    file(GLOB_RECURSE sources "*.cpp")
    list(APPEND _ARG_SOURCES ${sources})
  endif()

  add_library(${_ARG_NAME} ${_ARG_HEADERS} ${_ARG_SOURCES})

  target_include_directories(
    ${_ARG_NAME}
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
      $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}>
      $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  )

  target_compile_features(${_ARG_NAME} PUBLIC cxx_std_17)

  target_link_libraries(
    ${_ARG_NAME}
    PUBLIC ${_ARG_PUBLIC_LINK_LIBRARIES}
    PRIVATE ${_ARG_PRIVATE_LINK_LIBRARIES}
  )

  target_compile_definitions(
    ${_ARG_NAME}
    PUBLIC ${_ARG_PUBLIC_COMPILE_DEFINITIONS}
    PRIVATE ${_ARG_PRIVATE_COMPILE_DEFINITIONS}
  )

  set_target_properties(${_ARG_NAME} PROPERTIES OUTPUT_NAME dart-${_ARG_NAME})

  include(GNUInstallDirs)

  foreach(header ${_ARG_HEADERS})
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
  set(options REQUIRED)
  set(oneValueArgs INCLUDE_DIR SOURCE_DIR INSTALL_DIR)
  set(
    multiValueArgs
    INPUT # optional
    EXCLUDE
  )
  cmake_parse_arguments(
    "${prefix}"
    "${options}"
    "${oneValueArgs}"
    "${multiValueArgs}"
    ${ARGN}
  )

  include(GNUInstallDirs)

  if(NOT _ARG_INCLUDE_DIR)
    message(FATAL_ERROR "INCLUDE_DIR is not set")
  endif()

  if(NOT _ARG_SOURCE_DIR)
    message(FATAL_ERROR "SOURCE_DIR is not set")
  endif()

  if(NOT _ARG_INSTALL_DIR)
    set(
      _ARG_INSTALL_DIR
      ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_DOXDIR}/${PROJECT_NAME}${DART_VERSION_MAJOR}/coverage
    )
  endif()

  # Find gcovr
  if(NOT GCOVR_EXECUTABLE)
    find_program(GCOVR_EXECUTABLE gcovr QUIET)
    if(NOT GCOVR_EXECUTABLE)
      if(_ARG_REQUIRED)
        message(
          FATAL_ERROR
          "Failed to find gcovr. Install gcovr or remove REQUIRED option."
        )
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
  execute_process(
    COMMAND ${GCOVR_EXECUTABLE} --version
    OUTPUT_VARIABLE GCOVR_VERSION_RAW_OUTPUT
  )
  string(STRIP ${GCOVR_VERSION_RAW_OUTPUT} GCOVR_VERSION_RAW_OUTPUT)
  string(
    REPLACE
    "gcovr "
    ""
    GCOVR_VERSION_RAW_OUTPUT
    ${GCOVR_VERSION_RAW_OUTPUT}
  )

  # Set options based on the Gcovr version
  if(${GCOVR_VERSION_RAW_OUTPUT} VERSION_GREATER_EQUAL 4.2)
    set(gcovr_options --exclude-throw-branches)
  endif()

  add_custom_target(
    coverage
    COMMAND
      ${GCOVR_EXECUTABLE} -r ${CMAKE_CURRENT_SOURCE_DIR} -f ${gcovr_include_dir}
      -f ${gcovr_source_dir}
    DEPENDS tests_and_run
    COMMENT "Generating line coverage report..."
  )

  add_custom_target(
    coverage_branch
    COMMAND
      ${GCOVR_EXECUTABLE} -b ${gcovr_options} --exclude-unreachable-branches -r
      ${CMAKE_CURRENT_SOURCE_DIR} -f ${gcovr_include_dir} -f ${gcovr_source_dir}
    DEPENDS tests_and_run
    COMMENT "Generating branch coverage report..."
  )

  add_custom_target(
    coverage_html
    COMMAND ${CMAKE_COMMAND} -E make_directory ${gcovr_html_dir}
    COMMAND
      ${GCOVR_EXECUTABLE} --html --html-details ${gcovr_options}
      --exclude-unreachable-branches -o "${gcovr_index_path}" -r
      ${CMAKE_CURRENT_SOURCE_DIR} -f ${gcovr_include_dir} -f ${gcovr_source_dir}
    DEPENDS coverage coverage_branch
    COMMENT "Generating a detailed HTML coverage report in ${gcovr_index_path}"
  )

  if(APPLE)
    set(open_command_name "open")
  else()
    set(open_command_name "xdg-open")
  endif()
  find_program(
    open_command
    NAMES ${open_command_name}
    DOC "Path to ${open_command_name}"
  )

  if(open_command)
    add_custom_target(
      coverage_view
      "${open_command}" "${gcovr_index_path}"
      DEPENDS coverage_html
      COMMENT "Opening documentation in a web browser."
    )
  else()
    message(
      WARNING
      "Failed to find ${open_command_name}, to enable "
      "'coverage_view', please install xdg-utils"
    )
  endif()

  # Remove all gcovr data and delete the html directory
  add_custom_target(
    coverage_cleanup
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
  set(oneValueArgs TYPE TARGET_PREFIX TEST_LIST)
  set(
    multiValueArgs
    SOURCES
    INCLUDE_DIRS
    LINK_LIBRARIES
    LINK_DART_LIBRARIES
    COMPILE_DEFINITIONS
  )
  cmake_parse_arguments(
    "${prefix}"
    "${options}"
    "${oneValueArgs}"
    "${multiValueArgs}"
    ${ARGN}
  )

  if(NOT _ARG_TYPE)
    message(
      FATAL_ERROR
      "DEVELOPER ERROR: You must specify a TYPE for your benchmarks!"
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

  foreach(source ${_ARG_SOURCES})
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
    target_include_directories(${target_name} PRIVATE ${_ARG_INCLUDE_DIRS})

    target_link_libraries(
      ${target_name}
      PRIVATE benchmark::benchmark benchmark::benchmark_main
    )

    if(UNIX)
      # gbenchmark requires pthread when compiled on a Unix machine
      target_link_libraries(${target_name} PRIVATE pthread)
    endif()

    target_link_libraries(${target_name} PRIVATE ${_ARG_LINK_LIBRARIES})

    target_compile_definitions(
      ${target_name}
      PRIVATE ${_ARG_COMPILE_DEFINITIONS}
    )

    foreach(dart_lib ${_ARG_LINK_DART_LIBRARIES})
      if(NOT TARGET ${dart_lib})
        message(FATAL_ERROR "Invalid target: ${dart_lib}")
      endif()
      target_link_libraries(${target_name} PRIVATE ${dart_lib})
    endforeach()

    set_target_properties(
      ${target_name}
      PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${DART_BINARY_DIR}/bin"
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
    set(${_ARG_TEST_LIST} "${${_ARG_TEST_LIST}}" PARENT_SCOPE)
  endif()

  dart_format_add(${_ARG_SOURCES})
endfunction()
