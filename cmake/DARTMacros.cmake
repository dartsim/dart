#===============================================================================
# Appends items to a cached list.
# Usage:
#   dart_append_to_cached_string(_string _cacheDesc [items...])
#===============================================================================
macro(dart_append_to_cached_string _string _cacheDesc)
  foreach(newItem ${ARGN})
    set(${_string} "${${_string}}${newItem}" CACHE INTERNAL ${_cacheDesc} FORCE)
  endforeach()
endmacro()

#===============================================================================
# Get list of file names give list of full paths.
# Usage:
#   dart_get_filename_components(_var _cacheDesc [items...])
#===============================================================================
macro(dart_get_filename_components _var _cacheDesc)
  set(${_var} "" CACHE INTERNAL ${_cacheDesc} FORCE)
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

#===============================================================================
# DEPRECATED in 6.7 (see #1081)
#
# Generate header file list to a cached list.
# Usage:
#   dart_generate_include_header_list(_var _target_dir _cacheDesc [headers...])
#===============================================================================
macro(dart_generate_include_header_list _var _target_dir _cacheDesc)
  set(${_var} "" CACHE INTERNAL ${_cacheDesc} FORCE)
  foreach(header ${ARGN})
    dart_append_to_cached_string(
      ${_var}
      ${_cacheDesc}"_HEADERS"
      "#include \"${_target_dir}${header}\"\n"
    )
  endforeach()
endmacro()

#===============================================================================
# Generate header file.
# Usage:
#   dart_generate_include_header_file(file_path target_dir [headers...])
#===============================================================================
macro(dart_generate_include_header_file file_path target_dir)
  file(WRITE ${file_path} "// Automatically generated file by cmake\n\n")
  foreach(header ${ARGN})
    file(APPEND ${file_path} "#include \"${target_dir}${header}\"\n")
  endforeach()
endmacro()

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
