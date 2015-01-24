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
# Copied from https://bitbucket.org/osrf/gazebo/pull-request/638 and will be
# removed by DART 5.0
#===============================================================================
macro (dt_install_includes _subdir)
  install(FILES ${ARGN} DESTINATION include/dart/${_subdir} COMPONENT headers)
endmacro()

#===============================================================================
# Deprecated header files
# Install until next gazebo version on case-sensitive filesystems
# Copied from https://bitbucket.org/osrf/gazebo/pull-request/638 and will be
# removed by DART 5.0
#===============================================================================
macro(dt_issue_303 _name _output_name)
  if (FILESYSTEM_CASE_SENSITIVE)
    if (${DART_VERSION} VERSION_GREATER 4.3)
      message(WARNING "Installing deprecated ${_name}.h. This should be removed in DART 5.0.")
    endif()
    set(generated_file "${CMAKE_CURRENT_BINARY_DIR}/${_name}.h")
    execute_process(
      COMMAND bash ${PROJECT_SOURCE_DIR}/tools/issue_303_generator.bash ${_name} ${_output_name}
      OUTPUT_FILE ${generated_file}
    )
    string(TOLOWER ${_name} nameLower)
    dt_install_includes(${nameLower} ${generated_file})
  endif()
endmacro()

