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
    SOVERSION "${KIDO_MAJOR_VERSION}.${KIDO_MINOR_VERSION}"
    VERSION "${KIDO_VERSION}"
  )
endmacro()

