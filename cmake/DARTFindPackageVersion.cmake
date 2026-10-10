# Check versions after discovery because some config files only accept their
# own major version, and CMake's FindBullet does not check versions at all.
function(dart_check_package_version package minimum)
  string(TOUPPER "${package}" package_upper)
  if(NOT ${package}_FOUND AND NOT ${package_upper}_FOUND)
    return()
  endif()
  set(version "${${package}_VERSION}")
  if(NOT version)
    set(version "${${package_upper}_VERSION}")
  endif()
  # CMake compares numeric prefixes, including releases such as ImGui 1.91.9b.
  if(NOT version MATCHES "^[0-9]+(\\.[0-9]+)*" OR version VERSION_LESS minimum)
    if(NOT version)
      set(version "unknown")
    endif()
    message(
      STATUS
      "${package} >= ${minimum} is required; found version ${version}"
    )
    set(${package}_FOUND FALSE PARENT_SCOPE)
    set(${package_upper}_FOUND FALSE PARENT_SCOPE)
  endif()
endfunction()

# Use only the discovered package's include paths, not unrelated installations.
function(dart_read_header_version output header definition)
  set(${output} "" PARENT_SCOPE)
  foreach(include_dir IN LISTS ARGN)
    if(include_dir MATCHES "^\\$<BUILD_INTERFACE:(.+)>$")
      set(include_dir "${CMAKE_MATCH_1}")
    endif()
    if(EXISTS "${include_dir}/${header}")
      file(
        STRINGS "${include_dir}/${header}"
        version_line
        REGEX "^#[ \t]*define[ \t]+${definition}[ \t]+"
      )
      if(version_line MATCHES "${definition}[ \t]+\"?([0-9]+(\\.[0-9]+)*)")
        set(${output} "${CMAKE_MATCH_1}" PARENT_SCOPE)
        return()
      endif()
    endif()
  endforeach()
endfunction()
