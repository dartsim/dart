# Copyright (c) 2011-2025, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

# Bullet. Force MODULE mode to use the FindBullet.cmake file distributed with
# CMake. Otherwise, we may end up using the BulletConfig.cmake file distributed
# with Bullet, which uses relative paths and may break transitive dependencies.
#
# Single-precision Bullet's fixed GJK tolerances drop or tilt contacts of
# centimeter-scale shapes (gz-physics#1117), so DART prefers the -float64
# libraries that Ubuntu and conda-forge install beside the single-precision
# ones. A process must not load both precisions: building DART records the
# variant it links, and an installed DART passes it back through
# DART_BULLET_REQUIRE_FLOAT64 (TRUE, FALSE, or empty when unknown) together
# with DART_BULLET_REQUIRE_VERSION. DART_BULLET_REQUIRE_PRECISION independently
# requires the recorded scalar ABI. DART_BULLET_BUILT_PRECISION is double or
# single when an existing Bullet target links Bullet libraries that this
# project builds, which the precision probe cannot link.
include("${CMAKE_CURRENT_LIST_DIR}/DARTFindPackageVersion.cmake")

set(DART_BULLET_BUILT_PRECISION "")
set(_DART_BULLET_GRAPH_UNKNOWN FALSE)
set(_DART_BULLET_IMPORTED_FILES)
set(_DART_BULLET_INCLUDES)
set(_DART_BULLET_IMPORTED_VARIANT)
set(_DART_BULLET_PROBE_CONFIGS)

set(
  _dart_bullet_name_regex
  "^(lib)?(Bullet(Collision|Dynamics|Math|SoftBody)|LinearMath)"
)

# Sets <out> to float64, single (unsuffixed), or mixed by filename only.
# The scalar ABI is checked separately; an empty result means no named files.
# Targets, such as bundled Bullet's, are not files.
function(_dart_bullet_variant out)
  set(float64 FALSE)
  set(single FALSE)
  foreach(library IN LISTS ARGN)
    if(TARGET "${library}")
      continue()
    endif()
    get_filename_component(name "${library}" NAME)
    if(name MATCHES "${_dart_bullet_name_regex}")
      if(name MATCHES "-float64")
        set(float64 TRUE)
      else()
        set(single TRUE)
      endif()
    endif()
  endforeach()
  set(variant "")
  if(float64 AND single)
    set(variant mixed)
  elseif(float64)
    set(variant float64)
  elseif(single)
    set(variant single)
  endif()
  set(${out} "${variant}" PARENT_SCOPE)
endfunction()

# pkg-config rewrites -I/-L paths using its consumer's sysroot, not the builder's.
function(_dart_bullet_pc_path out path kind)
  set(root "${CMAKE_SYSROOT}")
  if(DEFINED CMAKE_SYSROOT_${kind})
    set(root "${CMAKE_SYSROOT_${kind}}")
  endif()
  if(root AND IS_ABSOLUTE "${path}")
    string(REGEX REPLACE "/+$" "" root "${root}")
    string(FIND "${path}" "${root}/" offset)
    if(path STREQUAL root)
      set(path "/")
    elseif(offset EQUAL 0)
      string(LENGTH "${root}" length)
      string(SUBSTRING "${path}" ${length} -1 path)
    endif()
  endif()
  set(${out} "${path}" PARENT_SCOPE)
endfunction()

function(_dart_bullet_pc_escape out token)
  string(REPLACE "\\" "\\\\" token "${token}")
  string(REPLACE " " "\\ " token "${token}")
  string(REPLACE "\t" "\\\t" token "${token}")
  set(${out} "${token}" PARENT_SCOPE)
endfunction()

# Records the variant that DART links while building it, and pkg-config
# flags for the selected includes, link directories, and libraries, without
# FindBullet's Debug entries, which dart.pc names when no bullet-float64
# module describes them.
function(_dart_bullet_record variant)
  cmake_parse_arguments(
    PARSE_ARGV
    1
    arg
    ""
    ""
    "INCLUDES;LIBRARIES;LINK_DIRECTORIES"
  )
  set(uses "")
  if(variant STREQUAL "float64")
    set(uses TRUE)
  elseif(variant STREQUAL "single")
    set(uses FALSE)
  endif()
  set(
    DART_BULLET_USES_FLOAT64
    "${uses}"
    CACHE INTERNAL
    "Whether DART links Bullet's -float64 libraries; empty when unknown"
  )
  set(libraries)
  set(skip FALSE)
  foreach(library IN LISTS arg_LIBRARIES)
    if(skip)
      set(skip FALSE)
    elseif(library STREQUAL "debug")
      set(skip TRUE)
    elseif(
      NOT library MATCHES "^(optimized|general)$"
      AND NOT TARGET "${library}"
    )
      list(APPEND libraries "${library}")
    endif()
  endforeach()
  set(includes ${arg_INCLUDES})
  list(REMOVE_DUPLICATES includes)
  set(cflags "")
  foreach(include_dir IN LISTS includes)
    _dart_bullet_pc_path(path "${include_dir}" COMPILE)
    _dart_bullet_pc_escape(path "${path}")
    string(APPEND cflags " -I${path}")
  endforeach()
  set(libs "")
  set(directories ${arg_LINK_DIRECTORIES})
  list(REMOVE_DUPLICATES directories)
  foreach(directory IN LISTS directories)
    _dart_bullet_pc_path(path "${directory}" LINK)
    _dart_bullet_pc_escape(path "${path}")
    string(APPEND libs " -L${path}")
  endforeach()
  set(error "")
  set(framework FALSE)
  foreach(library IN LISTS libraries)
    _dart_bullet_pc_path(path "${library}" LINK)
    if(NOT path STREQUAL library)
      # GNU-compatible ELF linkers can preserve the exact selected archive/DSO.
      # -l<stem> could silently substitute a shared library for a selected archive.
      if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
        set(
          error
          "Cannot serialize the selected Bullet library '${library}' for a relocatable sysroot on ${CMAKE_SYSTEM_NAME}"
        )
        break()
      endif()
      get_filename_component(directory "${path}" DIRECTORY)
      get_filename_component(name "${path}" NAME)
      _dart_bullet_pc_escape(directory "${directory}")
      _dart_bullet_pc_escape(name "${name}")
      string(APPEND libs " -L${directory} -l:${name}")
    else()
      # CMake turns a bare library name into a search flag. Keep flags, framework
      # arguments and paths literal, including repeated framework flag pairs.
      if(
        NOT framework
        AND NOT IS_ABSOLUTE "${path}"
        AND NOT path MATCHES "^[-/]|[/\\\\]"
      )
        set(path "-l${path}")
      endif()
      _dart_bullet_pc_escape(path "${path}")
      string(APPEND libs " ${path}")
    endif()
    set(framework FALSE)
    if(library STREQUAL "-framework" OR library STREQUAL "-weak_framework")
      set(framework TRUE)
    endif()
  endforeach()
  set(
    DART_BULLET_PC_ERROR
    "${error}"
    CACHE INTERNAL
    "Unsupported Bullet pkg-config serialization"
  )
  set(
    DART_BULLET_PC_CFLAGS
    "${cflags}"
    CACHE INTERNAL
    "Bullet's pkg-config flags"
  )
  set(
    DART_BULLET_PC_LIBS
    "${libs}"
    CACHE INTERNAL
    "Bullet's pkg-config libraries"
  )
endfunction()

# Sets <out> to the import library or location of an imported <target> for
# the configuration <suffix> (such as _RELEASE, or empty), if it has one.
function(_dart_bullet_location out target suffix)
  get_target_property(location "${target}" IMPORTED_IMPLIB${suffix})
  if(NOT location)
    get_target_property(location "${target}" IMPORTED_LOCATION${suffix})
  endif()
  if(NOT location)
    set(location "")
  endif()
  set(${out} "${location}" PARENT_SCOPE)
endfunction()

# Resolve the file CMake links for a configuration, including explicit mappings.
function(_dart_bullet_config_location out target config)
  string(TOUPPER "${config}" config)
  get_property(
    mapped
    TARGET "${target}"
    PROPERTY MAP_IMPORTED_CONFIG_${config}
    SET
  )
  if(mapped)
    get_target_property(configs "${target}" MAP_IMPORTED_CONFIG_${config})
  else()
    get_target_property(configs "${target}" IMPORTED_CONFIGURATIONS)
    if(NOT configs)
      set(configs)
    endif()
    set(configs "${config};;${configs}")
  endif()
  # Preserve empty mapping entries: CMake uses the configuration-less file.
  string(REPLACE ";" ";_" configs "_${configs}")
  set(location "")
  set(selected_suffix "")
  foreach(suffix IN LISTS configs)
    if(suffix STREQUAL "_")
      set(suffix "")
    endif()
    string(TOUPPER "${suffix}" suffix)
    _dart_bullet_location(location "${target}" "${suffix}")
    if(location)
      set(selected_suffix "${suffix}")
      break()
    endif()
  endforeach()
  set(${out} "${location}" PARENT_SCOPE)
  if(ARGC GREATER 3)
    set(${ARGV3} "${selected_suffix}" PARENT_SCOPE)
  endif()
endfunction()

# An explicitly set modern interface, even an empty one, overrides the legacy
# imported interface of concrete libraries. Interface-only targets ignore the
# legacy property. Legacy configuration selection follows the imported file.
function(_dart_bullet_link_interface out target config)
  get_property(modern TARGET "${target}" PROPERTY INTERFACE_LINK_LIBRARIES SET)
  set(property INTERFACE_LINK_LIBRARIES)
  if(NOT modern)
    get_target_property(imported "${target}" IMPORTED)
    get_target_property(type "${target}" TYPE)
    if(imported AND NOT type STREQUAL "INTERFACE_LIBRARY")
      _dart_bullet_config_location(location "${target}" "${config}" suffix)
      set(property IMPORTED_LINK_INTERFACE_LIBRARIES${suffix})
      get_property(configured TARGET "${target}" PROPERTY "${property}" SET)
      if(NOT configured)
        set(property IMPORTED_LINK_INTERFACE_LIBRARIES)
      endif()
    endif()
  endif()
  get_target_property(value "${target}" "${property}")
  if(NOT value)
    set(value "")
  endif()
  set(${out} "${value}" PARENT_SCOPE)
endfunction()

if(NOT DART_USE_SYSTEM_BULLET)
  if(TARGET BulletCollision AND TARGET LinearMath)
    if(DEFINED DART_BULLET_SOURCE_DIR)
      set(_bullet_include_dir "${DART_BULLET_SOURCE_DIR}/src")
    elseif(DEFINED BULLET_PHYSICS_SOURCE_DIR)
      set(_bullet_include_dir "${BULLET_PHYSICS_SOURCE_DIR}/src")
    endif()
    if(_bullet_include_dir)
      set(BULLET_INCLUDE_DIRS "${_bullet_include_dir}")
    endif()
    set(BULLET_LIBRARIES BulletCollision LinearMath)
    set(BULLET_FOUND TRUE)
    set(Bullet_FOUND TRUE)
    unset(_bullet_include_dir)
  endif()
endif()

if(DART_USE_SYSTEM_BULLET OR NOT BULLET_FOUND)
  # CMake's FindBullet ignores COMPONENTS and always returns all four
  # libraries. DART's collision backend uses only Collision and LinearMath,
  # so narrow BULLET_LIBRARIES to those two after discovery.
  find_package(Bullet MODULE QUIET)
  if(BULLET_FOUND OR Bullet_FOUND)
    set(_bullet_narrowed_libraries "")
    foreach(_bullet_lib_var BULLET_COLLISION_LIBRARY BULLET_MATH_LIBRARY)
      if(${_bullet_lib_var}_DEBUG)
        list(
          APPEND _bullet_narrowed_libraries
          optimized
          ${${_bullet_lib_var}}
          debug
          ${${_bullet_lib_var}_DEBUG}
        )
      elseif(${_bullet_lib_var})
        list(APPEND _bullet_narrowed_libraries ${${_bullet_lib_var}})
      endif()
    endforeach()
    if(_bullet_narrowed_libraries)
      set(BULLET_LIBRARIES "${_bullet_narrowed_libraries}")
    endif()
    unset(_bullet_narrowed_libraries)
    unset(_bullet_lib_var)
  endif()
endif()

macro(_dart_bullet_check_version includes)
  set(BULLET_VERSION "")
  dart_read_header_version(_bullet_header_version
    LinearMath/btScalar.h BT_BULLET_VERSION ${includes}
  )
  if(_bullet_header_version MATCHES "^[0-9]+$")
    math(EXPR _bullet_major "${_bullet_header_version} / 100")
    math(EXPR _bullet_minor "${_bullet_header_version} % 100")
    set(BULLET_VERSION "${_bullet_major}.${_bullet_minor}")
  endif()
  set(Bullet_VERSION "${BULLET_VERSION}")
  # Upstream Bullet 3.06 and 3.07 still report 305 (Ubuntu 22.04 ships 3.06).
  # DART built against Bullet 3.17 or newer uses API that 3.17 introduced.
  set(_dart_bullet_minimum 3.05)
  if(DART_BULLET_REQUIRE_VERSION)
    set(_dart_bullet_minimum "${DART_BULLET_REQUIRE_VERSION}")
  endif()
  dart_check_package_version(Bullet ${_dart_bullet_minimum})
  if(NOT DEFINED DART_BULLET_REQUIRE_FLOAT64)
    set(_dart_bullet_minimum 3.05)
    if(BULLET_VERSION VERSION_GREATER_EQUAL 3.17)
      set(_dart_bullet_minimum 3.17)
    endif()
    set(
      DART_BULLET_MINIMUM_VERSION
      ${_dart_bullet_minimum}
      CACHE INTERNAL
      "Minimum Bullet version for an installed DART"
    )
  endif()
  unset(_dart_bullet_minimum)
  unset(_bullet_header_version)
  unset(_bullet_major)
  unset(_bullet_minor)
endmacro()

if((BULLET_FOUND OR Bullet_FOUND) AND NOT TARGET Bullet)
  _dart_bullet_check_version("${BULLET_INCLUDE_DIRS}")
endif()

if((BULLET_FOUND OR Bullet_FOUND) AND NOT TARGET Bullet)
  if(DEFINED DART_BULLET_REQUIRE_FLOAT64)
    set(_dart_bullet_want_float64 "${DART_BULLET_REQUIRE_FLOAT64}")
  elseif(DART_BULLET_FLOAT64 AND BULLET_VERSION VERSION_GREATER_EQUAL 3.17)
    # Float heightfields need Bullet 3.17 in double precision.
    set(_dart_bullet_want_float64 TRUE)
  else()
    set(_dart_bullet_want_float64 FALSE)
  endif()
  # Bundled Bullet supplies targets; only FindBullet's library files have
  # -float64 siblings. Leave FindBullet's variables alone: consumers may run
  # it for their own targets.
  set(_dart_bullet_files TRUE)
  foreach(_dart_bullet_item IN LISTS BULLET_LIBRARIES)
    if(TARGET "${_dart_bullet_item}")
      set(_dart_bullet_files FALSE)
    endif()
  endforeach()
  set(_dart_bullet_libraries ${BULLET_LIBRARIES})
  if(
    _dart_bullet_want_float64
    AND _dart_bullet_files
    AND BULLET_COLLISION_LIBRARY
  )
    get_filename_component(
      _dart_bullet_dir
      "${BULLET_COLLISION_LIBRARY}"
      DIRECTORY
    )
    # Replace the collision and math libraries, keeping their order and Debug
    # variants; keep the selection when the -float64 libraries or their Debug
    # variants are missing. Search beside each library's Release and Debug
    # files, which multi-config layouts separate.
    set(_dart_bullet_needs_debug FALSE)
    if("${BULLET_LIBRARIES}" MATCHES "(^|;)debug;")
      set(_dart_bullet_needs_debug TRUE)
    endif()
    set(_dart_bullet_float64_libraries)
    set(_dart_bullet_complete TRUE)
    foreach(_dart_bullet_name COLLISION:BulletCollision MATH:LinearMath)
      string(REPLACE ":" ";" _dart_bullet_name "${_dart_bullet_name}")
      list(GET _dart_bullet_name 0 _dart_bullet_var)
      list(GET _dart_bullet_name 1 _dart_bullet_name)
      set(_dart_bullet_hints "${_dart_bullet_dir}")
      set(_dart_bullet_debug_hints "${_dart_bullet_dir}")
      foreach(
        _dart_bullet_item
        BULLET_${_dart_bullet_var}_LIBRARY
        BULLET_${_dart_bullet_var}_LIBRARY_DEBUG
        BULLET_COLLISION_LIBRARY_DEBUG
      )
        if(${_dart_bullet_item})
          get_filename_component(
            _dart_bullet_item_dir
            "${${_dart_bullet_item}}"
            DIRECTORY
          )
          list(APPEND _dart_bullet_debug_hints "${_dart_bullet_item_dir}")
          if(_dart_bullet_item STREQUAL "BULLET_${_dart_bullet_var}_LIBRARY")
            list(APPEND _dart_bullet_hints "${_dart_bullet_item_dir}")
          endif()
        endif()
      endforeach()
      find_library(
        _dart_bullet_library
        ${_dart_bullet_name}-float64
        HINTS ${_dart_bullet_hints}
        NO_DEFAULT_PATH
        NO_CACHE
      )
      find_library(
        _dart_bullet_debug
        NAMES ${_dart_bullet_name}-float64_Debug ${_dart_bullet_name}-float64_d
        HINTS ${_dart_bullet_debug_hints}
        NO_DEFAULT_PATH
        NO_CACHE
      )
      if(NOT _dart_bullet_library)
        set(_dart_bullet_complete FALSE)
      elseif(_dart_bullet_debug)
        list(
          APPEND _dart_bullet_float64_libraries
          optimized
          "${_dart_bullet_library}"
          debug
          "${_dart_bullet_debug}"
        )
      elseif(_dart_bullet_needs_debug)
        set(_dart_bullet_complete FALSE)
      else()
        list(APPEND _dart_bullet_float64_libraries "${_dart_bullet_library}")
      endif()
      unset(_dart_bullet_library)
      unset(_dart_bullet_debug)
    endforeach()
    if(_dart_bullet_complete)
      set(_dart_bullet_libraries ${_dart_bullet_float64_libraries})
    endif()
    unset(_dart_bullet_dir)
    unset(_dart_bullet_name)
    unset(_dart_bullet_var)
    unset(_dart_bullet_hints)
    unset(_dart_bullet_debug_hints)
    unset(_dart_bullet_item_dir)
    unset(_dart_bullet_needs_debug)
    unset(_dart_bullet_float64_libraries)
    unset(_dart_bullet_complete)
  endif()
  # Explicitly selected libraries count as well.
  _dart_bullet_variant(_dart_bullet_found ${_dart_bullet_libraries})
  if(_dart_bullet_found STREQUAL "mixed")
    message(
      WARNING
      "The selected Bullet libraries mix -float64 and single-precision ones; "
      "a process must not load both Bullet precisions."
    )
    set(BULLET_FOUND FALSE)
    set(Bullet_FOUND FALSE)
  elseif(
    DART_BULLET_REQUIRE_FLOAT64
    AND NOT _dart_bullet_found STREQUAL "float64"
  )
    message(
      WARNING
      "DART was built with Bullet's -float64 libraries, which were not found "
      "beside '${BULLET_COLLISION_LIBRARY}'."
    )
    set(BULLET_FOUND FALSE)
    set(Bullet_FOUND FALSE)
  elseif(
    DART_BULLET_REQUIRE_FLOAT64 STREQUAL "FALSE"
    AND _dart_bullet_found STREQUAL "float64"
  )
    message(
      WARNING
      "DART was built with single-precision Bullet, but the selected Bullet "
      "libraries are -float64 ones."
    )
    set(BULLET_FOUND FALSE)
    set(Bullet_FOUND FALSE)
  else()
    add_library(Bullet INTERFACE IMPORTED)
    target_include_directories(Bullet INTERFACE ${BULLET_INCLUDE_DIRS})
    target_link_libraries(Bullet INTERFACE ${_dart_bullet_libraries})
  endif()
  if(NOT DEFINED DART_BULLET_REQUIRE_FLOAT64)
    if(_dart_bullet_files)
      _dart_bullet_record(
        "${_dart_bullet_found}"
        INCLUDES ${BULLET_INCLUDE_DIRS}
        LIBRARIES ${_dart_bullet_libraries}
      )
    else()
      _dart_bullet_record("${_dart_bullet_found}")
    endif()
  endif()
  unset(_dart_bullet_want_float64)
  unset(_dart_bullet_files)
  unset(_dart_bullet_item)
  unset(_dart_bullet_libraries)
  unset(_dart_bullet_found)
endif()

# Resolve only the supported active expressions. Compile usage excludes LINK_ONLY;
# link usage includes it. Both use the property's owning target for CONFIG maps.
function(
  _dart_bullet_evaluate_items
  out
  unknown
  config
  owner
  link_only
)
  string(TOUPPER "${config}" _dart_bullet_build_type)
  set(_dart_bullet_owner "${owner}")
  set(_dart_bullet_queue ${ARGN})
  set(_dart_bullet_unknown FALSE)
  set(result)
  while(_dart_bullet_queue)
    list(GET _dart_bullet_queue 0 _dart_bullet_item)
    list(REMOVE_AT _dart_bullet_queue 0)
    set(_dart_bullet_active_configs "_${_dart_bullet_build_type}")
    get_target_property(_dart_bullet_imported "${_dart_bullet_owner}" IMPORTED)
    if(_dart_bullet_imported)
      get_property(
        _dart_bullet_mapped
        TARGET "${_dart_bullet_owner}"
        PROPERTY MAP_IMPORTED_CONFIG_${_dart_bullet_build_type}
        SET
      )
      if(_dart_bullet_mapped)
        get_target_property(
          _dart_bullet_mapped
          "${_dart_bullet_owner}"
          MAP_IMPORTED_CONFIG_${_dart_bullet_build_type}
        )
        string(TOUPPER "${_dart_bullet_mapped}" _dart_bullet_mapped)
        # Empty mappings select a default location, not an empty CONFIG match.
        list(FILTER _dart_bullet_mapped EXCLUDE REGEX "^$")
        list(TRANSFORM _dart_bullet_mapped PREPEND "_")
        list(APPEND _dart_bullet_active_configs ${_dart_bullet_mapped})
      endif()
    endif()
    # Rejoin generator expressions that list splitting cut at semicolons.
    set(_dart_bullet_part "${_dart_bullet_item}")
    set(_dart_bullet_depth 0)
    while(TRUE)
      string(REGEX MATCHALL "\\$<" _dart_bullet_opens "${_dart_bullet_part}")
      string(REGEX MATCHALL ">" _dart_bullet_closes "${_dart_bullet_part}")
      list(LENGTH _dart_bullet_opens _dart_bullet_open)
      list(LENGTH _dart_bullet_closes _dart_bullet_close)
      math(
        EXPR
        _dart_bullet_depth
        "${_dart_bullet_depth} + ${_dart_bullet_open} - ${_dart_bullet_close}"
      )
      if(_dart_bullet_depth LESS_EQUAL 0 OR NOT _dart_bullet_queue)
        break()
      endif()
      list(GET _dart_bullet_queue 0 _dart_bullet_part)
      list(REMOVE_AT _dart_bullet_queue 0)
      string(APPEND _dart_bullet_item ";${_dart_bullet_part}")
    endwhile()
    # Unwrap $<LINK_ONLY:...>, $<BUILD_INTERFACE:...>, true $<BOOL:...> and
    # $<CONFIG:...> conditions, and queue the elements of a list payload.
    # Only shorter payloads are queued, so the walk ends. Skip payloads that
    # are empty in this build, such as $<INSTALL_INTERFACE:...> and $<0:...>,
    # and reject unevaluated potentially active expressions below.
    set(_dart_bullet_part "${_dart_bullet_item}")
    set(_dart_bullet_other FALSE)
    while(
      _dart_bullet_item
        MATCHES
        "^\\$<(\\$<([^<>]|\\$<[^<>]*>)*>|[A-Za-z0-9_]+):(.*)>$"
    )
      set(_dart_bullet_item "${CMAKE_MATCH_3}")
      set(_dart_bullet_expression "${CMAKE_MATCH_1}")
      string(TOUPPER "${_dart_bullet_expression}" _dart_bullet_value)
      if(_dart_bullet_value MATCHES "^\\$<(NOT:\\$<)?CONFIG:([^<>]*)>")
        string(REPLACE "," ";" _dart_bullet_configs "${CMAKE_MATCH_2}")
        set(_dart_bullet_matches_config FALSE)
        foreach(_dart_bullet_active_config IN LISTS _dart_bullet_active_configs)
          string(
            SUBSTRING "${_dart_bullet_active_config}"
            1
            -1
            _dart_bullet_active_config
          )
          if(
            _dart_bullet_active_config STREQUAL _dart_bullet_configs
            OR _dart_bullet_active_config IN_LIST _dart_bullet_configs
          )
            set(_dart_bullet_matches_config TRUE)
          endif()
        endforeach()
        if(_dart_bullet_matches_config)
          if(_dart_bullet_value MATCHES "^\\$<NOT:")
            set(_dart_bullet_other TRUE)
          endif()
        elseif(NOT _dart_bullet_value MATCHES "^\\$<NOT:")
          set(_dart_bullet_other TRUE)
        endif()
        if(_dart_bullet_other)
          set(_dart_bullet_item "")
          break()
        endif()
      elseif(_dart_bullet_value MATCHES "^\\$<BOOL:([^<>]*)>$")
        if(
          CMAKE_MATCH_1
            MATCHES
            "^(|0|FALSE|OFF|N|NO|IGNORE|NOTFOUND|.*-NOTFOUND)$"
        )
          set(_dart_bullet_item "")
          break()
        endif()
      elseif(_dart_bullet_expression MATCHES "^\\$<PLATFORM_ID:([^<>]*)>$")
        string(REPLACE "," ";" _dart_bullet_platforms "${CMAKE_MATCH_1}")
        if(NOT CMAKE_SYSTEM_NAME IN_LIST _dart_bullet_platforms)
          set(_dart_bullet_item "")
          break()
        endif()
      elseif(_dart_bullet_value STREQUAL "TARGET_NAME_IF_EXISTS")
        if(NOT TARGET "${_dart_bullet_item}")
          set(_dart_bullet_item "")
          break()
        endif()
      elseif(_dart_bullet_value STREQUAL "LINK_ONLY" AND NOT link_only)
        set(_dart_bullet_item "")
        break()
      elseif(_dart_bullet_value MATCHES "^(INSTALL_INTERFACE|0)$")
        set(_dart_bullet_item "")
        break()
      elseif(
        NOT
          _dart_bullet_value
            MATCHES
            "^(LINK_ONLY|BUILD_INTERFACE|BUILD_LOCAL_INTERFACE|1)$"
      )
        set(_dart_bullet_unknown TRUE)
        set(_dart_bullet_item "")
        break()
      endif()
    endwhile()
    if(_dart_bullet_item MATCHES "\\$<")
      set(_dart_bullet_unknown TRUE)
      continue()
    endif()
    if(_dart_bullet_item STREQUAL "")
      continue()
    endif()
    if(
      _dart_bullet_item MATCHES ";"
      AND NOT _dart_bullet_item STREQUAL _dart_bullet_part
    )
      list(PREPEND _dart_bullet_queue ${_dart_bullet_item})
      continue()
    endif()
    list(APPEND result "${_dart_bullet_item}")
  endwhile()
  set(${out} "${result}" PARENT_SCOPE)
  set(${unknown} "${_dart_bullet_unknown}" PARENT_SCOPE)
endfunction()

# Project-built libraries cannot be linked by try_compile. Inspect their actual
# compile definitions, including usage inherited from private/public dependencies.
function(_dart_bullet_built_precision out unknown target config)
  set(queue "${target}")
  set(seen)
  set(definitions)
  set(unverified FALSE)
  while(queue)
    list(POP_FRONT queue item)
    if(NOT TARGET "${item}" OR item IN_LIST seen)
      continue()
    endif()
    list(APPEND seen "${item}")
    set(value)
    if(item STREQUAL target)
      get_target_property(directory "${item}" SOURCE_DIR)
      get_directory_property(value DIRECTORY "${directory}" COMPILE_DEFINITIONS)
      get_target_property(local "${item}" COMPILE_DEFINITIONS)
      if(local)
        list(APPEND value ${local})
      endif()
      get_target_property(links "${item}" LINK_LIBRARIES)
      if(NOT links)
        set(links)
      endif()
    else()
      get_target_property(value "${item}" INTERFACE_COMPILE_DEFINITIONS)
      if(NOT value)
        set(value)
      endif()
      _dart_bullet_link_interface(links "${item}" "${config}")
    endif()
    _dart_bullet_evaluate_items(active unresolved "${config}" "${item}" FALSE ${value})
    list(APPEND definitions ${active})
    if(unresolved)
      set(unverified TRUE)
    endif()
    _dart_bullet_evaluate_items(active unresolved "${config}" "${item}" FALSE ${links})
    list(APPEND queue ${active})
    if(unresolved)
      set(unverified TRUE)
    endif()
  endwhile()
  set(precision single)
  if(definitions MATCHES "(^|;)BT_USE_DOUBLE_PRECISION(=[^;]*)?(;|$)")
    set(precision double)
  else()
    # A public-only scalar definition disagrees with this library's own build.
    get_target_property(value "${target}" INTERFACE_COMPILE_DEFINITIONS)
    if(value)
      _dart_bullet_evaluate_items(active unresolved "${config}" "${target}" FALSE ${value})
      if(
        unresolved
        OR active MATCHES "(^|;)BT_USE_DOUBLE_PRECISION(=[^;]*)?(;|$)"
      )
        set(unverified TRUE)
      endif()
    endif()
  endif()
  set(${out} "${precision}" PARENT_SCOPE)
  set(${unknown} "${unverified}" PARENT_SCOPE)
endfunction()

# Use the same traversal for package metadata and each configuration's ABI probe.
# Classification and probes use the same active configuration walks.
function(_dart_bullet_collect_graph config all_configs)
  if(all_configs)
    get_property(multi_config GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
    set(configs "${CMAKE_BUILD_TYPE}")
    if(multi_config)
      set(configs "${CMAKE_CONFIGURATION_TYPES}")
    endif()
    # Prefix each list entry to preserve an empty configuration during transport.
    string(REPLACE ";" ";_" configs "_${configs}")
    set(files)
    set(built_precisions)
    set(unknown FALSE)
    foreach(active_config IN LISTS configs)
      string(SUBSTRING "${active_config}" 1 -1 active_config)
      _dart_bullet_collect_graph("${active_config}" FALSE)
      list(APPEND files ${_dart_bullet_files})
      list(APPEND built_precisions ${_dart_bullet_built_precisions})
      if(_dart_bullet_unknown)
        set(unknown TRUE)
      endif()
    endforeach()
    # Keep the packaging selection independent of the last configuration walked.
    _dart_bullet_collect_graph("${config}" FALSE)
    set(_dart_bullet_probe_configs "${configs}" PARENT_SCOPE)
    foreach(output files built_precisions unknown)
      set(_dart_bullet_${output} "${${output}}" PARENT_SCOPE)
    endforeach()
    foreach(output pc_files includes link_directories)
      set(_dart_bullet_${output} "${_dart_bullet_${output}}" PARENT_SCOPE)
    endforeach()
    return()
  endif()
  string(TOUPPER "${config}" _dart_bullet_build_type)
  set(_dart_bullet_queue ${ARGN})
  if(NOT _dart_bullet_queue)
    set(_dart_bullet_queue Bullet)
  endif()
  set(_dart_bullet_seen)
  set(_dart_bullet_files)
  set(_dart_bullet_pc_files)
  set(_dart_bullet_includes)
  set(_dart_bullet_link_directories)
  set(_dart_bullet_unknown FALSE)
  set(_dart_bullet_built_precisions)
  set(_dart_bullet_has_build_targets FALSE)
  while(_dart_bullet_queue)
    list(GET _dart_bullet_queue 0 _dart_bullet_item)
    list(REMOVE_AT _dart_bullet_queue 0)
    if(NOT TARGET "${_dart_bullet_item}")
      list(APPEND _dart_bullet_files "${_dart_bullet_item}")
      list(APPEND _dart_bullet_pc_files "${_dart_bullet_item}")
      continue()
    endif()
    if(_dart_bullet_item IN_LIST _dart_bullet_seen)
      continue()
    endif()
    list(APPEND _dart_bullet_seen "${_dart_bullet_item}")
    _dart_bullet_link_interface(_dart_bullet_value "${_dart_bullet_item}" "${config}")
    _dart_bullet_evaluate_items(_dart_bullet_value _dart_bullet_unresolved
      "${config}" "${_dart_bullet_item}" TRUE ${_dart_bullet_value}
    )
    list(PREPEND _dart_bullet_queue ${_dart_bullet_value})
    if(_dart_bullet_unresolved)
      set(_dart_bullet_unknown TRUE)
    endif()
    foreach(property INCLUDE_DIRECTORIES LINK_DIRECTORIES)
      get_target_property(
        _dart_bullet_value
        "${_dart_bullet_item}"
        INTERFACE_${property}
      )
      if(_dart_bullet_value)
        _dart_bullet_evaluate_items(_dart_bullet_value _dart_bullet_unresolved
          "${config}" "${_dart_bullet_item}" TRUE ${_dart_bullet_value}
        )
        if(_dart_bullet_unresolved)
          set(_dart_bullet_unknown TRUE)
        endif()
        foreach(_dart_bullet_dir IN LISTS _dart_bullet_value)
          if(IS_ABSOLUTE "${_dart_bullet_dir}")
            if(property STREQUAL "INCLUDE_DIRECTORIES")
              list(APPEND _dart_bullet_includes "${_dart_bullet_dir}")
            else()
              list(APPEND _dart_bullet_link_directories "${_dart_bullet_dir}")
            endif()
          endif()
        endforeach()
      endif()
    endforeach()
    get_target_property(_dart_bullet_imported "${_dart_bullet_item}" IMPORTED)
    if(NOT _dart_bullet_imported)
      get_target_property(_dart_bullet_type "${_dart_bullet_item}" TYPE)
      if(NOT _dart_bullet_type STREQUAL "INTERFACE_LIBRARY")
        set(_dart_bullet_has_build_targets TRUE)
      endif()
      # try_compile cannot link project-built Bullet libraries; inspect their
      # own definitions and inherited compile usage for the same configuration.
      get_target_property(_dart_bullet_value "${_dart_bullet_item}" TYPE)
      get_target_property(_dart_bullet_dir "${_dart_bullet_item}" NAME)
      if(
        NOT _dart_bullet_value STREQUAL "INTERFACE_LIBRARY"
        AND _dart_bullet_dir MATCHES "${_dart_bullet_name_regex}"
      )
        _dart_bullet_built_precision(_dart_bullet_precision _dart_bullet_unresolved
          "${_dart_bullet_item}" "${config}"
        )
        list(APPEND _dart_bullet_built_precisions "${_dart_bullet_precision}")
        if(_dart_bullet_unresolved)
          set(_dart_bullet_unknown TRUE)
        endif()
      endif()
      continue()
    endif()
    _dart_bullet_config_location(_dart_bullet_location "${_dart_bullet_item}"
      "${_dart_bullet_build_type}"
    )
    if(_dart_bullet_location)
      list(APPEND _dart_bullet_pc_files "${_dart_bullet_location}")
      list(APPEND _dart_bullet_files "${_dart_bullet_location}")
    else()
      get_target_property(_dart_bullet_type "${_dart_bullet_item}" TYPE)
      if(NOT _dart_bullet_type STREQUAL "INTERFACE_LIBRARY")
        set(_dart_bullet_unknown TRUE)
      endif()
    endif()
  endwhile()
  set(_dart_bullet_targets "${_dart_bullet_seen}")
  foreach(
    output
    targets
    files
    pc_files
    includes
    link_directories
    unknown
    built_precisions
    has_build_targets
  )
    set(_dart_bullet_${output} "${_dart_bullet_${output}}" PARENT_SCOPE)
  endforeach()
endfunction()

if(TARGET Bullet AND (BULLET_FOUND OR Bullet_FOUND))
  # dart.pc describes the chosen build type, defaulting to Release.
  set(_dart_bullet_build_type "${CMAKE_BUILD_TYPE}")
  if(NOT _dart_bullet_build_type)
    set(_dart_bullet_build_type Release)
  endif()
  _dart_bullet_collect_graph("${_dart_bullet_build_type}" TRUE)
  set(_DART_BULLET_INCLUDES "${_dart_bullet_includes}")
  _dart_bullet_check_version("${_dart_bullet_includes}")
  list(REMOVE_DUPLICATES _dart_bullet_built_precisions)
  list(LENGTH _dart_bullet_built_precisions _dart_bullet_precision_count)
  if(_dart_bullet_precision_count EQUAL 1)
    set(DART_BULLET_BUILT_PRECISION "${_dart_bullet_built_precisions}")
  endif()
  _dart_bullet_variant(_dart_bullet_found ${_dart_bullet_files})
  set(_dart_bullet_expected "")
  if(DART_BULLET_REQUIRE_FLOAT64 STREQUAL "TRUE")
    set(_dart_bullet_expected float64)
  elseif(DART_BULLET_REQUIRE_FLOAT64 STREQUAL "FALSE")
    set(_dart_bullet_expected single)
  endif()
  if(
    _dart_bullet_found STREQUAL "mixed"
    OR _dart_bullet_precision_count GREATER 1
    OR _dart_bullet_unknown
  )
    message(
      WARNING
      "The existing Bullet target has mixed or unverified precision metadata; "
      "a process must not load both Bullet precisions."
    )
    set(BULLET_FOUND FALSE)
    set(Bullet_FOUND FALSE)
  elseif(
    _dart_bullet_found
    AND _dart_bullet_expected
    AND NOT _dart_bullet_found STREQUAL _dart_bullet_expected
  )
    message(
      WARNING
      "The existing Bullet target links a different Bullet precision than "
      "DART was built with (-float64: ${DART_BULLET_REQUIRE_FLOAT64}); a "
      "process must not load both Bullet precisions."
    )
    set(BULLET_FOUND FALSE)
    set(Bullet_FOUND FALSE)
  elseif(NOT _dart_bullet_found AND _dart_bullet_expected)
    message(
      WARNING
      "The existing Bullet target should link the Bullet libraries that "
      "DART was built with (-float64: ${DART_BULLET_REQUIRE_FLOAT64}); a "
      "process must not load both Bullet precisions."
    )
  endif()
  if(NOT DEFINED DART_BULLET_REQUIRE_FLOAT64)
    _dart_bullet_record(
      "${_dart_bullet_found}"
      INCLUDES ${_dart_bullet_includes}
      LIBRARIES ${_dart_bullet_pc_files}
      LINK_DIRECTORIES ${_dart_bullet_link_directories}
    )
  endif()
  set(_DART_BULLET_GRAPH_UNKNOWN "${_dart_bullet_unknown}")
  set(_DART_BULLET_IMPORTED_FILES "${_dart_bullet_pc_files}")
  _dart_bullet_variant(_DART_BULLET_IMPORTED_VARIANT ${_dart_bullet_pc_files})
  list(REMOVE_DUPLICATES _dart_bullet_probe_configs)
  set(_DART_BULLET_PROBE_CONFIGS "${_dart_bullet_probe_configs}")
  unset(_dart_bullet_build_type)
  unset(_dart_bullet_files)
  unset(_dart_bullet_pc_files)
  unset(_dart_bullet_includes)
  unset(_dart_bullet_found)
  unset(_dart_bullet_expected)
endif()

# Quote a literal CMake argument without expanding caller variables or list items.
function(_dart_bullet_literal out value)
  string(REPLACE "\\" "\\\\" value "${value}")
  string(REPLACE "\"" "\\\"" value "${value}")
  string(REPLACE "$" "\\$" value "${value}")
  string(REPLACE "\r" "\\r" value "${value}")
  string(REPLACE "\n" "\\n" value "${value}")
  set(${out} "\"${value}\"" PARENT_SCOPE)
endfunction()

# Identity conditions must observe the original scope, including inactive targets.
function(_dart_bullet_snapshot_value out targets supported value)
  set(extra)
  foreach(expression TARGET_EXISTS TARGET_NAME_IF_EXISTS)
    string(
      REGEX MATCHALL
      "\\$<${expression}:[A-Za-z0-9_.:+-]+>"
      matches
      "${value}"
    )
    foreach(match IN LISTS matches)
      string(REGEX REPLACE "^\\$<${expression}:(.*)>$" "\\1" name "${match}")
      set(replacement "")
      if(expression STREQUAL "TARGET_EXISTS")
        set(replacement 0)
        if(TARGET "${name}")
          set(replacement 1)
        endif()
      elseif(TARGET "${name}")
        set(replacement "${name}")
        list(APPEND extra "${name}")
      endif()
      string(REPLACE "${match}" "${replacement}" value "${value}")
    endforeach()
  endforeach()
  set(valid TRUE)
  # shortcut: context-dependent target/property expressions need a fuller snapshot;
  # reject them until a concrete supported consumer requires that metadata.
  if(value MATCHES "\\$<(TARGET_|GENEX_EVAL:)|::@")
    set(valid FALSE)
  endif()
  set(${out} "${value}" PARENT_SCOPE)
  set(${targets} "${extra}" PARENT_SCOPE)
  set(${supported} "${valid}" PARENT_SCOPE)
endfunction()

# C-only consumers need a synchronous link check without enabling CXX in their
# scope. Ordinary child executables retain target identities that try_compile's
# further target export can omit. The child never runs a target program.
function(_dart_bullet_probe_child single_out double_out config source)
  set(${single_out} FALSE PARENT_SCOPE)
  set(${double_out} FALSE PARENT_SCOPE)
  if(
    DEFINED CMAKE_CXX_COMPILER
    AND CMAKE_CXX_COMPILER MATCHES "(^|-)NOTFOUND($|;)"
  )
    message(
      STATUS
      "C-only Bullet ABI validation has no usable selected CXX compiler"
    )
    return()
  endif()
  if(_dart_bullet_has_build_targets)
    message(
      STATUS
      "C-only Bullet ABI validation cannot snapshot project-built link dependencies"
    )
    return()
  endif()
  get_cmake_property(variables VARIABLES)
  foreach(variable IN LISTS variables)
    if(
      variable MATCHES "^CMAKE_PROJECT_.*INCLUDE"
      AND NOT "${${variable}}" STREQUAL ""
    )
      message(
        STATUS
        "C-only Bullet ABI validation cannot replay project include hooks"
      )
      return()
    endif()
  endforeach()

  set(queue ${_dart_bullet_targets})
  set(seen)
  set(declarations "")
  set(properties "")
  set(aliases "")
  string(TOUPPER "${config}" upper_config)
  while(queue)
    list(POP_FRONT queue target)
    if(target IN_LIST seen)
      continue()
    endif()
    list(APPEND seen "${target}")
    _dart_bullet_literal(quoted_target "${target}")
    get_target_property(alias "${target}" ALIASED_TARGET)
    if(alias)
      _dart_bullet_literal(quoted_alias "${alias}")
      string(
        APPEND aliases
        "add_library(${quoted_target} ALIAS ${quoted_alias})\n"
      )
      list(APPEND queue "${alias}")
      continue()
    endif()
    get_target_property(type "${target}" TYPE)
    get_target_property(imported "${target}" IMPORTED)
    if(
      NOT type MATCHES "^(INTERFACE|STATIC|SHARED|UNKNOWN)_LIBRARY$"
      OR (NOT imported AND NOT type STREQUAL "INTERFACE_LIBRARY")
    )
      message(
        STATUS
        "C-only Bullet ABI validation cannot snapshot target '${target}' of type '${type}'"
      )
      return()
    endif()
    string(REPLACE "_LIBRARY" "" type "${type}")
    set(kind "${type}")
    if(imported)
      string(APPEND kind " IMPORTED GLOBAL")
    endif()
    string(APPEND declarations "add_library(${quoted_target} ${kind})\n")
    foreach(
      property
      INTERFACE_SOURCES
      INTERFACE_PRECOMPILE_HEADERS
      INTERFACE_LINK_LIBRARIES_DIRECT
      INTERFACE_LINK_LIBRARIES_DIRECT_EXCLUDE
      COMPATIBLE_INTERFACE_BOOL
      COMPATIBLE_INTERFACE_STRING
      COMPATIBLE_INTERFACE_NUMBER_MIN
      COMPATIBLE_INTERFACE_NUMBER_MAX
      FRAMEWORK
      MACOSX_BUNDLE
    )
      get_target_property(value "${target}" "${property}")
      if(value)
        message(
          STATUS
          "C-only Bullet ABI validation cannot snapshot '${target}' property '${property}'"
        )
        return()
      endif()
    endforeach()
    set(
      names
      INTERFACE_LINK_LIBRARIES
      INTERFACE_LINK_OPTIONS
      INTERFACE_LINK_DIRECTORIES
      INTERFACE_LINK_DEPENDS
      SYSTEM
      INTERFACE_INCLUDE_DIRECTORIES
      INTERFACE_SYSTEM_INCLUDE_DIRECTORIES
      INTERFACE_COMPILE_DEFINITIONS
      INTERFACE_COMPILE_OPTIONS
      INTERFACE_COMPILE_FEATURES
      INTERFACE_POSITION_INDEPENDENT_CODE
    )
    if(imported)
      get_target_property(configs "${target}" IMPORTED_CONFIGURATIONS)
      if(NOT configs)
        set(configs)
      endif()
      get_target_property(
        mapped
        "${target}"
        MAP_IMPORTED_CONFIG_${upper_config}
      )
      if(NOT mapped)
        set(mapped)
      endif()
      list(APPEND configs "${upper_config}" ${mapped})
      list(REMOVE_DUPLICATES configs)
      list(
        APPEND names
        IMPORTED_CONFIGURATIONS
        MAP_IMPORTED_CONFIG_${upper_config}
      )
      foreach(
        property
        IMPORTED_LOCATION
        IMPORTED_IMPLIB
        IMPORTED_SONAME
        IMPORTED_NO_SONAME
        IMPORTED_LINK_INTERFACE_LANGUAGES
        IMPORTED_LINK_INTERFACE_LIBRARIES
        IMPORTED_LINK_DEPENDENT_LIBRARIES
        IMPORTED_LINK_INTERFACE_MULTIPLICITY
      )
        list(APPEND names "${property}")
        foreach(import_config IN LISTS configs)
          if(NOT import_config STREQUAL "")
            string(TOUPPER "${import_config}" import_config)
            list(APPEND names "${property}_${import_config}")
          endif()
        endforeach()
      endforeach()
    endif()
    foreach(property IN LISTS names)
      get_property(present TARGET "${target}" PROPERTY "${property}" SET)
      if(NOT present)
        continue()
      endif()
      get_target_property(value "${target}" "${property}")
      if(property MATCHES "^IMPORTED_LINK_DEPENDENT_LIBRARIES" AND value)
        message(
          STATUS
          "C-only Bullet ABI validation cannot snapshot hidden dependent libraries"
        )
        return()
      endif()
      _dart_bullet_snapshot_value(value extra supported "${value}")
      if(NOT supported)
        message(
          STATUS
          "C-only Bullet ABI validation cannot snapshot target context in '${target}' property '${property}'"
        )
        return()
      endif()
      foreach(extra_target IN LISTS extra)
        _dart_bullet_collect_graph("${config}" FALSE "${extra_target}")
        if(_dart_bullet_unknown OR _dart_bullet_has_build_targets)
          message(
            STATUS
            "C-only Bullet ABI validation cannot snapshot identity target '${extra_target}'"
          )
          return()
        endif()
        list(APPEND queue ${_dart_bullet_targets})
      endforeach()
      _dart_bullet_literal(quoted_property "${property}")
      _dart_bullet_literal(quoted_value "${value}")
      string(
        APPEND properties
        "set_property(TARGET ${quoted_target} PROPERTY ${quoted_property} ${quoted_value})\n"
      )
    endforeach()
  endwhile()

  set(
    inputs
    CMAKE_TOOLCHAIN_FILE
    CMAKE_MODULE_PATH
    CMAKE_PREFIX_PATH
    CMAKE_PROGRAM_PATH
    CMAKE_GENERATOR_INSTANCE
    CMAKE_CXX_COMPILER
    CMAKE_CXX_COMPILER_ARG1
    CMAKE_CXX_COMPILER_TARGET
    CMAKE_CXX_COMPILER_EXTERNAL_TOOLCHAIN
    CMAKE_CXX_COMPILER_LAUNCHER
    CMAKE_CXX_LINKER_LAUNCHER
    CMAKE_LINKER
    CMAKE_AR
    CMAKE_RANLIB
    CMAKE_CXX_COMPILER_AR
    CMAKE_CXX_COMPILER_RANLIB
    CMAKE_SYSROOT
    CMAKE_SYSROOT_COMPILE
    CMAKE_SYSROOT_LINK
    CMAKE_FIND_ROOT_PATH
    CMAKE_FIND_ROOT_PATH_MODE_PROGRAM
    CMAKE_FIND_ROOT_PATH_MODE_LIBRARY
    CMAKE_FIND_ROOT_PATH_MODE_INCLUDE
    CMAKE_FIND_ROOT_PATH_MODE_PACKAGE
    CMAKE_OSX_ARCHITECTURES
    CMAKE_OSX_SYSROOT
    CMAKE_OSX_DEPLOYMENT_TARGET
    CMAKE_APPLE_SILICON_PROCESSOR
    CMAKE_ANDROID_NDK
    CMAKE_ANDROID_STANDALONE_TOOLCHAIN
    CMAKE_ANDROID_ARCH_ABI
    CMAKE_ANDROID_ARM_MODE
    CMAKE_ANDROID_ARM_NEON
    CMAKE_ANDROID_STL_TYPE
    CMAKE_ANDROID_NDK_TOOLCHAIN_VERSION
    CMAKE_CXX_FLAGS
    CMAKE_CXX_FLAGS_INIT
    CMAKE_EXE_LINKER_FLAGS
    CMAKE_EXE_LINKER_FLAGS_INIT
    CMAKE_CXX_STANDARD
    CMAKE_CXX_STANDARD_REQUIRED
    CMAKE_CXX_EXTENSIONS
    CMAKE_CXX_SCAN_FOR_MODULES
    CMAKE_POSITION_INDEPENDENT_CODE
    CMAKE_MSVC_RUNTIME_LIBRARY
    CMAKE_MSVC_DEBUG_INFORMATION_FORMAT
    CMAKE_ENABLE_EXPORTS
    CMAKE_LINK_SEARCH_START_STATIC
    CMAKE_LINK_SEARCH_END_STATIC
    CMAKE_TRY_COMPILE_PLATFORM_VARIABLES
    ${CMAKE_TRY_COMPILE_PLATFORM_VARIABLES}
  )
  if(NOT CMAKE_GENERATOR MATCHES "^(Xcode$|Green Hills MULTI$|Visual Studio)")
    list(APPEND inputs CMAKE_MAKE_PROGRAM)
  endif()
  if(CMAKE_CROSSCOMPILING)
    list(
      APPEND inputs
      CMAKE_SYSTEM_NAME
      CMAKE_SYSTEM_VERSION
      CMAKE_SYSTEM_PROCESSOR
    )
  endif()
  foreach(prefix CMAKE_CXX_FLAGS CMAKE_EXE_LINKER_FLAGS)
    list(APPEND inputs ${prefix}_${upper_config} ${prefix}_${upper_config}_INIT)
  endforeach()
  list(REMOVE_DUPLICATES inputs)
  set(project "cmake_minimum_required(VERSION 3.28)\n")
  foreach(policy CMP0091 CMP0141 CMP0156 CMP0179)
    if(POLICY ${policy})
      cmake_policy(GET ${policy} value)
      # An unset parent policy uses OLD behavior, even when the child's minimum
      # CMake version would otherwise select NEW behavior.
      if(NOT value)
        set(value OLD)
      endif()
      string(APPEND project "cmake_policy(SET ${policy} ${value})\n")
    endif()
  endforeach()
  foreach(variable IN LISTS inputs)
    if(NOT variable MATCHES "^[A-Za-z_][A-Za-z0-9_]*$")
      message(
        STATUS
        "C-only Bullet ABI validation cannot snapshot platform variable '${variable}'"
      )
      return()
    endif()
    if(DEFINED ${variable})
      set(value "${${variable}}")
      if(
        variable STREQUAL "CMAKE_TOOLCHAIN_FILE"
        AND value
        AND NOT IS_ABSOLUTE "${value}"
      )
        if(EXISTS "${CMAKE_BINARY_DIR}/${value}")
          get_filename_component(
            value
            "${value}"
            ABSOLUTE
            BASE_DIR "${CMAKE_BINARY_DIR}"
          )
        else()
          get_filename_component(
            value
            "${value}"
            ABSOLUTE
            BASE_DIR "${CMAKE_SOURCE_DIR}"
          )
        endif()
      endif()
      _dart_bullet_literal(value "${value}")
      string(
        APPEND project
        "set(${variable} ${value} CACHE STRING \"Bullet ABI probe input\" FORCE)\n"
      )
    endif()
  endforeach()
  # Use ordinary CMake discovery, with the selected C compiler's directory as a
  # program-search hint; do not invent a CXX compiler from its executable name.
  if(NOT DEFINED CMAKE_CXX_COMPILER)
    get_filename_component(compiler_directory "${CMAKE_C_COMPILER}" DIRECTORY)
    _dart_bullet_literal(value "${compiler_directory};${CMAKE_PROGRAM_PATH}")
    string(
      APPEND project
      "set(CMAKE_PROGRAM_PATH ${value} CACHE STRING \"Bullet compiler search\" FORCE)\n"
    )
  endif()
  _dart_bullet_literal(value "${config}")
  get_property(multi_config GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
  if(multi_config)
    string(
      APPEND project
      "set(CMAKE_CONFIGURATION_TYPES ${value} CACHE STRING \"Bullet probe configuration\" FORCE)\n"
    )
  else()
    string(
      APPEND project
      "set(CMAKE_BUILD_TYPE ${value} CACHE STRING \"Bullet probe configuration\" FORCE)\n"
    )
  endif()
  string(
    APPEND project
    "project(DARTBulletABI LANGUAGES CXX)\n${declarations}${properties}${aliases}"
  )
  _dart_bullet_literal(includes "${CMAKE_REQUIRED_INCLUDES}")
  foreach(precision single double)
    string(
      APPEND project
      "add_executable(_dart_bullet_${precision} EXCLUDE_FROM_ALL ${precision}.cpp)\ntarget_include_directories(_dart_bullet_${precision} PRIVATE ${includes})\ntarget_compile_options(_dart_bullet_${precision} PRIVATE -w)\ntarget_link_libraries(_dart_bullet_${precision} PRIVATE Bullet)\n"
    )
  endforeach()
  string(RANDOM LENGTH 16 ALPHABET 0123456789abcdef nonce)
  set(directory "${CMAKE_BINARY_DIR}/CMakeFiles/DARTBulletABI-${nonce}")
  while(EXISTS "${directory}")
    string(RANDOM LENGTH 16 ALPHABET 0123456789abcdef nonce)
    set(directory "${CMAKE_BINARY_DIR}/CMakeFiles/DARTBulletABI-${nonce}")
  endwhile()
  file(MAKE_DIRECTORY "${directory}")
  file(WRITE "${directory}/CMakeLists.txt" "${project}")
  foreach(precision single double)
    set(definition "#undef BT_USE_DOUBLE_PRECISION\n")
    if(precision STREQUAL "double")
      string(APPEND definition "#define BT_USE_DOUBLE_PRECISION\n")
    endif()
    file(WRITE "${directory}/${precision}.cpp" "${definition}${source}")
  endforeach()
  execute_process(
    COMMAND
      "${CMAKE_COMMAND}" -S "${directory}" -B "${directory}/build" -G
      "${CMAKE_GENERATOR}" -A "${CMAKE_GENERATOR_PLATFORM}" -T
      "${CMAKE_GENERATOR_TOOLSET}"
    RESULT_VARIABLE result
    OUTPUT_VARIABLE output
    ERROR_VARIABLE error
  )
  file(WRITE "${directory}/configure.log" "${output}\n${error}")
  if(NOT result STREQUAL "0")
    message(
      STATUS
      "Bullet isolated CXX ABI probe could not configure; see ${directory}/configure.log"
    )
    return()
  endif()
  foreach(precision single double)
    execute_process(
      COMMAND
        "${CMAKE_COMMAND}" --build "${directory}/build" --target
        "_dart_bullet_${precision}" --config "${config}"
      RESULT_VARIABLE result
      OUTPUT_VARIABLE output
      ERROR_VARIABLE error
    )
    file(WRITE "${directory}/${precision}.log" "${output}\n${error}")
    if(result STREQUAL "0")
      set(${precision}_out_value TRUE)
    else()
      set(${precision}_out_value FALSE)
    endif()
  endforeach()
  set(${single_out} "${single_out_value}" PARENT_SCOPE)
  set(${double_out} "${double_out_value}" PARENT_SCOPE)
endfunction()

# Verify scalar ABI independently of library names. Both Collision and LinearMath
# expose scalar-dependent symbols; requiring both detects mixed unsuffixed pairs.
function(_dart_bullet_probe_precision out)
  set(${out} "" PARENT_SCOPE)
  if(_DART_BULLET_GRAPH_UNKNOWN)
    return()
  endif()
  include(CheckCXXSourceCompiles)
  include(CMakePushCheckState)
  cmake_push_check_state(RESET)
  # Header warnings must not turn a valid scalar ABI into a failed link probe.
  set(CMAKE_REQUIRED_FLAGS "-w")
  set(CMAKE_TRY_COMPILE_TARGET_TYPE EXECUTABLE)
  set(probe_target Bullet)
  get_target_property(probe_alias Bullet ALIASED_TARGET)
  if(probe_alias)
    set(probe_target "${probe_alias}")
  endif()
  get_target_property(probe_imported "${probe_target}" IMPORTED)
  # Use exactly the configurations whose dependency graphs were classified.
  set(configs "${_DART_BULLET_PROBE_CONFIGS}")
  set(observed)
  foreach(config IN LISTS configs)
    string(SUBSTRING "${config}" 1 -1 CMAKE_TRY_COMPILE_CONFIGURATION)
    _dart_bullet_collect_graph("${CMAKE_TRY_COMPILE_CONFIGURATION}" FALSE)
    if(_dart_bullet_unknown)
      cmake_pop_check_state()
      return()
    endif()
    list(REMOVE_DUPLICATES _dart_bullet_built_precisions)
    list(APPEND observed ${_dart_bullet_built_precisions})
    _dart_bullet_variant(imported_variant ${_dart_bullet_pc_files})
    if(_dart_bullet_built_precisions AND NOT imported_variant)
      continue()
    endif()
    if(NOT CMAKE_CXX_COMPILER_LOADED AND NOT CMAKE_C_COMPILER_LOADED)
      message(
        STATUS
        "Bullet scalar ABI validation requires an enabled CXX language"
      )
      cmake_pop_check_state()
      return()
    endif()
    set(CMAKE_REQUIRED_INCLUDES "${_dart_bullet_includes}")
    if(_dart_bullet_has_build_targets)
      set(CMAKE_REQUIRED_LIBRARIES "${_dart_bullet_pc_files}")
    elseif(probe_imported)
      set(CMAKE_REQUIRED_LIBRARIES "${probe_target}")
    else()
      get_target_property(
        CMAKE_REQUIRED_LIBRARIES
        "${probe_target}"
        INTERFACE_LINK_LIBRARIES
      )
    endif()
    set(
      collision_probe
      "btVector3 v(0., 0., 1.); btStaticPlaneShape plane(v, 0.);"
    )
    set(
      math_probe
      "btConvexHullComputer hull; hull.compute(static_cast<const float*>(nullptr), 0, 0, 0., 0.);"
    )
    if(_dart_bullet_built_precisions)
      # A try_compile cannot link project-built targets. Probe only the imported
      # components, then compare with the project components' compile definitions.
      set(CMAKE_REQUIRED_LIBRARIES "${_dart_bullet_pc_files}")
      set(imported_components)
      foreach(library IN LISTS _dart_bullet_pc_files)
        get_filename_component(name "${library}" NAME)
        if(
          name
            MATCHES
            "^(lib)?(BulletCollision|LinearMath|BulletMath)(-float64)?(_Debug|_d|-dbg)?(\\.|$)"
        )
          list(APPEND imported_components "${CMAKE_MATCH_2}")
        endif()
      endforeach()
      if(NOT "BulletCollision" IN_LIST imported_components)
        set(collision_probe "")
      endif()
      if(
        NOT "LinearMath" IN_LIST imported_components
        AND NOT "BulletMath" IN_LIST imported_components
      )
        set(math_probe "")
      endif()
      if(NOT collision_probe AND NOT math_probe)
        cmake_pop_check_state()
        return()
      endif()
    endif()
    if(NOT CMAKE_CXX_COMPILER_LOADED)
      _dart_bullet_probe_child(_dart_bullet_probe_single _dart_bullet_probe_double
        "${CMAKE_TRY_COMPILE_CONFIGURATION}"
        "#include <btBulletCollisionCommon.h>
        #include <LinearMath/btConvexHullComputer.h>
        int main() { ${collision_probe} ${math_probe} return 0; }
        "
      )
    else()
      foreach(precision single double)
        set(definition "#undef BT_USE_DOUBLE_PRECISION")
        if(precision STREQUAL "double")
          string(APPEND definition "\n#define BT_USE_DOUBLE_PRECISION")
        endif()
        unset(_dart_bullet_probe_${precision} CACHE)
        check_cxx_source_compiles(
          "${definition}
          #include <btBulletCollisionCommon.h>
          #include <LinearMath/btConvexHullComputer.h>
          int main() { ${collision_probe} ${math_probe} return 0; }
          "
          _dart_bullet_probe_${precision}
        )
      endforeach()
    endif()
    if(_dart_bullet_probe_single AND NOT _dart_bullet_probe_double)
      list(APPEND observed single)
    elseif(_dart_bullet_probe_double AND NOT _dart_bullet_probe_single)
      list(APPEND observed double)
    else()
      list(APPEND observed unknown)
    endif()
  endforeach()
  list(REMOVE_DUPLICATES observed)
  if(observed STREQUAL "single" OR observed STREQUAL "double")
    set(${out} "${observed}" PARENT_SCOPE)
  endif()
  cmake_pop_check_state()
endfunction()

if((BULLET_FOUND OR Bullet_FOUND) AND DART_BULLET_REQUIRE_PRECISION)
  _dart_bullet_probe_precision(_dart_bullet_precision)
  if(NOT _dart_bullet_precision STREQUAL DART_BULLET_REQUIRE_PRECISION)
    message(
      WARNING
      "DART requires ${DART_BULLET_REQUIRE_PRECISION}-precision Bullet; "
      "the selected libraries have '${_dart_bullet_precision}' scalar ABI (empty means unverified)."
    )
    set(BULLET_FOUND FALSE)
    set(Bullet_FOUND FALSE)
  endif()
endif()
