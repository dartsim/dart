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
# flags for the INCLUDES and the Bullet files among LIBRARIES, without
# FindBullet's Debug entries, which dart.pc names when no bullet-float64
# module describes them.
function(_dart_bullet_record variant)
  cmake_parse_arguments(PARSE_ARGV 1 arg "" "" "INCLUDES;LIBRARIES")
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
    get_filename_component(name "${library}" NAME)
    if(skip)
      set(skip FALSE)
    elseif(library STREQUAL "debug")
      set(skip TRUE)
    elseif(name MATCHES "${_dart_bullet_name_regex}")
      list(APPEND libraries "${library}")
    endif()
  endforeach()
  set(includes ${arg_INCLUDES})
  list(REMOVE_DUPLICATES includes)
  list(REMOVE_DUPLICATES libraries)
  set(cflags "")
  foreach(include_dir IN LISTS includes)
    _dart_bullet_pc_path(path "${include_dir}" COMPILE)
    _dart_bullet_pc_escape(path "${path}")
    string(APPEND cflags " -I${path}")
  endforeach()
  set(libs "")
  set(error "")
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
      _dart_bullet_pc_escape(path "${path}")
      string(APPEND libs " ${path}")
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
  foreach(suffix IN LISTS configs)
    if(suffix STREQUAL "_")
      set(suffix "")
    endif()
    string(TOUPPER "${suffix}" suffix)
    _dart_bullet_location(location "${target}" "${suffix}")
    if(location)
      break()
    endif()
  endforeach()
  set(${out} "${location}" PARENT_SCOPE)
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

# CONFIG expressions inherit mappings from the target that owns their property.
macro(_dart_bullet_enqueue queue position entries owner)
  set(_dart_bullet_entries ${${entries}})
  foreach(_dart_bullet_entry IN LISTS _dart_bullet_entries)
    list(${position} ${queue}_owners "${owner}")
  endforeach()
  list(${position} ${queue} ${_dart_bullet_entries})
endmacro()

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
    foreach(output pc_files includes)
      set(_dart_bullet_${output} "${_dart_bullet_${output}}" PARENT_SCOPE)
    endforeach()
    return()
  endif()
  string(TOUPPER "${config}" _dart_bullet_build_type)
  set(_dart_bullet_queue Bullet)
  set(_dart_bullet_queue_owners Bullet)
  set(_dart_bullet_seen)
  set(_dart_bullet_files)
  set(_dart_bullet_pc_files)
  set(_dart_bullet_includes)
  set(_dart_bullet_unknown FALSE)
  set(_dart_bullet_built_precisions)
  set(_dart_bullet_has_build_targets FALSE)
  while(_dart_bullet_queue)
    list(GET _dart_bullet_queue 0 _dart_bullet_item)
    list(REMOVE_AT _dart_bullet_queue 0)
    list(GET _dart_bullet_queue_owners 0 _dart_bullet_owner)
    list(REMOVE_AT _dart_bullet_queue_owners 0)
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
      list(REMOVE_AT _dart_bullet_queue_owners 0)
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
      _dart_bullet_enqueue(_dart_bullet_queue PREPEND _dart_bullet_item "${_dart_bullet_owner}")
      continue()
    endif()
    if(NOT TARGET "${_dart_bullet_item}")
      list(APPEND _dart_bullet_files "${_dart_bullet_item}")
      list(APPEND _dart_bullet_pc_files "${_dart_bullet_item}")
      continue()
    endif()
    if(_dart_bullet_item IN_LIST _dart_bullet_seen)
      continue()
    endif()
    list(APPEND _dart_bullet_seen "${_dart_bullet_item}")
    get_target_property(
      _dart_bullet_value
      "${_dart_bullet_item}"
      INTERFACE_LINK_LIBRARIES
    )
    if(_dart_bullet_value)
      _dart_bullet_enqueue(_dart_bullet_queue PREPEND _dart_bullet_value "${_dart_bullet_item}")
    endif()
    get_target_property(
      _dart_bullet_value
      "${_dart_bullet_item}"
      INTERFACE_INCLUDE_DIRECTORIES
    )
    foreach(_dart_bullet_dir IN LISTS _dart_bullet_value)
      while(
        _dart_bullet_dir
          MATCHES
          "^\\$<(BUILD_INTERFACE|BUILD_LOCAL_INTERFACE):(.+)>$"
      )
        set(_dart_bullet_dir "${CMAKE_MATCH_2}")
      endwhile()
      if(_dart_bullet_dir MATCHES "^\\$<INSTALL_INTERFACE:")
        continue()
      elseif(_dart_bullet_dir MATCHES "\\$<")
        set(_dart_bullet_unknown TRUE)
      endif()
      if(IS_ABSOLUTE "${_dart_bullet_dir}")
        list(APPEND _dart_bullet_includes "${_dart_bullet_dir}")
      endif()
    endforeach()
    get_target_property(_dart_bullet_imported "${_dart_bullet_item}" IMPORTED)
    if(NOT _dart_bullet_imported)
      get_target_property(_dart_bullet_type "${_dart_bullet_item}" TYPE)
      if(NOT _dart_bullet_type STREQUAL "INTERFACE_LIBRARY")
        set(_dart_bullet_has_build_targets TRUE)
      endif()
      # try_compile cannot link Bullet libraries that this project builds;
      # Bullet's USE_DOUBLE_PRECISION option defines BT_USE_DOUBLE_PRECISION
      # for their directories instead.
      get_target_property(_dart_bullet_value "${_dart_bullet_item}" TYPE)
      get_target_property(_dart_bullet_dir "${_dart_bullet_item}" NAME)
      if(
        NOT _dart_bullet_value STREQUAL "INTERFACE_LIBRARY"
        AND _dart_bullet_dir MATCHES "${_dart_bullet_name_regex}"
      )
        get_target_property(_dart_bullet_dir "${_dart_bullet_item}" SOURCE_DIR)
        get_directory_property(
          _dart_bullet_value
          DIRECTORY "${_dart_bullet_dir}"
          COMPILE_DEFINITIONS
        )
        get_target_property(
          _dart_bullet_dir
          "${_dart_bullet_item}"
          COMPILE_DEFINITIONS
        )
        list(APPEND _dart_bullet_value "${_dart_bullet_dir}")
        if(
          _dart_bullet_value
            MATCHES
            "(^|;)BT_USE_DOUBLE_PRECISION(=[^;]*)?(;|$)"
        )
          list(APPEND _dart_bullet_built_precisions double)
        elseif(_dart_bullet_value MATCHES "\\$<")
          set(_dart_bullet_unknown TRUE)
        else()
          list(APPEND _dart_bullet_built_precisions single)
          get_target_property(
            _dart_bullet_dir
            "${_dart_bullet_item}"
            INTERFACE_COMPILE_DEFINITIONS
          )
          if(_dart_bullet_dir MATCHES "BT_USE_DOUBLE_PRECISION|\\$<")
            set(_dart_bullet_unknown TRUE)
          endif()
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
  foreach(
    output
    files
    pc_files
    includes
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
    if(NOT CMAKE_CXX_COMPILER_LOADED)
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
