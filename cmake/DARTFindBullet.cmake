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
# with DART_BULLET_REQUIRE_VERSION. DART_BULLET_BUILT_PRECISION is double or
# single when an existing Bullet target links Bullet libraries that this
# project builds, which the precision probe cannot link.
include("${CMAKE_CURRENT_LIST_DIR}/DARTFindPackageVersion.cmake")

set(DART_BULLET_BUILT_PRECISION "")

set(
  _dart_bullet_name_regex
  "^(lib)?(Bullet(Collision|Dynamics|Math|SoftBody)|LinearMath)"
)

# Sets <out> to float64, single or mixed for the Bullet library files among
# the arguments, judged by file name, or to an empty string if there are none.
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
  list(TRANSFORM includes PREPEND " -I")
  list(TRANSFORM libraries PREPEND " ")
  string(JOIN "" cflags ${includes})
  string(JOIN "" libs ${libraries})
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
  find_package(Bullet COMPONENTS BulletMath BulletCollision MODULE QUIET)
endif()

if(BULLET_FOUND OR Bullet_FOUND)
  unset(BULLET_VERSION)
  dart_read_header_version(_bullet_header_version
    LinearMath/btScalar.h BT_BULLET_VERSION ${BULLET_INCLUDE_DIRS}
  )
  if(_bullet_header_version MATCHES "^[0-9]+$")
    math(EXPR _bullet_major "${_bullet_header_version} / 100")
    math(EXPR _bullet_minor "${_bullet_header_version} % 100")
    set(BULLET_VERSION "${_bullet_major}.${_bullet_minor}")
  endif()
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
    # Keep FindBullet's libraries, link order and Debug variants; the collision
    # and math libraries are required. Keep FindBullet's selection when it has
    # Debug variants that the -float64 libraries lack. Search beside each
    # library's Release and Debug files, which multi-config layouts separate.
    set(_dart_bullet_needs_debug FALSE)
    if("${BULLET_LIBRARIES}" MATCHES "(^|;)debug;")
      set(_dart_bullet_needs_debug TRUE)
    endif()
    set(_dart_bullet_float64_libraries)
    set(_dart_bullet_complete TRUE)
    foreach(
      _dart_bullet_name
      DYNAMICS:BulletDynamics
      COLLISION:BulletCollision
      MATH:LinearMath
      SOFTBODY:BulletSoftBody
    )
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
        if(_dart_bullet_name MATCHES "^(BulletCollision|LinearMath)$")
          set(_dart_bullet_complete FALSE)
        endif()
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
elseif(TARGET Bullet)
  # A parent project or an earlier include created the Bullet target, which
  # DART links as is. Classify the Bullet library files of the target and of
  # every target it links, unwrapping generator expressions and including
  # every configuration's locations. DART rejects a target that mixes
  # variants, an installed DART also rejects one of the other variant, and it
  # warns when the target names no Bullet library files. dart.pc may name the
  # files that the build type links, which defaults to Release: entries for
  # other configurations wait in a second queue that only classifies them.
  string(TOUPPER "${CMAKE_BUILD_TYPE}" _dart_bullet_build_type)
  if(NOT _dart_bullet_build_type)
    set(_dart_bullet_build_type RELEASE)
  endif()
  set(_dart_bullet_queue Bullet)
  set(_dart_bullet_other_queue)
  set(_dart_bullet_pc TRUE)
  set(_dart_bullet_seen)
  set(_dart_bullet_files)
  set(_dart_bullet_pc_files)
  set(_dart_bullet_includes)
  while(_dart_bullet_queue OR _dart_bullet_other_queue)
    if(NOT _dart_bullet_queue)
      set(_dart_bullet_queue ${_dart_bullet_other_queue})
      set(_dart_bullet_other_queue)
      set(_dart_bullet_pc FALSE)
    endif()
    list(GET _dart_bullet_queue 0 _dart_bullet_item)
    list(REMOVE_AT _dart_bullet_queue 0)
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
    # Unwrap $<LINK_ONLY:...>, $<$<CONFIG:Debug>:...> and similar entries,
    # and queue the elements of a list payload. Only shorter payloads are
    # queued, so the walk ends.
    set(_dart_bullet_part "${_dart_bullet_item}")
    set(_dart_bullet_other FALSE)
    while(
      _dart_bullet_item
        MATCHES
        "^\\$<(\\$<([^<>]|\\$<[^<>]*>)*>|[A-Za-z0-9_]+):(.*)>$"
    )
      set(_dart_bullet_item "${CMAKE_MATCH_3}")
      string(TOUPPER "${CMAKE_MATCH_1}" _dart_bullet_value)
      if(_dart_bullet_value MATCHES "^\\$<(NOT:\\$<)?CONFIG:([^<>]*)>")
        string(REPLACE "," ";" _dart_bullet_configs "${CMAKE_MATCH_2}")
        if(_dart_bullet_build_type IN_LIST _dart_bullet_configs)
          if(_dart_bullet_value MATCHES "^\\$<NOT:")
            set(_dart_bullet_other TRUE)
          endif()
        elseif(NOT _dart_bullet_value MATCHES "^\\$<NOT:")
          set(_dart_bullet_other TRUE)
        endif()
      endif()
    endwhile()
    if(_dart_bullet_other AND _dart_bullet_pc)
      list(APPEND _dart_bullet_other_queue ${_dart_bullet_part})
      continue()
    endif()
    if(
      _dart_bullet_item MATCHES ";"
      AND NOT _dart_bullet_item STREQUAL _dart_bullet_part
    )
      list(APPEND _dart_bullet_queue ${_dart_bullet_item})
      continue()
    endif()
    if(NOT TARGET "${_dart_bullet_item}")
      list(APPEND _dart_bullet_files "${_dart_bullet_item}")
      if(_dart_bullet_pc)
        list(APPEND _dart_bullet_pc_files "${_dart_bullet_item}")
      endif()
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
      list(APPEND _dart_bullet_queue ${_dart_bullet_value})
    endif()
    get_target_property(
      _dart_bullet_value
      "${_dart_bullet_item}"
      INTERFACE_INCLUDE_DIRECTORIES
    )
    foreach(_dart_bullet_dir IN LISTS _dart_bullet_value)
      if(_dart_bullet_pc AND IS_ABSOLUTE "${_dart_bullet_dir}")
        list(APPEND _dart_bullet_includes "${_dart_bullet_dir}")
      endif()
    endforeach()
    get_target_property(_dart_bullet_imported "${_dart_bullet_item}" IMPORTED)
    if(NOT _dart_bullet_imported)
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
        foreach(
          _dart_bullet_property
          COMPILE_DEFINITIONS
          INTERFACE_COMPILE_DEFINITIONS
        )
          get_target_property(
            _dart_bullet_dir
            "${_dart_bullet_item}"
            ${_dart_bullet_property}
          )
          list(APPEND _dart_bullet_value "${_dart_bullet_dir}")
        endforeach()
        if(
          _dart_bullet_value
            MATCHES
            "(^|;)BT_USE_DOUBLE_PRECISION(=[^;]*)?(;|$)"
        )
          set(DART_BULLET_BUILT_PRECISION double)
        elseif(NOT DART_BULLET_BUILT_PRECISION)
          set(DART_BULLET_BUILT_PRECISION single)
        endif()
      endif()
      continue()
    endif()
    get_target_property(
      _dart_bullet_configs
      "${_dart_bullet_item}"
      IMPORTED_CONFIGURATIONS
    )
    if(NOT _dart_bullet_configs)
      set(_dart_bullet_configs)
    endif()
    # dart.pc names the location that CMake links for the build type: a
    # mapped or matching configuration, the default one, or a listed one.
    if(_dart_bullet_pc)
      get_target_property(
        _dart_bullet_value
        "${_dart_bullet_item}"
        MAP_IMPORTED_CONFIG_${_dart_bullet_build_type}
      )
      if(NOT _dart_bullet_value)
        set(_dart_bullet_value ${_dart_bullet_build_type})
      endif()
      set(_dart_bullet_location "")
      foreach(_dart_bullet_config IN LISTS _dart_bullet_value)
        string(TOUPPER "_${_dart_bullet_config}" _dart_bullet_config)
        if(NOT _dart_bullet_location)
          _dart_bullet_location(
            _dart_bullet_location
            "${_dart_bullet_item}"
            "${_dart_bullet_config}"
          )
        endif()
      endforeach()
      if(NOT _dart_bullet_location)
        _dart_bullet_location(_dart_bullet_location "${_dart_bullet_item}" "")
      endif()
      foreach(_dart_bullet_config IN LISTS _dart_bullet_configs)
        string(TOUPPER "_${_dart_bullet_config}" _dart_bullet_config)
        if(NOT _dart_bullet_location)
          _dart_bullet_location(
            _dart_bullet_location
            "${_dart_bullet_item}"
            "${_dart_bullet_config}"
          )
        endif()
      endforeach()
      if(_dart_bullet_location)
        list(APPEND _dart_bullet_pc_files "${_dart_bullet_location}")
      endif()
    endif()
    # CMake also uses IMPORTED_LOCATION_<CONFIG> without a listed <CONFIG>.
    list(
      APPEND _dart_bullet_configs
      ${CMAKE_BUILD_TYPE}
      ${CMAKE_CONFIGURATION_TYPES}
      Release
      Debug
      RelWithDebInfo
      MinSizeRel
    )
    set(_dart_bullet_properties IMPORTED_LOCATION IMPORTED_IMPLIB)
    foreach(_dart_bullet_config IN LISTS _dart_bullet_configs)
      string(TOUPPER "${_dart_bullet_config}" _dart_bullet_config)
      list(
        APPEND _dart_bullet_properties
        IMPORTED_LOCATION_${_dart_bullet_config}
        IMPORTED_IMPLIB_${_dart_bullet_config}
      )
    endforeach()
    foreach(_dart_bullet_property IN LISTS _dart_bullet_properties)
      get_target_property(
        _dart_bullet_value
        "${_dart_bullet_item}"
        ${_dart_bullet_property}
      )
      if(_dart_bullet_value)
        list(APPEND _dart_bullet_files "${_dart_bullet_value}")
      endif()
    endforeach()
  endwhile()
  _dart_bullet_variant(_dart_bullet_found ${_dart_bullet_files})
  set(_dart_bullet_expected "")
  if(DART_BULLET_REQUIRE_FLOAT64 STREQUAL "TRUE")
    set(_dart_bullet_expected float64)
  elseif(DART_BULLET_REQUIRE_FLOAT64 STREQUAL "FALSE")
    set(_dart_bullet_expected single)
  endif()
  if(_dart_bullet_found STREQUAL "mixed")
    message(
      WARNING
      "The existing Bullet target links -float64 and single-precision Bullet "
      "libraries; a process must not load both Bullet precisions."
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
  unset(_dart_bullet_build_type)
  unset(_dart_bullet_queue)
  unset(_dart_bullet_other_queue)
  unset(_dart_bullet_pc)
  unset(_dart_bullet_other)
  unset(_dart_bullet_seen)
  unset(_dart_bullet_files)
  unset(_dart_bullet_pc_files)
  unset(_dart_bullet_includes)
  unset(_dart_bullet_dir)
  unset(_dart_bullet_item)
  unset(_dart_bullet_part)
  unset(_dart_bullet_depth)
  unset(_dart_bullet_opens)
  unset(_dart_bullet_closes)
  unset(_dart_bullet_open)
  unset(_dart_bullet_close)
  unset(_dart_bullet_value)
  unset(_dart_bullet_imported)
  unset(_dart_bullet_configs)
  unset(_dart_bullet_config)
  unset(_dart_bullet_location)
  unset(_dart_bullet_properties)
  unset(_dart_bullet_property)
  unset(_dart_bullet_found)
  unset(_dart_bullet_expected)
endif()
unset(_dart_bullet_name_regex)
