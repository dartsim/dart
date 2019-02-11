# Copyright (c) 2016 Carnegie Mellon University
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors
# may be used to endorse or promote products derived from this software without
# specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

cmake_minimum_required(VERSION 3.5.1)

include(CMakeParseArguments)

# This must be called before any other component function
# as it initializes the requried global properties.
function(initialize_component_helpers package_name)
  define_property(GLOBAL PROPERTY "${package_name}_INCLUDE_DIRS"
    BRIEF_DOCS "Global include directories used by all components."
    FULL_DOCS "Global include directories used by all components."
    )
  define_property(GLOBAL PROPERTY "${package_name}_COMPONENTS"
    BRIEF_DOCS "List all known ${package_name} components."
    FULL_DOCS "List all known ${package_name} components."
    )
endfunction(initialize_component_helpers)

#==============================================================================
function(is_component output_variable package_name component)
  set(component_prefix "${package_name}_component_")
  set(target "${component_prefix}${component}")

  if(TARGET "${target}")
    get_property(output TARGET "${target}"
      PROPERTY "${package_component_prefix}COMPONENT")
    set("${output_variable}" ${output} PARENT_SCOPE)
  else()
    set("${output_variable}" FALSE PARENT_SCOPE)
  endif()
endfunction()

#==============================================================================
# Create a custom target for a component. The target is named
# <package_name>_component_<component>. Afterwards, the dependency targets of
# the component can be added to the target using add_component_targets().
#
# Warning: At least one dependency target must be added to the component target,
# otherwise this function returns error.
function(add_component package_name component)
  set(component_prefix "${package_name}_component_")
  set(target "${component_prefix}${component}")
  add_custom_target("${target}")

  install(EXPORT "${target}"
    FILE "${package_name}_${component}Targets.cmake"
    DESTINATION "${CONFIG_INSTALL_DIR}"
    )
  # TODO(JS): It would be nice if we could check if ${target} has at least one
  # dependency target.

  set_property(TARGET "${target}" PROPERTY "${component_prefix}COMPONENT" TRUE)
  set_property(TARGET "${target}" PROPERTY "${component_prefix}DEPENDENCIES")
  set_property(TARGET "${target}" PROPERTY "${component_prefix}INCLUDE_DIRS")
  set_property(TARGET "${target}" PROPERTY "${component_prefix}LIBRARIES")
  set_property(GLOBAL APPEND
    PROPERTY "${package_name}_COMPONENTS" "${component}")
endfunction()

#==============================================================================
function(add_component_include_directories package_name component)
  set(component_prefix "${package_name}_component_")
  set(target "${component_prefix}${component}")
  set_property(TARGET "${target}" APPEND
    PROPERTY "${component_prefix}INCLUDE_DIRS" ${ARGN})
endfunction()

#==============================================================================
function(add_component_dependencies package_name component)
  set(component_prefix "${package_name}_component_")
  set(dependency_components ${ARGN})

  is_component(is_valid ${package_name} "${component}")
  if(NOT ${is_valid})
    message(FATAL_ERROR
      "Target '${component}' is not a component of ${package_name}.")
  endif()

  set(target "${component_prefix}${component}")

  foreach(dependency_component ${dependency_components})
    is_component(is_valid ${package_name} "${dependency_component}")
    if(NOT ${is_valid})
      message(FATAL_ERROR
        "Target '${dependency_component}' is not a component of ${package_name}.")
    endif()

    set(dependency_target "${component_prefix}${dependency_component}")
    add_dependencies("${target}" "${dependency_target}")
  endforeach()

  set_property(TARGET "${target}" APPEND
    PROPERTY "${component_prefix}DEPENDENCIES" ${dependency_components})
endfunction()

#==============================================================================
function(add_component_dependency_packages package_name component)
  set(component_prefix "${package_name}_component_")
  set(dependency_package ${ARGN})

  is_component(is_valid ${package_name} "${component}")
  if(NOT ${is_valid})
    message(FATAL_ERROR
      "Target '${component}' is not a component of ${package_name}.")
  endif()

  set(target "${component_prefix}${component}")

  set_property(TARGET "${target}" APPEND
    PROPERTY "${component_prefix}dependency_package" ${dependency_package})
endfunction()

#==============================================================================
function(add_component_targets package_name component)
  set(component_prefix "${package_name}_component_")
  set(dependency_targets ${ARGN})

  is_component(is_valid ${package_name} "${component}")
  if(NOT ${is_valid})
    message(FATAL_ERROR
      "Target '${component}' is not a component of ${package_name}.")
  endif()

  set(target "${component_prefix}${component}")
  add_dependencies("${target}" ${ARGN})

  foreach(dependency_target ${dependency_targets})
    if(NOT TARGET "${dependency_target}")
      message(FATAL_ERROR "Target '${dependency_target}' does not exist.")
    endif()

    get_property(dependency_type TARGET "${dependency_target}" PROPERTY TYPE)
    if(NOT ("${dependency_type}" STREQUAL STATIC_LIBRARY
          OR "${dependency_type}" STREQUAL SHARED_LIBRARY))
      message(FATAL_ERROR
        "Target '${dependency_target}' has unsupported type"
        " '${dependency_type}'. Only 'STATIC_LIBRARY' and 'SHARED_LIBRARY'"
        " are supported.")
    endif()

    install(TARGETS "${dependency_target}"
      EXPORT "${target}"
      ARCHIVE DESTINATION "${LIBRARY_INSTALL_DIR}"
      LIBRARY DESTINATION "${LIBRARY_INSTALL_DIR}"
      )
  endforeach()

  set_property(TARGET "${target}" APPEND
    PROPERTY "${component_prefix}LIBRARIES" ${dependency_targets})
endfunction()

#==============================================================================
function(install_component_exports package_name)
  set(component_prefix "${package_name}_component_")
  get_property(components GLOBAL PROPERTY "${package_name}_COMPONENTS")

  set(output_prefix "${CMAKE_CURRENT_BINARY_DIR}/${CONFIG_INSTALL_DIR}")

  foreach(component ${components})
    set(target "${component_prefix}${component}")

    # TODO: Replace this manual generation with a configure_file.
    set(output_path
      "${output_prefix}/${package_name}_${component}Component.cmake")

    get_property(internal_dependencies TARGET "${target}"
      PROPERTY "${component_prefix}DEPENDENCIES")

    get_property(libraries TARGET "${target}"
      PROPERTY "${component_prefix}LIBRARIES")

    get_property(dependency_package TARGET "${target}"
      PROPERTY "${component_prefix}dependency_package")
    set(external_dependencies)
    foreach(dependent_package ${dependency_package})
      set(find_pkg_name "Find${dependent_package}.cmake")
      set(find_pkg_path "")
      set(dart_find_pkg_name "DARTFind${dependent_package}.cmake")
      set(dart_find_pkg_path "")
      foreach(module_path ${CMAKE_MODULE_PATH})
        set(find_pkg_path_candidate "${module_path}/${find_pkg_name}")
        set(dart_find_pkg_path_candidate "${module_path}/${dart_find_pkg_name}")
        if("${find_pkg_path}" STREQUAL "" AND EXISTS "${find_pkg_path_candidate}")
          set(find_pkg_path ${find_pkg_path_candidate})
        endif()
        if("${dart_find_pkg_path}" STREQUAL "" AND EXISTS "${dart_find_pkg_path_candidate}")
          set(dart_find_pkg_path ${dart_find_pkg_path_candidate})
        endif()
      endforeach()
      if(NOT "${find_pkg_path}" STREQUAL "")
        install(FILES "${find_pkg_path}" DESTINATION "${CONFIG_INSTALL_DIR}")
      endif()
      if("${dart_find_pkg_path}" STREQUAL "")
        message(FATAL_ERROR "Failed to find '${dart_find_pkg_path}'.")
      endif()
      list(APPEND external_dependencies ${dependent_package})
      install(FILES "${dart_find_pkg_path}" DESTINATION "${CONFIG_INSTALL_DIR}")
    endforeach()

    configure_file(
      "${CMAKE_SOURCE_DIR}/cmake/dart_Component.cmake.in"
      "${output_path}"
      @ONLY)

    install(FILES "${output_path}"
      DESTINATION "${CONFIG_INSTALL_DIR}")
  endforeach()
endfunction()
