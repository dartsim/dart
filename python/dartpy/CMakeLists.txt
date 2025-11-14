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

# Determine Python site-packages directory
# For wheel builds with scikit-build-core, SKBUILD_PLATLIB_DIR is used directly
# For traditional CMake installs, we need to calculate the relative path
# and prefix it with CMAKE_INSTALL_PREFIX
execute_process(COMMAND ${Python3_EXECUTABLE} -c
  "import sysconfig; print(sysconfig.get_path('platlib', scheme='posix_prefix', vars={'base': '', 'platbase': ''}).lstrip('/'))"
  OUTPUT_VARIABLE PYTHON_SITE_PACKAGES_REL
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
set(PYTHON_SITE_PACKAGES "${CMAKE_INSTALL_PREFIX}/${PYTHON_SITE_PACKAGES_REL}")
set(PYTHON_SITE_PACKAGES ${PYTHON_SITE_PACKAGES} PARENT_SCOPE)

file(GLOB_RECURSE dartpy_headers "*.h" "*.hpp")
file(GLOB_RECURSE dartpy_sources "*.cpp")

# Remove the USD bindings if the core library was built without USD support.
if(NOT TARGET dart-utils-usd)
  list(FILTER dartpy_sources EXCLUDE REGEX "utils/UsdParser.cpp$")
endif()

# Exclude GUI OSG bindings if GUI is not built
if(NOT DART_BUILD_GUI_OSG)
  list(FILTER dartpy_headers EXCLUDE REGEX ".*/gui/osg/.*")
  list(FILTER dartpy_sources EXCLUDE REGEX ".*/gui/osg/.*")
endif()

# Python binding module name
set(pybind_module dartpy)

# Build a Python extension module:
# pybind11_add_module(<name> [MODULE | SHARED] [EXCLUDE_FROM_ALL]
#                     [NO_EXTRAS] [SYSTEM] [THIN_LTO] source1 [source2 ...])
#
# When building wheels, don't use EXCLUDE_FROM_ALL so the module is built automatically
if(DART_BUILD_WHEELS)
  pybind11_add_module(${pybind_module}
    MODULE
    ${dartpy_headers}
    ${dartpy_sources}
  )
else()
  pybind11_add_module(${pybind_module}
    MODULE
    EXCLUDE_FROM_ALL
    ${dartpy_headers}
    ${dartpy_sources}
  )
endif()

target_include_directories(${pybind_module}
  SYSTEM PUBLIC
    ${PYTHON_INCLUDE_DIRS}
    ${pybind11_INCLUDE_DIRS}
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

set(dartpy_libraries
  dart
  dart-utils
  dart-utils-urdf
)
if(TARGET dart-utils-usd)
  list(APPEND dartpy_libraries dart-utils-usd)
endif()
foreach(dartpy_library ${dartpy_libraries})
  if(NOT TARGET ${dartpy_library})
    message(FATAL_ERROR "Cannot find the library ${dartpy_library}. Please check the configuration of DART.")
  endif()
endforeach()

target_link_libraries(${pybind_module}
  PUBLIC ${dartpy_libraries}
)
if(TARGET dart-optimizer-nlopt)
  target_link_libraries(${pybind_module} PUBLIC dart-optimizer-nlopt)
endif()
if(TARGET dart-collision-bullet)
  target_link_libraries(${pybind_module} PUBLIC dart-collision-bullet)
endif()
if(TARGET dart-collision-ode)
  target_link_libraries(${pybind_module} PUBLIC dart-collision-ode)
endif()
if(TARGET dart-gui-osg)
  target_link_libraries(${pybind_module} PUBLIC dart-gui-osg)
endif()

target_compile_definitions(${pybind_module}
  PRIVATE DARTPY_VERSION_INFO="${DARTPY_VERSION_INFO}"
)

if(DART_PYBIND11_EXTRA_DEFINITIONS)
  target_compile_definitions(${pybind_module}
    PRIVATE ${DART_PYBIND11_EXTRA_DEFINITIONS}
  )
endif()

# Pass DART_BUILD_GUI_OSG to Python bindings so they can conditionally compile OSG support
if(DART_BUILD_GUI_OSG)
  target_compile_definitions(${pybind_module} PRIVATE DART_BUILD_GUI_OSG=1)
else()
  target_compile_definitions(${pybind_module} PRIVATE DART_BUILD_GUI_OSG=0)
endif()

# Remove debug postfix for dartpy
set_target_properties(${pybind_module} PROPERTIES DEBUG_POSTFIX "")

# Emit the dartpy extension directly into the configured runtime directory so
# multi-config generators (MSVC/Xcode) don't append config subfolders.
set(_dartpy_output_dir "${DART_DARTPY_RUNTIME_DIR}")
foreach(_type LIBRARY RUNTIME ARCHIVE)
  set_target_properties(${pybind_module} PROPERTIES
    ${_type}_OUTPUT_DIRECTORY "${_dartpy_output_dir}"
  )
endforeach()
if(CMAKE_CONFIGURATION_TYPES)
  foreach(_config ${CMAKE_CONFIGURATION_TYPES})
    string(TOUPPER ${_config} _config_upper)
    foreach(_type LIBRARY RUNTIME ARCHIVE)
      set_target_properties(${pybind_module} PROPERTIES
        ${_type}_OUTPUT_DIRECTORY_${_config_upper} "${_dartpy_output_dir}"
      )
    endforeach()
  endforeach()
endif()

# Get the path to the bind module
set(PYBIND_MODULE $<TARGET_FILE:${pybind_module}>)

# Custom target to install (copy) the bind module. This target may require
# `sudo` if the destination is a system directory.
set(install_comment "Installing ${pybind_module}...")
if(BUILD_SHARED_LIBS)
  string(CONCAT install_comment
    "${install_comment}\n"
    "NOTE: ${pybind_module} is built against the DART's shared libraries. "
    "Install the shared libraries to be able to import ${pybind_module}."
  )
endif()

# Install the pybind module to the wheel
# For scikit-build-core, we use SKBUILD_PLATLIB_DIR if available,
# otherwise fall back to PYTHON_SITE_PACKAGES for traditional builds
if(DEFINED SKBUILD_PLATLIB_DIR)
  install(TARGETS ${pybind_module}
    LIBRARY DESTINATION "${SKBUILD_PLATLIB_DIR}"
  )
else()
  install(TARGETS ${pybind_module}
    LIBRARY DESTINATION "${PYTHON_SITE_PACKAGES}"
  )
endif()

list(REMOVE_ITEM dartpy_headers
  ${CMAKE_CURRENT_LIST_DIR}/eigen_geometry_pybind.h
  ${CMAKE_CURRENT_LIST_DIR}/eigen_pybind.h
)
list(REMOVE_ITEM dartpy_sources
  ${CMAKE_CURRENT_LIST_DIR}/eigen_geometry_pybind.cpp
)
dart_format_add(${dartpy_headers} ${dartpy_sources})
