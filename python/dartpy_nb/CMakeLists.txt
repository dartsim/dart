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

# Determine Python site-packages directory if it is not already computed.
if(NOT DEFINED PYTHON_SITE_PACKAGES)
  execute_process(COMMAND ${Python3_EXECUTABLE} -c
    "import sysconfig; print(sysconfig.get_path('platlib', scheme='posix_prefix', vars={'base': '', 'platbase': ''}).lstrip('/'))"
    OUTPUT_VARIABLE PYTHON_SITE_PACKAGES_REL
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  set(PYTHON_SITE_PACKAGES "${CMAKE_INSTALL_PREFIX}/${PYTHON_SITE_PACKAGES_REL}")
  set(PYTHON_SITE_PACKAGES ${PYTHON_SITE_PACKAGES} PARENT_SCOPE)
endif()

# Locate nanobind (prefer the copy shipped with the active Python interpreter).
set(_dartpy_nb_nanobind_found FALSE)
if(DART_USE_SYSTEM_NANOBIND)
  find_package(nanobind CONFIG REQUIRED)
  set(_dartpy_nb_nanobind_found TRUE)
else()
  if(NOT DEFINED nanobind_DIR)
    execute_process(
      COMMAND ${Python3_EXECUTABLE} -m nanobind --cmake_dir
      OUTPUT_VARIABLE nanobind_ROOT
      OUTPUT_STRIP_TRAILING_WHITESPACE
      RESULT_VARIABLE nanobind_NOTFOUND
    )
    if(nanobind_NOTFOUND)
      message(WARNING "nanobind was not found via 'python -m nanobind --cmake_dir'. "
        "Install it with 'pip install nanobind' or set nanobind_DIR explicitly.")
      return()
    endif()
    list(APPEND CMAKE_PREFIX_PATH "${nanobind_ROOT}")
    set(nanobind_DIR "${nanobind_ROOT}" CACHE PATH "nanobind CMake package path" FORCE)
  endif()
  find_package(nanobind CONFIG REQUIRED)
  set(_dartpy_nb_nanobind_found TRUE)
endif()

if(NOT _dartpy_nb_nanobind_found)
  message(WARNING "nanobind is required to build dartpy_nb; skipping target.")
  return()
endif()

set(dartpy_nb_headers
  common/composite.hpp
  common/module.hpp
  common/logging.hpp
  common/observer.hpp
  common/resource.hpp
  common/resource_retriever.hpp
  common/stopwatch.hpp
  common/string.hpp
  common/subject.hpp
  common/uri.hpp
  utils/module.hpp
  utils/dart_loader.hpp
  utils/resource_retriever.hpp
  utils/skel_parser.hpp
  utils/sdf_parser.hpp
  utils/mjcf_parser.hpp
  optimizer/module.hpp
  optimizer/function.hpp
  optimizer/problem.hpp
  optimizer/solver.hpp
  optimizer/gradient_descent.hpp
  math/module.hpp
  math/constants.hpp
  math/geometry.hpp
  math/random.hpp
)

set(dartpy_nb_sources
  dartpy_nb.cpp
  common/composite.cpp
  common/module.cpp
  common/logging.cpp
  common/observer.cpp
  common/resource.cpp
  common/resource_retriever.cpp
  common/stopwatch.cpp
  common/string.cpp
  common/subject.cpp
  common/uri.cpp
  utils/module.cpp
  utils/dart_loader.cpp
  utils/resource_retriever.cpp
  utils/skel_parser.cpp
  utils/sdf_parser.cpp
  utils/mjcf_parser.cpp
  optimizer/module.cpp
  optimizer/function.cpp
  optimizer/problem.cpp
  optimizer/solver.cpp
  optimizer/gradient_descent.cpp
  math/module.cpp
  math/constants.cpp
  math/geometry.cpp
  math/random.cpp
)

nanobind_add_module(dartpy_nb
  NB_STATIC
  ${dartpy_nb_sources}
)

target_include_directories(dartpy_nb
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_BINARY_DIR}
)

set(dartpy_nb_libraries
  dart
  dart-utils
  dart-utils-urdf
)
foreach(lib ${dartpy_nb_libraries})
  if(TARGET ${lib})
    target_link_libraries(dartpy_nb PRIVATE ${lib})
  else()
    message(FATAL_ERROR "Required DART library not found: ${lib}")
  endif()
endforeach()

# Surface a version string to Python.
target_compile_definitions(dartpy_nb
  PRIVATE DARTPY_NB_VERSION_INFO="${DART_VERSION}"
)

set_target_properties(dartpy_nb PROPERTIES OUTPUT_NAME "dartpy_nb")

if(DEFINED DART_DARTPY_NB_RUNTIME_DIR)
  set(_dartpy_nb_output_dir "${DART_DARTPY_NB_RUNTIME_DIR}")
else()
  set(_dartpy_nb_output_dir "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}")
endif()

foreach(_type LIBRARY RUNTIME ARCHIVE)
  if(_dartpy_nb_output_dir)
    set_target_properties(dartpy_nb PROPERTIES
      ${_type}_OUTPUT_DIRECTORY "${_dartpy_nb_output_dir}"
    )
  endif()
endforeach()
if(CMAKE_CONFIGURATION_TYPES AND _dartpy_nb_output_dir)
  foreach(_config ${CMAKE_CONFIGURATION_TYPES})
    string(TOUPPER ${_config} _config_upper)
    foreach(_type LIBRARY RUNTIME ARCHIVE)
      set_target_properties(dartpy_nb PROPERTIES
        ${_type}_OUTPUT_DIRECTORY_${_config_upper} "${_dartpy_nb_output_dir}"
      )
    endforeach()
  endforeach()
endif()

if(DEFINED SKBUILD_PLATLIB_DIR)
  install(TARGETS dartpy_nb
    LIBRARY DESTINATION "${SKBUILD_PLATLIB_DIR}"
  )
elseif(DEFINED PYTHON_SITE_PACKAGES)
  install(TARGETS dartpy_nb
    LIBRARY DESTINATION "${PYTHON_SITE_PACKAGES}"
  )
endif()

dart_format_add(${dartpy_nb_headers} ${dartpy_nb_sources})
