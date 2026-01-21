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

# Ensure a Python interpreter/development environment is available. nanobind
# requires `find_package(Python ...)` rather than `Python3`, so call it here
# even though the rest of the project uses `Python3`.*
find_package(Python COMPONENTS Interpreter Development REQUIRED)

# Determine Python site-packages directory if it is not already computed.
if(NOT DEFINED PYTHON_SITE_PACKAGES)
  execute_process(COMMAND ${Python_EXECUTABLE} -c
    "import sysconfig; print(sysconfig.get_path('platlib', scheme='posix_prefix', vars={'base': '', 'platbase': ''}).lstrip('/'))"
    OUTPUT_VARIABLE PYTHON_SITE_PACKAGES_REL
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  set(PYTHON_SITE_PACKAGES "${CMAKE_INSTALL_PREFIX}/${PYTHON_SITE_PACKAGES_REL}")
  set(PYTHON_SITE_PACKAGES ${PYTHON_SITE_PACKAGES} PARENT_SCOPE)
endif()

set(_dartpy_nanobind_min_version 2.9.2)

# Locate nanobind (prefer the copy shipped with the active Python interpreter).
set(_dartpy_nanobind_found FALSE)
if(DART_USE_SYSTEM_NANOBIND)
  find_package(nanobind ${_dartpy_nanobind_min_version} CONFIG REQUIRED)
  set(_dartpy_nanobind_found TRUE)
else()
  if(NOT DEFINED nanobind_DIR)
    execute_process(
      COMMAND ${Python_EXECUTABLE} -m nanobind --cmake_dir
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
  find_package(nanobind ${_dartpy_nanobind_min_version} CONFIG REQUIRED)
  set(_dartpy_nanobind_found TRUE)
endif()

if(NOT _dartpy_nanobind_found)
  message(WARNING "nanobind is required to build dartpy; skipping target.")
  return()
endif()

set(dartpy_headers
  common/composite.hpp
  common/module.hpp
  common/logging.hpp
  common/observer.hpp
  common/repr.hpp
  common/resource.hpp
  common/resource_retriever.hpp
  common/stopwatch.hpp
  common/string.hpp
  common/subject.hpp
  common/uri.hpp
  utils/module.hpp
  utils/urdf_parser.hpp
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
  math/eigen_geometry.hpp
  math/trimesh.hpp
  dynamics/module.hpp
  dynamics/entity.hpp
  dynamics/degree_of_freedom.hpp
  dynamics/frame.hpp
  dynamics/jacobian_node.hpp
  dynamics/shape.hpp
  dynamics/shape_frame.hpp
  dynamics/simple_frame.hpp
  dynamics/inertia.hpp
  dynamics/shape_node.hpp
  dynamics/body_node.hpp
  dynamics/joint.hpp
  dynamics/free_joint.hpp
  dynamics/revolute_joint.hpp
  dynamics/euler_joint.hpp
  dynamics/planar_joint.hpp
  dynamics/translational_joint2d.hpp
  dynamics/translational_joint.hpp
  dynamics/prismatic_joint.hpp
  dynamics/end_effector.hpp
  dynamics/linkage.hpp
  dynamics/screw_joint.hpp
  dynamics/universal_joint.hpp
  dynamics/ball_joint.hpp
  dynamics/weld_joint.hpp
  dynamics/zero_dof_joint.hpp
  dynamics/chain.hpp
  dynamics/meta_skeleton.hpp
  dynamics/hierarchical_ik.hpp
  dynamics/inverse_kinematics.hpp
  dynamics/skeleton.hpp
  collision/module.hpp
  collision/collision_option.hpp
  collision/collision_result.hpp
  collision/raycast.hpp
  collision/collision_detector.hpp
  collision/collision_group.hpp
  simulation/module.hpp
  simulation/world.hpp
  simulation/constraint_solver.hpp
  constraint/module.hpp
  constraint/constraint_base.hpp
  constraint/dynamic_joint_constraint.hpp
  constraint/joint_constraint.hpp
  constraint/joint_coulomb_friction_constraint.hpp
)

set(dartpy_sources
  dartpy.cpp
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
  utils/urdf_parser.cpp
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
  math/eigen_geometry.cpp
  math/tri_mesh.cpp
  dynamics/module.cpp
  dynamics/entity.cpp
  dynamics/degree_of_freedom.cpp
  dynamics/frame.cpp
  dynamics/jacobian_node.cpp
  dynamics/shape.cpp
  dynamics/shape_frame.cpp
  dynamics/simple_frame.cpp
  dynamics/inertia.cpp
  dynamics/shape_node.cpp
  dynamics/body_node.cpp
  dynamics/joint.cpp
  dynamics/free_joint.cpp
  dynamics/revolute_joint.cpp
  dynamics/euler_joint.cpp
  dynamics/planar_joint.cpp
  dynamics/translational_joint2d.cpp
  dynamics/translational_joint.cpp
  dynamics/prismatic_joint.cpp
  dynamics/end_effector.cpp
  dynamics/linkage.cpp
  dynamics/screw_joint.cpp
  dynamics/universal_joint.cpp
  dynamics/ball_joint.cpp
  dynamics/weld_joint.cpp
  dynamics/zero_dof_joint.cpp
  dynamics/chain.cpp
  dynamics/meta_skeleton.cpp
  dynamics/hierarchical_ik.cpp
  dynamics/inverse_kinematics.cpp
  dynamics/skeleton.cpp
  collision/module.cpp
  collision/collision_option.cpp
  collision/collision_result.cpp
  collision/raycast.cpp
  collision/collision_detector.cpp
  collision/collision_group.cpp
  simulation/module.cpp
  simulation/world.cpp
  simulation/constraint_solver.cpp
  constraint/module.cpp
  constraint/constraint_base.cpp
  constraint/dynamic_joint_constraint.cpp
  constraint/joint_constraint.cpp
  constraint/joint_coulomb_friction_constraint.cpp
)

set(dartpy_gui_headers
  gui/module.hpp
  gui/gui.hpp
  gui/utils.hpp
  gui/support_polygon_visual.hpp
)

set(dartpy_gui_sources
  gui/module.cpp
  gui/gui_event_handler.cpp
  gui/world_node.cpp
  gui/interactive_frame.cpp
  gui/viewer_attachment.cpp
  gui/support_polygon_visual.cpp
  gui/drag_and_drop.cpp
  gui/shadow_technique.cpp
  gui/viewer.cpp
  gui/imgui.cpp
)

if(DART_BUILD_GUI)
  list(APPEND dartpy_headers ${dartpy_gui_headers})
  list(APPEND dartpy_sources ${dartpy_gui_sources})
else()
  message(FATAL_ERROR "Building dartpy requires DART_BUILD_GUI=ON")
endif()

nanobind_add_module(dartpy
  NB_STATIC
  ${dartpy_sources}
)

foreach(_nb_target nanobind-static nanobind-static-abi3 nanobind-static-ft)
  if(TARGET ${_nb_target})
    target_compile_options(${_nb_target} PRIVATE "-UNB_COMPACT_ASSERTIONS")
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
      # Silence format warnings from nanobind sources that become errors under
      # our global -Werror. These are third-party sources.
      target_compile_options(
        ${_nb_target}
        PRIVATE
          -Wno-format
          -Wno-format-security
          -Wno-error
          -Wno-error=format
          -Wno-error=format-security
      )
    endif()
  endif()
endforeach()

target_include_directories(dartpy
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_BINARY_DIR}
)

if(BULLET_INCLUDE_DIR)
  target_include_directories(dartpy SYSTEM PRIVATE ${BULLET_INCLUDE_DIR})
endif()

set(dartpy_libraries
  dart
  dart-utils
  dart-utils-urdf
)
foreach(lib ${dartpy_libraries})
  if(TARGET ${lib})
    target_link_libraries(dartpy PRIVATE ${lib})
    foreach(prop INCLUDE_DIRECTORIES INTERFACE_INCLUDE_DIRECTORIES)
      get_target_property(_lib_includes ${lib} ${prop})
      if(_lib_includes)
        target_include_directories(dartpy PRIVATE ${_lib_includes})
      endif()
    endforeach()
  else()
    message(FATAL_ERROR "Required DART library not found: ${lib}")
  endif()
endforeach()

if(TARGET dart-gui)
  target_link_libraries(dartpy PRIVATE dart-gui)
  foreach(prop INCLUDE_DIRECTORIES INTERFACE_INCLUDE_DIRECTORIES)
    get_target_property(_gui_includes dart-gui ${prop})
    if(_gui_includes)
      target_include_directories(dartpy PRIVATE ${_gui_includes})
    endif()
  endforeach()
endif()

# Surface a version string to Python.
target_compile_definitions(dartpy
  PRIVATE
    DARTPY_VERSION_INFO="${DART_VERSION}"
)
target_compile_options(dartpy PRIVATE "-UNB_COMPACT_ASSERTIONS")

set_target_properties(dartpy PROPERTIES OUTPUT_NAME "_dartpy")

# Copy pure-Python helpers into the build tree so tests can import them
foreach(_py_file __init__.py _naming.py _layout.py)
  configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/${_py_file}
    ${CMAKE_CURRENT_BINARY_DIR}/${_py_file}
    COPYONLY
  )
  add_custom_command(
    TARGET dartpy POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CMAKE_CURRENT_SOURCE_DIR}/${_py_file}
            ${CMAKE_CURRENT_BINARY_DIR}/${_py_file}
    COMMENT "Updating dartpy ${_py_file}"
  )
endforeach()

if(DEFINED DART_DARTPY_RUNTIME_DIR)
  set(_dartpy_output_dir "${DART_DARTPY_RUNTIME_DIR}")
else()
  set(_dartpy_output_dir "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}")
endif()

foreach(_type LIBRARY RUNTIME ARCHIVE)
  if(_dartpy_output_dir)
    set_target_properties(dartpy PROPERTIES
      ${_type}_OUTPUT_DIRECTORY "${_dartpy_output_dir}"
    )
  endif()
endforeach()
if(CMAKE_CONFIGURATION_TYPES AND _dartpy_output_dir)
  foreach(_config ${CMAKE_CONFIGURATION_TYPES})
    string(TOUPPER ${_config} _config_upper)
    foreach(_type LIBRARY RUNTIME ARCHIVE)
      set_target_properties(dartpy PROPERTIES
        ${_type}_OUTPUT_DIRECTORY_${_config_upper} "${_dartpy_output_dir}"
      )
    endforeach()
  endforeach()
endif()

if(DEFINED SKBUILD_PLATLIB_DIR)
  install(TARGETS dartpy
    LIBRARY DESTINATION "${SKBUILD_PLATLIB_DIR}/dartpy"
  )
  install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/__init__.py
                ${CMAKE_CURRENT_SOURCE_DIR}/_naming.py
                ${CMAKE_CURRENT_SOURCE_DIR}/_layout.py
    DESTINATION "${SKBUILD_PLATLIB_DIR}/dartpy"
  )
elseif(DEFINED PYTHON_SITE_PACKAGES)
  install(TARGETS dartpy
    LIBRARY DESTINATION "${PYTHON_SITE_PACKAGES}/dartpy"
  )
  install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/__init__.py
                ${CMAKE_CURRENT_SOURCE_DIR}/_naming.py
                ${CMAKE_CURRENT_SOURCE_DIR}/_layout.py
    DESTINATION "${PYTHON_SITE_PACKAGES}/dartpy"
  )
endif()

dart_format_add(${dartpy_headers} ${dartpy_sources})
