# Copyright (c) 2011-2025, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
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

add_library(dart8 STATIC
  body/rigid_body.cpp
  common/diagnostics.cpp
  common/exceptions.cpp
  common/profiling.cpp
  comps/dynamics.cpp
  comps/frame_types.cpp
  comps/joint.cpp
  comps/link.cpp
  comps/multi_body.cpp
  comps/name.cpp
  comps/rigid_body.cpp
  frame/fixed_frame.cpp
  frame/frame.cpp
  frame/free_frame.cpp
  io/binary_io.cpp
  io/serializer.cpp
  multi_body/joint.cpp
  multi_body/link.cpp
  multi_body/multi_body.cpp
  space/state_space.cpp
  space/vector_mapper.cpp
  world.cpp
)

include(dart8_defs)
include(dart8_dependencies)

target_include_directories(dart8
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(dart8 PUBLIC cxx_std_20)

target_link_libraries(dart8
  PUBLIC
    Eigen3::Eigen
    EnTT::EnTT
    spdlog::spdlog
)

target_compile_definitions(dart8
  PUBLIC
    DART_BUILD_SHARED=${DART_BUILD_SHARED_VALUE}
  PRIVATE
    DART_BUILDING_DART8
)

add_library(dart8::dart8 ALIAS dart8)
