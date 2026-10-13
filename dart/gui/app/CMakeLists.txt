#
# Copyright (c) 2011, The DART development contributors
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
#

option(
  DART_BUILD_DEMOS_MEMORY_DIAGNOSTICS
  "Build memory diagnostics into the private GUI demo app host"
  OFF
)
file(GLOB app_sources "*.hpp" "*.cpp")
if(NOT DART_BUILD_DEMOS_MEMORY_DIAGNOSTICS)
  list(FILTER app_sources EXCLUDE REGEX "memory_diagnostics.*\\.(hpp|cpp)$")
endif()
add_library(dart-gui-app STATIC EXCLUDE_FROM_ALL ${app_sources})
set_target_properties(dart-gui-app PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(dart-gui-app PUBLIC cxx_std_17)
target_link_libraries(dart-gui-app PUBLIC dart-gui dart-demo-models)
target_include_directories(
  dart-gui-app
  PRIVATE "${PROJECT_SOURCE_DIR}/examples"
)
target_compile_definitions(
  dart-gui-app
  PUBLIC
    DART_BUILD_DEMOS_MEMORY_DIAGNOSTICS=$<BOOL:${DART_BUILD_DEMOS_MEMORY_DIAGNOSTICS}>
  PRIVATE DART_DEMOS_HAS_ALLOCATOR_MEMORY_LAYOUT=1
)
if(WIN32 AND DART_BUILD_DEMOS_MEMORY_DIAGNOSTICS)
  target_link_libraries(dart-gui-app PRIVATE Psapi)
endif()
