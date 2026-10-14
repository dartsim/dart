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

# Private native code for the Python demo catalog and C++ tests; builds with DART_BUILD_GUI=OFF.
add_library(
  dart-demo-models
  STATIC
  ContactArrowLayout.cpp
  WorldHistory.cpp
  WamIkFast.cpp
)
target_link_libraries(dart-demo-models PUBLIC dart)
set_source_files_properties(
  WamIkFast.cpp
  PROPERTIES SKIP_UNITY_BUILD_INCLUSION TRUE COMPILE_OPTIONS "-w"
)
target_include_directories(
  dart-demo-models
  PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/.."
)

if(HAVE_OCTOMAP)
  target_sources(dart-demo-models PRIVATE PointCloudSensor.cpp)
endif()

if(TARGET dart-utils AND TARGET dart-utils-urdf)
  target_sources(
    dart-demo-models
    PRIVATE
      soft_foot_simbicon/SoftFootSimbiconModel.cpp
      atlas_simbicon/Controller.cpp
      atlas_simbicon/State.cpp
      atlas_simbicon/StateMachine.cpp
      atlas_simbicon/TerminalCondition.cpp
  )
  target_link_libraries(dart-demo-models PUBLIC dart-utils dart-utils-urdf)
endif()

file(GLOB_RECURSE demo_model_format_files "*.hpp" "*.cpp")
dart_format_add(${demo_model_format_files})
