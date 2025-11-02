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

set(dartpy7_sources
  dartpy7.cpp
)

if(DART_BUILD_DARTPY7_GUI)
  list(APPEND dartpy7_sources
    gui/module.cpp
  )
endif()

nanobind_add_module(
  dartpy7
  MODULE
  ${dartpy7_sources}
)

if(TARGET nanobind)
  target_compile_options(nanobind PRIVATE -Wno-error -Wno-error=sign-compare)
endif()

if(TARGET nanobind-static)
  target_compile_options(nanobind-static PRIVATE -Wno-error -Wno-error=sign-compare)
endif()

target_include_directories(dartpy7
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(dartpy7
  PRIVATE
    dart7::dart7
)

if(DART_BUILD_DARTPY7_GUI)
  target_link_libraries(dartpy7
    PRIVATE
      dart7::gui
  )
endif()

target_compile_definitions(dartpy7
  PRIVATE
    DART_BUILD_DART7_GUI=$<BOOL:${DART_BUILD_DART7_GUI}>
    DART_BUILD_DARTPY7_GUI=$<BOOL:${DART_BUILD_DARTPY7_GUI}>
)

set_target_properties(dartpy7 PROPERTIES OUTPUT_NAME "dartpy7")
