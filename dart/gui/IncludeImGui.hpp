/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef DART_GUI_INCLUDEIMGUI_HPP_
#define DART_GUI_INCLUDEIMGUI_HPP_

#include <dart/config.hpp>

// ImGui header inclusion handling for different configurations
//
// When DART_USE_SYSTEM_IMGUI=OFF:
//   - Build tree: ImGui fetched via FetchContent, headers in
//   build/_deps/imgui-src
//   - Install tree: Headers installed to include/ and include/backends/
//   - Include directories configured in cmake/DARTFindDependencies.cmake
//
// When DART_USE_SYSTEM_IMGUI=ON:
//   - Package managers install ImGui in different locations:
//     * conda-forge: include/imgui.h, include/imgui_impl_opengl2.h
//     * vcpkg: include/imgui.h, include/imgui/backends/imgui_impl_opengl2.h
//     * apt (varies): include/imgui/imgui.h,
//     include/imgui/backends/imgui_impl_opengl2.h
//   - cmake/Findimgui.cmake handles these variations and adds proper include
//   paths
//
// Both configurations end up with compatible include paths, so we try standard
// patterns in order of preference:
#if __has_include(<imgui.h>)
  #include <imgui.h>
#elif __has_include(<imgui/imgui.h>)
  #include <imgui/imgui.h>
#else
  #error "Could not find imgui.h - check your ImGui installation"
#endif

#if __has_include(<imgui_impl_opengl2.h>)
  #include <imgui_impl_opengl2.h>
#elif __has_include(<backends/imgui_impl_opengl2.h>)
  #include <backends/imgui_impl_opengl2.h>
#elif __has_include(<imgui/backends/imgui_impl_opengl2.h>)
  #include <imgui/backends/imgui_impl_opengl2.h>
#else
  #error "Could not find imgui_impl_opengl2.h - check your ImGui installation"
#endif

#endif // DART_GUI_INCLUDEIMGUI_HPP_
