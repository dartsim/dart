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

#ifndef DART_GUI_DETAIL_RENDER_CONTEXT_HPP_
#define DART_GUI_DETAIL_RENDER_CONTEXT_HPP_

#include <utils/Entity.h>

#include <array>

#include <cstddef>

namespace filament {

class Camera;
class Engine;
class Renderer;
class Scene;
class SwapChain;
class View;

} // namespace filament

namespace dart::gui {

struct RunOptions;

} // namespace dart::gui

namespace dart::gui::detail {

inline constexpr std::size_t kFilamentViewportSlots = 4u;

struct FilamentRenderContext
{
  ::filament::Engine* engine = nullptr;
  ::filament::Renderer* renderer = nullptr;
  ::filament::SwapChain* swapChain = nullptr;
  ::filament::View* view = nullptr;
  ::filament::Scene* scene = nullptr;
  ::filament::Camera* camera = nullptr;
  std::array<::filament::View*, kFilamentViewportSlots> views{};
  std::array<::filament::Camera*, kFilamentViewportSlots> cameras{};
  std::array<utils::Entity, kFilamentViewportSlots> cameraEntities{};
  std::size_t activeViewCount = 1u;
  utils::Entity cameraEntity;
  // Name of the graphics backend actually selected (static string literal).
  const char* backendName = "opengl";
};

FilamentRenderContext createFilamentRenderContext(
    const dart::gui::RunOptions& options, void* nativeWindow);

// Most recent valid GPU frame duration in milliseconds from the Filament
// frame-info history, or 0 if unavailable (e.g. NOOP backend, or results not
// ready yet).
double latestGpuFrameMs(const FilamentRenderContext& context);

bool beginFilamentFrame(FilamentRenderContext& context);

bool shouldSkipRenderedWorkAfterFrameSkip(
    const FilamentRenderContext& context, bool headless);

void renderFilamentViews(
    FilamentRenderContext& context, ::filament::View* overlayView);

void endFilamentFrame(FilamentRenderContext& context);

void destroyFilamentRenderContext(FilamentRenderContext& context);

} // namespace dart::gui::detail

#endif // DART_GUI_DETAIL_RENDER_CONTEXT_HPP_
